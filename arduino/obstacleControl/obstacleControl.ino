// USER SETTINGS

// pin assignments
const int stepPin = 4;
const int stepDirPin = 5;
const int encoderPinA = 20; // don't change encoderPins, because direct port manipulation is used that assumes 20 and 21 are used for encoder pins
const int encoderPinB = 21;
const int waterPin = 7;
const int obstaclePin = 13; // signals whether the obstacle is engaged... this is sent to an arduino that controls the obstacle servo
const int motorOffPin = 8; // turns on stepper motor driver
const int startLimitPin = 9; // signal is LOW when engaged
const int stopLimitPin = 10; // signal is LOW when engaged
const int servoSwingTime = 500; // ms, approximate amount of time it takes for the osbtacle to pop out // this is used as a delay bewteen the obstacle reaching the end of the track and it coming back, to avoid it whacking the guy in the butt!

// other user settings
volatile int state = 2; // 1: no platform movement, no obstaacles, 2: platform movement, no obstacles, 3: platform movement and obstacles
const int speedLookupLength = 100;
const float rampResolution = .1; // < 1, smaller values are longer ramps
volatile int stepperDelays[speedLookupLength];
const float rewardRotations = 6;
const int microStepping = 16; // only (1/microStepping) steps per pulse // this should correspond to the setting on the stepper motor driver, which is set by 3 digital inputs
const int endPositionBuffer = 50 * microStepping; // motor stops endPositionBuffer steps before the beginning and end of the track
const int waterDuration = 80; // milliseconds
const int maxStepperRPS = 12;
const int motorSteps = 200;
const int encoderSteps = 2880; // 720cpr * 4
const int timingPulleyRad = 15.2789; // mm
const float wheelRad = 95.25;
int obstacleLocations[] = {2*encoderSteps, 4*encoderSteps, rewardRotations*encoderSteps*2}; // expressed in wheel ticks // the last element is a hack... the index goes up and the wheel position will never reach the last value, which is the desired behavior
volatile float slowSpeedMultiplier = .4;

// initializations
const long maxStepperSpeed = maxStepperRPS * 200 * long(microStepping);
volatile int wheelTicks = 0;
volatile int wheelTicksTemp = 0; // this variable temporarily copies wheelTicks in the main code to avoid having to access it multiple times, potentially colliding with its access in the interrupt
volatile int obstacleInd = 0; // keeps track of which obstacle is being delivered for each reward trial
volatile int stepperTicks = 0;
const int stepperStartPosition = endPositionBuffer;
volatile int stepperStopPosition; // value to be determined by call to initializeLimits in setup()
volatile int targetStepperTicks = stepperStartPosition;
volatile int stepsToTake = 0; // when driving the motor, stepsToTake is how many motor ticks required to get to target position
const int rewardPosition = rewardRotations * encoderSteps; // expressed in wheelTicks
const float conversionFactor = (wheelRad / timingPulleyRad) * (float(motorSteps) / encoderSteps) * microStepping; // this converts from analogRead reading of wheel encoder to desired number of steps in stepper driver
volatile bool stepDir = HIGH;
volatile bool obstacleEngaged = false;
volatile float stepperDelayInd = 0;
volatile int pulseDuration;
volatile int stepDelay;





void setup() {
  
  // prepare ins and outs
  pinMode(stepPin, OUTPUT);
  pinMode(stepDirPin, OUTPUT);
  pinMode(waterPin, OUTPUT);
  pinMode(motorOffPin, OUTPUT);
  pinMode(obstaclePin, OUTPUT);
  pinMode(startLimitPin, INPUT_PULLUP);
  pinMode(stopLimitPin, INPUT_PULLUP);
  
  digitalWrite(stepPin, LOW);
  digitalWrite(stepDirPin, stepDir);
  digitalWrite(waterPin, LOW);
  digitalWrite(motorOffPin, LOW);
  digitalWrite(obstaclePin, LOW);

  // initialize encoder hhardware interrupt
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), encoder_isr, CHANGE);

  // initialize lookup table for stepper speeds
  for (int i=0; i<speedLookupLength; i++){
    long tempSpeed = (i+1) * (maxStepperSpeed / speedLookupLength);
    stepperDelays[i] = (pow(10, 6) / tempSpeed) / 2;
  }
  pulseDuration = stepperDelays[speedLookupLength];

  // begin serial communication
  Serial.begin(9600);
//  delay(1000);
  Serial.println("1: rewards only");
  Serial.println("2: platform, no obstacles");
  Serial.println("3: platform, with obstacles");
  Serial.print("min stepper delay: ");
  Serial.println(stepperDelays[speedLookupLength-1]);;
  
  // initialize track limits and move to starting position
  getStartLimit();
  getEndLimit();
  getStartLimit();
  stepperDelayInd = 0;
  takeStep(stepperStartPosition, slowSpeedMultiplier); // move back to stepperStartPosition
  digitalWrite(motorOffPin, HIGH);
}




void loop(){

  // store current wheelTicks value with interrupts disabled
  noInterrupts();
  wheelTicksTemp = wheelTicks;
  interrupts();

  
  // check if obstacle should be engaged
  // engage:
  if (!obstacleEngaged){
    if (wheelTicksTemp >= obstacleLocations[obstacleInd]){
      
      if ((state==2) || (state==3)){
        obstacleEngaged = true;
        digitalWrite(motorOffPin, LOW); // engages stepper motor driver
        stepperDelayInd = 0; // start at lowest velocity
      }
      
      if (state==3){
        digitalWrite(obstaclePin, HIGH); // engages servo motor
      }
    }
  // disengage:
  }else if (stepperTicks >= stepperStopPosition){
    obstacleEngaged = false;
    digitalWrite(obstaclePin, LOW);
    obstacleInd++;
    
    // return stage to starting position
    delay(servoSwingTime); // delay before returning the platform to avoid whacking the mouse in the butt
    getStartLimit();
    stepperDelayInd = 0;
    takeStep(stepperStartPosition, slowSpeedMultiplier);
    digitalWrite(motorOffPin, HIGH); // disengage stepper motor driver
  }
     

  // give water if reward location reached
  if (wheelTicksTemp > rewardPosition){
    wheelTicksTemp = 0;
    noInterrupts();
    wheelTicks = 0;
    interrupts();

    obstacleInd = 0;
    digitalWrite(waterPin, HIGH);
    delay(waterDuration);
    digitalWrite(waterPin, LOW);

  }
  
  
  // compute and move to target stepper position
  if (obstacleEngaged){
    targetStepperTicks = ((wheelTicksTemp - obstacleLocations[obstacleInd]) * conversionFactor)  + stepperStartPosition;  
    targetStepperTicks = constrain(targetStepperTicks, stepperStartPosition, stepperStopPosition);

    stepsToTake = targetStepperTicks - stepperTicks;
    if (stepsToTake!=0){
      takeStep(stepsToTake, 1);
    }
  }
  
  
  // check for user input
  if (Serial.available()){
    state = Serial.read() - 48;
  }
}



// move stepper one step in stepDirection
void takeStep(int stepsToTake, float speedMultiplier){

  // set motor direction
  digitalWrite(stepDirPin, (stepsToTake>0));
  delayMicroseconds(1); // TCM2100 stepper motor driver requires 20 nanoseconds between stepDirectionPin change and stepPin input // the driver was messing up occasionally, so hopefully this will fix the problem

  for (int i = 0; i < abs(stepsToTake); i++){

//    long micros1 = micros();
    
    // increment motor speed
    stepperDelayInd = min(stepperDelayInd + rampResolution, speedLookupLength-1);

    // get speed index
    stepDelay = stepperDelays[round(stepperDelayInd)] / speedMultiplier;

//    long micros2 = micros();
//    Serial.println(micros2 - micros1);
    
    // take step
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
  }

  stepperTicks += stepsToTake;
}




// read rotary encoder
void encoder_isr() {
    static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
    static uint8_t enc_val = 0;
    
    enc_val = enc_val << 2;
//    enc_val = enc_val | ((PIND & 0b1100) >> 2); this work for arduino uno if encoder pins are set to 2 and 3
    enc_val = enc_val | ((REG_PIOB_PDSR & (0b11 << 12)) >> 12); // read all pins from port B, and bitmask to get just inputs 12 and 13 from port B, which correspond to Due pins 20 and 21
 
    wheelTicks = wheelTicks - lookup_table[enc_val & 0b1111];
}



void getStartLimit(){

  noInterrupts();

  stepperDelayInd = 0; // start at lowest velocity
  
  // find start limit
  while (digitalRead(startLimitPin)){
    takeStep(-1, slowSpeedMultiplier);
  }
  stepperTicks = 0;

  interrupts();
  
}



void getEndLimit(){

  noInterrupts();

  stepperDelayInd = 0; // start at lowest velocity

  // find stop limit
  while (digitalRead(stopLimitPin)){
    takeStep(1, slowSpeedMultiplier);
  }
  stepperStopPosition = stepperTicks - endPositionBuffer;

  interrupts();
  
}


