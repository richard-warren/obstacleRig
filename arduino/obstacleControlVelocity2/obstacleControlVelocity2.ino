// OBSTACLE CONTROL



// pin assignments
#define stepPin         4
#define stepDirPin      5
#define encoderPinA     20   // don't change encoderPins, because direct port manipulation is used that assumes 20 and 21 are used for encoder pins
#define encoderPinB     21
#define waterPin        7
#define obstaclePin     13   // signals whether the obstacle is engaged... this is sent to an arduino that controls the obstacle servo
#define motorOffPin     8    // turns on stepper motor driver
#define startLimitPin   9    // signal is LOW when engaged
#define stopLimitPin    10   // signal is LOW when engaged
#define obsLightPin     2    // controls when the light for the obstacle turns on




// other user settings
const int servoSwingTime = 300; // ms, approximate amount of time it takes for the osbtacle to pop out // this is used as a delay bewteen the obstacle reaching the end of the track and it coming back, to avoid it whacking the guy in the butt!
volatile int state = 2; // 1: no platform movement, no obstaacles, 2: platform movement, no obstacles, 3: platform movement and obstacles
char* conditionNames[] = {"rewards only", "platform, no obstacles", "platform, with obstacles"};
const int speedLookupLength = 100;
const float rampResolution = .2; // < 1, smaller values are longer ramps
volatile int stepperDelays[speedLookupLength];
volatile float rewardRotations = 9.01;
const int microStepping = 16; // only (1/microStepping) steps per pulse // this should correspond to the setting on the stepper motor driver, which is set by 3 digital inputs
const int endPositionBuffer = 75 * microStepping; // motor stops endPositionBuffer steps before the beginning and end of the track
const int waterDuration = 80; // milliseconds
const int maxStepperRPS = 12;
const int motorSteps = 200;
const int encoderSteps = 2880; // 720cpr * 4
const float timingPulleyRad = 15.2789; // mm
const float wheelRad = 95.25;
int obstacleLocations[] = {1.5*encoderSteps, 4.5*encoderSteps, 7.5*encoderSteps, rewardRotations*encoderSteps*20}; // expressed in wheel ticks // the last element is a hack... the index goes up and the wheel position will never reach the last value, which is the desired behavior
volatile float slowSpeedMultiplier = .4;




// initializations
volatile int userInput;
const long maxStepperSpeed = maxStepperRPS * 200 * long(microStepping);
volatile int wheelTicks = 0;
volatile int wheelTicksTemp = 0; // this variable temporarily copies wheelTicks in the main code to avoid having to access it multiple times, potentially colliding with its access in the interrupt
volatile int obstacleInd = 0; // keeps track of which obstacle is being delivered for each reward trial
volatile int stepperTicks = 0;
const int stepperStartPosition = 50 * microStepping;
volatile int stepperStopPosition; // value to be determined by call to initializeLimits in setup()
volatile int targetStepperTicks = stepperStartPosition;
volatile int stepsToTake = 0; // when driving the motor, stepsToTake is how many motor ticks required to get to target position
volatile int rewardPosition = rewardRotations * encoderSteps; // expressed in wheelTicks
const double conversionFactor = (wheelRad / timingPulleyRad) * (float(motorSteps) / encoderSteps) * microStepping; // this converts from analogRead reading of wheel encoder to desired number of steps in stepper driver
const double delayConversionFactor = ((2*PI*timingPulleyRad) / (motorSteps*microStepping)) / ((2*PI*wheelRad) / encoderSteps); // multiply time interval between wheel encoder tics by this to get interval between stepper tics that gives the same velocity
volatile bool stepDir = HIGH;
volatile bool obstacleEngaged = false;
volatile float stepperDelayInd = 0;
volatile int pulseDuration;
volatile int stepDelay;
const int faultyStepperTics = 25000; // if stepper has taken more than this many steps it is porbably frozen, and software will re-calibrate
volatile long currentMicros = 0;
volatile long lastMicros = 0;
volatile int deltaMicros = 0;
volatile int targetStepDelay = 0;
volatile int forgivenSteps = 0;




void setup() {
  
  // prepare ins and outs
  pinMode(stepPin, OUTPUT);
  pinMode(stepDirPin, OUTPUT);
  pinMode(waterPin, OUTPUT);
  pinMode(motorOffPin, OUTPUT);
  pinMode(obstaclePin, OUTPUT);
  pinMode(startLimitPin, INPUT_PULLUP);
  pinMode(stopLimitPin, INPUT_PULLUP);
  pinMode(obsLightPin, OUTPUT);
  
  digitalWrite(stepPin, LOW);
  digitalWrite(stepDirPin, stepDir);
  digitalWrite(waterPin, LOW);
  digitalWrite(motorOffPin, LOW);
  digitalWrite(obstaclePin, LOW);
  digitalWrite(obsLightPin, LOW);
  

  // initialize encoder hardware interrupt
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), encoder_isr, CHANGE);

  
  // initialize lookup table for stepper speeds
  for (int i=0; i<speedLookupLength; i++){
    long tempSpeed = (i+1) * (maxStepperSpeed / speedLookupLength);
    stepperDelays[i] = (pow(10, 6) / tempSpeed) / 2;
  }
  pulseDuration = stepperDelays[speedLookupLength];

  
  // begin serial communication
  Serial.begin(115200);
  printMenuAndSettings();
  
  
  // initialize track limits and move to starting position
  initializeLimits();
}




void loop(){

  // check for user input
  getUserInput();
  
  
  // store current wheelTicks value with interrupts disabled
  noInterrupts();
  wheelTicksTemp = wheelTicks;
  interrupts();



  
  // check if obstacle should be engaged or disengaged  
  
  // engage:
  if (!obstacleEngaged){
    if (wheelTicksTemp >= obstacleLocations[obstacleInd]){
      
      if ((state==2) || (state==3)){
        obstacleEngaged = true;
        digitalWrite(motorOffPin, LOW); // engages stepper motor driver
        stepperDelayInd = 0; // start at lowest velocity
        startTracking();
      }
      
      if (state==3){
        digitalWrite(obstaclePin, HIGH); // engages servo motor
        digitalWrite(obsLightPin, HIGH);
      }
    }
  
  // disengage:
  }else if (stepperTicks >= stepperStopPosition){
    obstacleEngaged = false;
    digitalWrite(obstaclePin, LOW);
    digitalWrite(obsLightPin, LOW);
    obstacleInd++;
    
    // reset the stepper driver before going home, which prevents the driver from freezing due to fast de-acceleration at the end of the track
    digitalWrite(motorOffPin, HIGH); // disengage stepper motor driver
    delay(50);
    digitalWrite(motorOffPin, LOW);
    
    // return stage to starting position
    delay(servoSwingTime); // delay before returning the platform to avoid whacking the mouse in the butt
    getStartLimit();
    stepperDelayInd = 0;
    if (state==3){
      digitalWrite(obstaclePin, HIGH);
    }
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
    targetStepperTicks = ((wheelTicksTemp - obstacleLocations[obstacleInd]) * conversionFactor)  + stepperStartPosition - forgivenSteps;
    targetStepperTicks = constrain(targetStepperTicks, stepperStartPosition, stepperStopPosition);

    stepsToTake = targetStepperTicks - stepperTicks;
    if (stepsToTake!=0){
      takeStep(stepsToTake, 1);
    }
  }
}




// move stepper one step in stepDirection
void startTracking(){

  // set motor direction
  digitalWrite(stepDirPin, HIGH);

  targetStepDelay = deltaMicros * delayConversionFactor;
  stepDelay = stepperDelays[0];

  while (stepDelay >= targetStepDelay){
    
    // increment motor speed
    stepperDelayInd = min(stepperDelayInd + rampResolution, speedLookupLength-1);

    // get speed index
    stepDelay = stepperDelays[round(stepperDelayInd)];

    // take step
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
  }

  // set the current position to the target position
  noInterrupts();
  wheelTicksTemp = wheelTicks;
  interrupts();
  targetStepperTicks = ((wheelTicksTemp - obstacleLocations[obstacleInd]) * conversionFactor)  + stepperStartPosition;
  forgivenSteps = targetStepperTicks - stepperTicks;
}




// move stepper one step in stepDirection
void takeStep(int stepsToTake, float speedMultiplier){

  // set motor direction
  digitalWrite(stepDirPin, (stepsToTake>0));
  delayMicroseconds(1); // TCM2100 stepper motor driver requires 20 nanoseconds between stepDirectionPin change and stepPin input // the driver was messing up occasionally, so hopefully this will fix the problem

  for (int i = 0; i < abs(stepsToTake); i++){

    
    // increment motor speed
    stepperDelayInd = min(stepperDelayInd + rampResolution, speedLookupLength-1);

    // get speed index
    stepDelay = stepperDelays[round(stepperDelayInd)] / speedMultiplier;

    // take step
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
  }

  stepperTicks += stepsToTake;

  // check that the motor is not going way off the fucking rails (ie the stepper driver has stalled)
  if (abs(stepperTicks) > faultyStepperTics){

    interrupts();
    
    Serial.println("stepper driver is resetting...");
    delay(500);
    
    // reset stepper driver
    digitalWrite(motorOffPin, HIGH); // disengage stepper motor driver
    delay(50);
    digitalWrite(motorOffPin, LOW);

    noInterrupts();

    // recalibrate track and reset parameters
    initializeLimits();
    
    noInterrupts();
    wheelTicks = 0;
    obstacleInd = 0;
    interrupts();   
    
  }
}




// read rotary encoder
void encoder_isr() {
    
    currentMicros = micros();
    deltaMicros = currentMicros - lastMicros;
    lastMicros = currentMicros;
    
    static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
    static uint8_t enc_val = 0;
    
    enc_val = enc_val << 2;
    enc_val = enc_val | ((REG_PIOB_PDSR & (0b11 << 12)) >> 12); // read all pins from port B, and bitmask to get just inputs 12 and 13 from port B, which correspond to Due pins 20 and 21
 
    wheelTicks = wheelTicks - lookup_table[enc_val & 0b1111];

}



void getStartLimit(){

  stepperDelayInd = 0; // start at lowest velocity
  
  // find start limit
  while (digitalRead(startLimitPin)){
    takeStep(-1, slowSpeedMultiplier);
  }
  stepperTicks = 0;
  
}



void getEndLimit(){

  stepperDelayInd = 0; // start at lowest velocity

  // find stop limit
  while (digitalRead(stopLimitPin)){
    takeStep(1, slowSpeedMultiplier);
  }
  stepperStopPosition = stepperTicks - endPositionBuffer;

}



void initializeLimits(){

  getStartLimit();
  getEndLimit();
  getStartLimit();
  stepperDelayInd = 0; // start at lowest velocity
  takeStep(stepperStartPosition, slowSpeedMultiplier); // move back to stepperStartPosition
  digitalWrite(motorOffPin, HIGH);
  
}




void printMenuAndSettings(){

  // print settings
  Serial.println("CURRENT SETTINGS:");
  Serial.print("condition: ");
  Serial.println(conditionNames[state-1]);
  Serial.print("reward rotations: ");
  Serial.println(rewardRotations);
  Serial.println("");
  delay(500);

  // print menu
  Serial.println("MENU:");
  Serial.println("1: set condition to rewards only");
  Serial.println("2: set condition to platform, no obstacles");
  Serial.println("3: set condition to platform, with obstacles");
  Serial.println("4: set reward rotation number");
  Serial.println("5: get current settings");
  Serial.println("");
  delay(500);
  
}




void getUserInput(){

  // check for user input
  if (Serial.available()){
    
    userInput = Serial.parseInt();
    
    switch (userInput){

      // set condition to rewards only
      case 1:
        state = userInput;
        printMenuAndSettings();
        digitalWrite(obstaclePin, LOW);
        break;
        
      // set condition to platform movement, no obstacles
      case 2:
        state = userInput;
        printMenuAndSettings();
        digitalWrite(obstaclePin, LOW);
        break;
        
      // set condition to platform movement with obstacles
      case 3:
        state = userInput;
        printMenuAndSettings();
        digitalWrite(obstaclePin, HIGH);
        break;
        
      // enter reward rotation number
      case 4:
        Serial.print("enter reward rotations...\n\n");
        while (Serial.available() == 0)  {}
        rewardRotations = Serial.parseFloat();
        printMenuAndSettings();
        rewardPosition = rewardRotations * encoderSteps;
        break;
        
      // print settings
      case 5:
        printMenuAndSettings();
        break;
    }
  }
}

