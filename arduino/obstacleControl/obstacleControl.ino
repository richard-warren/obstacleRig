#include <digitalWriteFast.h>


// USER SETTINGS

// pin assignments
const int stepPin = 4;
const int stepDirPin = 5;
const int encoderPinA = 2; // encoder pins must be set to 2 and 3 for Arduino Uno because hardware interrupts exist on these pins
const int encoderPinB = 3;
const int waterPin = 7;
const int obstaclePin = 13; // signals whether the obstacle is engaged... this is sent to an arduino that controls the obstacle servo
const int motorOffPin = 8; // turns on stepper motor driver
const int startLimitPin = 9; // signal is LOW when engaged
const int stopLimitPin = 10; // signal is LOW when engaged
const int servoSwingTime = 200; // ms, approximate amount of time it takes for the osbtacle to pop out // this is used as a delay bewteen the obstacle reaching the end of the track and it coming back, to avoid it whacking the guy in the butt!

// other user settings
const int speedLookupLength = 100;
const float rampResolution = .05; // < 1, smaller values are longer ramps
volatile int stepperDelays[speedLookupLength];


volatile int state = 2; // 1: no platform movement, no obstaacles, 2: platform movement, no obstacles, 3: platform movement and obstacles
const float rewardRotations = 6;
const int microStepping = 16; // only (1/microStepping) steps per pulse // this should correspond to the setting on the stepper motor driver, which is set by 3 digital inputs
const int endPositionBuffer = 50 * microStepping; // motor stops endPositionBuffer steps before the beginning and end of the track
const int waterDuration = 80; // milliseconds
const long maxStepperSpeed = 12000 * long(microStepping); // (Hz) servo will move this fast to desired positions // maximum, unloaded appears to be around 2000 // NOTE: right now even a delay of microsecond fails to keep up with very fast speeds, so execution of other code in takeStep is rate limiting factor...
const int motorSteps = 200;
const int encoderSteps = 2880; // 720cpr * 4
const int timingPulleyRad = 15.2789; // mm
const float wheelRad = 95.25;
int obstacleLocations[] = {2*encoderSteps, 4*encoderSteps, rewardRotations*encoderSteps*2}; // expressed in wheel ticks // the last element is a hack... the index goes up and the wheel position will never reach the last value, which is the desired behavior
volatile float slowSpeedMultiplier = .25;

// initializations
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
  
  digitalWriteFast(stepPin, LOW);
  digitalWrite(stepDirPin, stepDir);
  digitalWriteFast(waterPin, LOW);
  digitalWriteFast(motorOffPin, LOW);
  digitalWriteFast(obstaclePin, LOW);

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
  Serial.begin(1200);
  Serial.println("1: rewards only");
  Serial.println("2: platform, no obstacles");
  Serial.println("3: platform, with obstacles");
  Serial.println(stepperDelays[speedLookupLength-1]);;
  
  // initialize track limits and move to starting position
  getStartLimit();
  getEndLimit();
  getStartLimit();
  stepperDelayInd = 0;
  takeStep(stepperStartPosition, slowSpeedMultiplier); // move back to stepperStartPosition
  digitalWriteFast(motorOffPin, HIGH);
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
        digitalWriteFast(motorOffPin, LOW); // engages stepper motor driver
        stepperDelayInd = 0; // start at lowest velocity
      }
      
      if (state==3){
        digitalWriteFast(obstaclePin, HIGH); // engages servo motor
      }
    }
  // disengage:
  }else if (stepperTicks >= stepperStopPosition){
    obstacleEngaged = false;
    digitalWriteFast(obstaclePin, LOW);
    obstacleInd++;
    
    // return stage to starting position
    delay(servoSwingTime); // delay before returning the platform to avoid whacking the mouse in the butt
    getStartLimit();
    stepperDelayInd = 0;
    takeStep(stepperStartPosition, slowSpeedMultiplier);
    digitalWriteFast(motorOffPin, HIGH); // disengage stepper motor driver
  }
     

  // give water if reward location reached
  if (wheelTicksTemp > rewardPosition){
    wheelTicksTemp = 0;
    noInterrupts();
    wheelTicks = 0;
    interrupts();

    obstacleInd = 0;
    digitalWriteFast(waterPin, HIGH);
    delay(waterDuration);
    digitalWriteFast(waterPin, LOW);

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
  digitalWrite(stepDirPin, (stepsToTake<0));

  for (int i = 0; i < abs(stepsToTake); i++){

    // the two initial computation below take ~60 microseconds, thus rate limiting step speed...
    // also, microseconds of stepPin pulse near 1, so in danger of being to short for driver?
    
    // increment motor speed
    stepperDelayInd = min(stepperDelayInd + rampResolution, speedLookupLength-1);

    // get speed index
    stepDelay = stepperDelays[int(stepperDelayInd)] / speedMultiplier;
    
    // take step
    digitalWriteFast(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWriteFast(stepPin, LOW);
    delayMicroseconds(stepDelay);
  }

  stepperTicks += stepsToTake;
}




// read rotary encoder
void encoder_isr() {
    static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
    static uint8_t enc_val = 0;
    
    enc_val = enc_val << 2;
    enc_val = enc_val | ((PIND & 0b1100) >> 2);
 
    wheelTicks = wheelTicks - lookup_table[enc_val & 0b1111];
}



void getStartLimit(){

  stepperDelayInd = 0; // start at lowest velocity
  
  // find start limit
  while (digitalReadFast(startLimitPin)){
    takeStep(-1, slowSpeedMultiplier);
  }
  stepperTicks = 0;
  
}



void getEndLimit(){

  stepperDelayInd = 0; // start at lowest velocity

  // find stop limit
  while (digitalReadFast(stopLimitPin)){
    takeStep(1, slowSpeedMultiplier);
  }
  stepperStopPosition = stepperTicks - endPositionBuffer;
  
}


