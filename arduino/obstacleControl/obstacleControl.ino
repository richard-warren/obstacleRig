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
const int servoSwingTime = 500; // ms, approximate amount of time it takes for the osbtacle to pop out // this is used as a delay bewteen the obstacle reaching the end of the track and it coming back, to avoid it whacking the guy in the butt!

// other user settings
const float rewardRotations = 6;
const int microStepping = 16; // only (1/microStepping) steps per pulse // this should correspond to the setting on the stepper motor driver, which is set by 3 digital inputs
const int endPositionBuffer = 50 * microStepping; // motor stops endPositionBuffer steps before the beginning and end of the track
const int waterDuration = 80; // milliseconds
const int stepperSpeed = 800 * microStepping; // (Hz) servo will move this fast to desired positions // maximum, unloaded appears to be around 2000
const int motorSteps = 200;
const int encoderSteps = 2880; // 720cpr * 4
const int timingPulleyRad = 15.2789; // mm
const float wheelRad = 95.25;
const int obstacleLocations[] = {2*encoderSteps, 4*encoderSteps, rewardRotations*encoderSteps*2}; // expressed in wheel ticks // the last element is a hack... the index goes up and the wheel position will never reach the last value, which is the desired behavior

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
const int pulseDuration = (pow(10, 6) / stepperSpeed) / 2; // wait this many microseconds before sending successive stepper pulses // this is constrained by max stepper frequency
volatile bool stepDir = HIGH;
volatile bool obstacleEngaged = false;



void setup() {
//  Serial.begin(115200);

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

  // initialize track limits and move to starting position
  initializeLimits();
  digitalWrite(motorOffPin, HIGH);
}




void loop(){

  // display variables
//  Serial.print("wheelTicks: ");
//  Serial.println(wheelTicks);

  // store current wheelTicks value with interrupts disabled
  noInterrupts();
  wheelTicksTemp = wheelTicks;
  interrupts();

  
  // check if obstacle should be engaged
  // engage:
  if (!obstacleEngaged){
    if (wheelTicksTemp >= obstacleLocations[obstacleInd]){
      obstacleEngaged = true;
      digitalWrite(motorOffPin, LOW); // engages stepper motor driver
      digitalWrite(obstaclePin, HIGH);
    }
  // disengage:
  }else if (stepperTicks >= stepperStopPosition){
    obstacleEngaged = false;
    digitalWrite(obstaclePin, LOW);
    obstacleInd++;
    
    // return stage to starting position
    targetStepperTicks = stepperStartPosition;
    stepsToTake = targetStepperTicks - stepperTicks;
    delay(servoSwingTime); // delay before returning the platform to avoid whacking the mouse in the butt
    takeStep(stepsToTake);
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
  
  
  // compute target stepper position
  if (obstacleEngaged){
    targetStepperTicks = ((wheelTicksTemp - obstacleLocations[obstacleInd]) * conversionFactor)  + stepperStartPosition;  
    targetStepperTicks = constrain(targetStepperTicks, stepperStartPosition, stepperStopPosition);

    stepsToTake = targetStepperTicks - stepperTicks;
    if (stepsToTake!=0){
      takeStep(stepsToTake);
    }
  }  
}



// move stepper one step in stepDirection
void takeStep(int stepsToTake){
  digitalWrite(stepDirPin, (stepsToTake<0));

  for (int i = 0; i < abs(stepsToTake); i++){
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(pulseDuration);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(pulseDuration);
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



// initialize motor track limits
void initializeLimits(){

  noInterrupts();

  // find start limit
  while (digitalRead(startLimitPin)){
    takeStep(-1);
  }
  stepperTicks = 0;
  takeStep(stepperStartPosition);

  // find stop limit
  while (digitalRead(stopLimitPin)){
    takeStep(1);
  }
  stepperStopPosition = stepperTicks - endPositionBuffer;

  // move back to stepperStartPosition
  takeStep(-stepperTicks + stepperStartPosition);

  interrupts();
  
}



