#include <digitalWriteFast.h>
#include <Servo.h>



// USER SETTINGS

// pin assignments
const int stepPin = 4;
const int stepDirPin = 5;
const int encoderPinA = 2;
const int encoderPinB = 3;
const int waterPin = 7;
const int servoPin = 9; // it's possible this should only be 9 or 10...
const int motorOnPin = 8; // turns on stepper motor driver

// other user settings
const float rewardRotations = 10;
const int obsEngagedPosition = 30;
const int obsDisengagedPosition = obsEngagedPosition + 90;
const int stepperStartPosition = 100; // position at which the obstacle will appear
const int stepperStopPosition = 1600; // eventually replace this with start and stop sensors...
const int waterDuration = 80; // milliseconds
const int microStepping = 2; // only (1/microStepping) steps per pulse // this should correspond to the setting on the stepper motor driver, which is set by 3 digital inputs
const int stepperSpeed = 600 * microStepping; // (Hz) servo will move this fast to desired positions // maximum, unloaded appears to be around 2000
const int motorSteps = 200;
const int encoderSteps = 720;
const int timingPulleyRad = 36; // mm
const float wheelRad = 95.25;
const int obstacleLocations[] = {3*encoderSteps, 6*encoderSteps, rewardRotations*encoderSteps*2}; // expressed in wheel ticks // the last element is a hack... the index goes up and the wheel position will never reach the last value, which is the desired behavior
//volatile long lastMicros = 0;
//volatile long currentMicros = 0;


// initializations
volatile int wheelTicks = 0;
volatile int obstacleInd = 0; // keeps track of which obstacle is being delivered for each reward trial
volatile int stepperTicks = 0;
volatile int targetStepperTicks = stepperStartPosition;
volatile int stepsToTake = 0; // when driving the motor, stepsToTake is how many motor ticks required to get to target position
const int rewardPosition = rewardRotations * encoderSteps; // expressed in wheelTicks
const float conversionFactor = (wheelRad / timingPulleyRad) * (float(motorSteps) / encoderSteps) * microStepping; // this converts from analogRead reading of wheel encoder to desired number of steps in stepper driver
const int pulseDuration = (pow(10, 6) / stepperSpeed) / 2; // wait this many microseconds before sending successive stepper pulses // this is constrained by max stepper frequency
volatile bool encoderBState;
volatile bool stepDir = HIGH;
volatile bool obstacleEngaged = false;
Servo obstacleServo;



void setup() {
//  Serial.begin(115200);

  // prepare ins and outs
  pinMode(stepPin, OUTPUT);
  pinMode(stepDirPin, stepDir);
  pinMode(waterPin, OUTPUT);
  pinMode(motorOnPin, OUTPUT);
  
  digitalWrite(stepPin, LOW);
  digitalWrite(stepDirPin, LOW);
  digitalWrite(waterPin, LOW);
  digitalWrite(motorOnPin, HIGH);

  // initialize timer interrupts
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderInterrupt, RISING);

  // initialize servo
  obstacleServo.attach(servoPin);
  obstacleServo.write(obsDisengagedPosition);

  // move stepper to starting position
  takeStep(stepperStartPosition);
  digitalWrite(motorOnPin, LOW);
}




void loop(){

  // display variables
//  Serial.print("wheelTicks:");
//  Serial.print(wheelTicks);
//  Serial.print("targetStepperTicks:");
//  currentMicros = micros();
//  Serial.println(currentMicros - lastMicros);
//  lastMicros = currentMicros;
//  Serial.println(postPulseDelay);

  
  // check if obstacle should be engaged
  // engage:
  if (!obstacleEngaged){
    if (wheelTicks > obstacleLocations[obstacleInd]){
      obstacleEngaged = true;
      digitalWrite(motorOnPin, HIGH);
      obstacleServo.write(obsEngagedPosition);
    }
  // disengage:
  }else if (stepperTicks > stepperStopPosition){
    obstacleEngaged = false;
    obstacleServo.write(obsDisengagedPosition);
    obstacleInd++;
    
    targetStepperTicks = stepperStartPosition;
    stepsToTake = targetStepperTicks - stepperTicks;
    takeStep(stepsToTake);
    digitalWrite(motorOnPin, LOW);
  }
     

  // give water if reward location reached
  if (wheelTicks>rewardPosition){
    wheelTicks = 0;
    obstacleInd = 0;
    digitalWrite(waterPin, HIGH);
    delay(waterDuration);
    digitalWrite(waterPin, LOW);

  }
  
  
  // compute target stepper position
  if (obstacleEngaged){
    targetStepperTicks = ((wheelTicks - obstacleLocations[obstacleInd]) * conversionFactor)  + stepperStartPosition;  
    targetStepperTicks = max(targetStepperTicks, 0);

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
void encoderInterrupt(){
  encoderBState = digitalReadFast(encoderPinB);
  wheelTicks += encoderBState ? -1 : 1;
}


