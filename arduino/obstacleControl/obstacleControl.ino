/* OBSTACLE CONTROL

todo:

replace runningmedian with delay list, and cmopoute delay by summing across list...
reset stepper motor after trial...
add jitter and other things things that need to be initialized after rewards and/or obs offs... // should i still offset obs?
add default args to takeSteps
only check for user input when obs not on, and disable interrupts when user input is being collected...
add description of units: meters, wheelTics, motorTics
integrate startTracking and takeSteps?
*/



// INITIALIZATIONS

// imports
#include "config.h"
#include <RunningMedian.h>  // https://playground.arduino.cc/Main/RunningMedian/

// state variables
bool isObsOn = false;        // whether state is water only, or water with obstacles (does NOT keep track of whether obstacle is CURRENTLY moving)
bool isObsTracking = false;  // whether obs is currently tracking wheel movements
bool stepDir = HIGH;         // direction in which stepper motor is moving
int obsInd = 0;              // which obstacle is next

// user input
char inputChar;              // user input for characters
int inputInt;                // user input for ints
int inputFloat;              // user input for floats
char inputBuffer[100];       // user input buffer

// unit conversions
const double mPerWheelTic = (2*PI*wheelRad) / encoderSteps;                         // meters for tic of the stepper motor
const double mPerMotorTic = (2*PI*timingPulleyRad) / (motorSteps*microStepping);    // meters for tic of the stepper motor
const double motorTicsPerWheelTic = mPerWheelTic / mPerMotorTic;                    // stepper motor tics per tic of wheel rotary encoder

// positions
int wheelTics = 0;           // tics of wheel rotary encoder
int wheelTicsTemp = 0;       // this variable temporarily copies wheelTicks in the main code to avoid having to access it multiple times, potentially colliding with its access in the interrupt
int startingWheelTics;       // wheel tics at the moment obstacle starts tracking the wheel (after initial velocity matching period)
int motorTics = 0;           // tics of stepper motor
int startingMotorTics;       // motor tics at the moment obstacle starts tracking the wheel (after initial velocity matching period)
int targetMotorTics;         // how many tics should the motor have moved to match the movement of the wheel

// speed
float motorSpeed;            // (m/s) speed of stepper motor
int motorDelay;              // (microseconds) delay between motor steps corresponding to desired motorSpeed
float wheelSpeed;            // (m/s) speed of wheel
long currentMicros = 0;      // 'Micros' variables are used to measure wheel velocity, estimated using time between successive wheel tics
long lastMicros = 0;
int deltaMicros = 0;
RunningMedian deltaMicroSmps = RunningMedian(speedSamples);  // running list of microsecond intervals between wheel tics // used to estimate wheel velocity



void setup() {
  
  // begin serial communication
  Serial.begin(9600);
  while (!Serial) {}; // wait for serial port to connect
//  printMenu();
  
  // prepare ins and outs
  pinMode(stepPin, OUTPUT);
  pinMode(stepDirPin, OUTPUT);
  pinMode(waterPin, OUTPUT);
  pinMode(motorOffPin, OUTPUT);
  pinMode(obsOutPin, OUTPUT);
  pinMode(startLimitPin, INPUT_PULLUP);
  pinMode(stopLimitPin, INPUT_PULLUP);
  pinMode(obsLightPin1, OUTPUT);
  pinMode(obsLightPin2, OUTPUT);
  pinMode(obsTrackingPin, OUTPUT);
  
  digitalWrite(stepPin, LOW);
  digitalWrite(stepDirPin, stepDir);
  digitalWrite(waterPin, LOW);
  digitalWrite(motorOffPin, LOW);
  digitalWrite(obsOutPin, LOW);
  digitalWrite(obsLightPin1, LOW);
  digitalWrite(obsLightPin2, LOW);
  digitalWrite(obsTrackingPin, LOW);

  // initialiez random seed
  randomSeed(analogRead(0));
  
  // initialize encoder hardware interrupt
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), encoder_isr, CHANGE);
  
  initializeLimits();
}




void loop(){

  // check for user input
  getUserInput();
  
  
  // wheel position update
  noInterrupts();
  wheelTicsTemp = wheelTics;
  interrupts();

  
  // obstacle engage
  if (!isObsTracking & (wheelTicsTemp >= (obsLocations[obsInd]/mPerWheelTic))){
    if (debug){Serial.println("engaging obstacle");}
    
    // ramp up obstacle velocity to match wheel velocity
    startTracking();
    
    // turn on obstacle light
    if(random(0,100) < obsLightProbability*100.0){
      digitalWrite(obsLightPin1, HIGH);
      digitalWrite(obsLightPin2, HIGH);
    }
  }
  
  
  // obstacle disengage
  else if (isObsTracking & motorTics >= (trackEndPosition/mPerMotorTic)){
    if (debug){Serial.println("disengaging obstacle");}

    // todo: should we slow down here first?
    findStartLimit(callibrationSpeedMax); // find start limit again
    // todo: move to start position?
  }

  
  // water delivery
  else if (wheelTicsTemp >= (waterDistance/mPerWheelTic)){
    if (debug){Serial.println("delivering water");}
    giveWater();
  }
  
  
  // obstacle position update
  else if (isObsTracking){
    targetMotorTics = (wheelTicsTemp - startingWheelTics)*motorTicsPerWheelTic + startingMotorTics;
    targetMotorTics = constrain(targetMotorTics, 0, trackEndPosition/mPerMotorTic);
    takeSteps(targetMotorTics - motorTics, 1, obsSpeedMax, obsSpeedMax, obsSpeedMax, false);
  }
}
