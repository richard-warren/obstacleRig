/* OBSTACLE CONTROL

todo:
improve acceleration computation
try microstepping adjustment
startTracking

add jitter and other things things that need to be initialized after rewards and/or obs offs... // should i still offset obs?
only check for user input when obs not on, and disable interrupts when user input is being collected...
document, including description of units used
avoid hack with having one too many entries in obsLocations
*/



// INITIALIZATIONS

// imports
#include "config.h"

// state variables
bool isObsOn = false;            // whether state is water only, or water with obstacles (does NOT keep track of whether obstacle is CURRENTLY moving)
bool isObsTracking = false;      // whether obs is currently tracking wheel movements
bool stepDir = HIGH;             // direction in which stepper motor is moving
int obsInd = 0;                  // which obstacle is next
  
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

// motor speed
float motorSpeed;            // (m/s) speed of stepper motor
int motorDelay;              // (microseconds) delay between motor steps corresponding to desired motorSpeed

// wheel speed
long lastMicros = 0;
const int dtSz = int(wheelSpeedDistance/mPerWheelTic);
int wheelDts[dtSz];          // (microseconds) running list of intervals between wheel rotary encoder tics
int dtInd = 0;               // index for wheelDeltas
int dtSum = 0;               // sum of values in wheelDeltas


void setup() {
  
  // begin serial communication
  Serial.begin(115200);
  while (!Serial) {}; // wait for serial port to connect
  
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

  // initialize random seed
  randomSeed(analogRead(0));
  
  // initialize encoder hardware interrupt
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), encoder_isr, CHANGE);
  
  initializeLimits();
  printMenu();
}




void loop(){

  // check for user input
  getUserInput();
  
  
  // wheel position update
  noInterrupts();
  wheelTicsTemp = wheelTics;
  interrupts();

  
  // obstacle engage
  if (isObsOn && !isObsTracking && (wheelTicsTemp>=(obsLocations[obsInd]/mPerWheelTic))){
    
    // ramp up obstacle velocity to match wheel velocity
    digitalWrite(motorOffPin, LOW); // engages stepper motor driver
    isObsTracking = true;
    startTracking();
    
    // turn on obstacle light
    if(random(0,100) < obsLightProbability*100.0){
      digitalWrite(obsLightPin1, HIGH);
      digitalWrite(obsLightPin2, HIGH);
    }
  }
  
  
  // obstacle disengage
  else if (isObsTracking && motorTics>=((trackEndPosition-obsStopPos)/mPerMotorTic)){
    
    // turn off obstacle
    isObsTracking = false;
    digitalWrite(obsLightPin1, LOW);
    digitalWrite(obsLightPin2, LOW);
    obsInd++;
    
    // reset stepper motor driver while disengaging obstacle
    digitalWrite(motorOffPin, HIGH);
    digitalWrite(obsOutPin, LOW);  // swing obstacle out of the way
    delay(servoSwingTime);
    digitalWrite(motorOffPin, LOW);
    
    // find the start limit switch and move back to starting position
    findStartLimit(callibrationSpeeds[0], callibrationSpeeds[1]);
    digitalWrite(obsOutPin, HIGH);  // swing obstacle out again
    takeAcceleratingSteps(obsStartPos/mPerMotorTic, obsAcceleration, callibrationSpeeds[0], callibrationSpeeds[1]);  // moving back to start position
    digitalWrite(motorOffPin, HIGH);
  }

  
  // water delivery
  else if (wheelTicsTemp >= (waterDistance/mPerWheelTic)){
    Serial.println("delivering water");
    giveWater();
  }
  
  
  // obstacle position update
  else if (isObsTracking){
    targetMotorTics = (wheelTicsTemp-startingWheelTics)*motorTicsPerWheelTic + startingMotorTics;
    targetMotorTics = constrain(targetMotorTics, 0, trackEndPosition/mPerMotorTic)  - motorTics;
    if (targetMotorTics!=0){takeSteps(targetMotorTics);}
  }

  Serial.println(getWheelSpeed());
}
