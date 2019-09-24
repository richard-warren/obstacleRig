
/* OBSTACLE CONTROL

Richard Warren - 190924

GENERAL:
This code is for the Arduion Due that is the master controller for the obstacle experiments.
The mouse runs on a wheel until waterDistance meters have been traveled, at which point water is delivered.
Obstacles pop out at obsLocations and match the speed of the wheel.
Settings can be adjusted in 'config.h'.

UNITS:
Wheel speed is measured with a digital rotary encoder that emmits a pulse every mPerWheelTic meters of wheel movement.
Wheel displacement is measured in 'wheelTics', i.e. the number of pulses from the rotary encoder.
The stepper motor controlling the obstacles is also moved discretely using TTLs. The position of the motor is measured in 'motorTics'.
wheelTics are measured relative to the last delivered reward, and motorTics are measured relative to the position of the limit switch at the beginning of the track.
Events (water reward, obstacle engagement) occur at positions specified in meters relative to last reward. Converting between meters and tics can be done using mPerWheelTic and mPerMotorTic conversion factors.

OBSTCLE TRACKING:
To match the movement of the obstacle to the speed of the wheel, the number of wheelTics since obstacle engagement is compared to the number of motorTics since engagement.
Both are converted to meters. The motor is moved a discrete number of steps that minimizes the discrepancy between these positions.

ACCELERATION:
The obstacle cannot achieve high velocities immediately from stand-still. Therefore, when the obstacle starts tracking, the velocity is increased with contant acceleration until the obstacle and wheel velocity are matched (see startTracking()).
At that point, the tracking procedure described above begins. Nominal obstacle velocity is controlled by computing the inter-motor-step interval necessary to achieve desired speeds.
The algebra necessary to compute subsequent velocities at constant acceleration takes ~27 microseconds, which drastcailly slows down the code.
Therefore, 'speeds' and 'delays' lookup tables store a list of linearly increasing speeds (constant acceleration) and their associated delays.
Accelerating and decelerating then becomes as simple as moving up and down in this list. The length of the list is a function of the velocity range and acceleration.
At slow accelerations and long velocity intervals, the list can become to long, at which point the code will complain and ask you to adjust the parameters.

A subtle but important point: The code drastically overestimates the speed of obstacle movement. It assumes the interstep delay it delivers (with delayMicroseconds()) is the effective delay.
However, processing delays in the code (on the order of 10-20 microSeconds) decrease the effective speed by about 40 percent.
Therefore, the speeds and acceleration specified in config.h are nominal and not actual speeds.
The discrepency between nominal and actual speed becomes important in startTracking(), where the software attempts to match the obstacle speed (measured poorly) and the wheel speed (measured effectively).
To compensate for this discrepancy, the nominal motor speed is multiplied by 'speedCompensation' in 'startTracking()'.
If the obstacle speed is too fast or too slow when startTracking() completes (when acceleration desists) you should increase or decrease 'speedCompensation'.

MEASURING WHEEL SPEED:
The wheel rotary encoder triggers a hardware interrupt (encoder_isr()) that must run very quickly to avoid delaying the code:
In the isr, microseconds elapsed from last wheel tic is recorded in an array (wheelDts). An index keeps track of the position in the array, and it wraps around to the beginning of the array when the maximum location is reached.
Computing the speed requires summing these elements, which requires redundant computation if performed on every wheel speed check. Therefore, a running sum is computed:
On every wheel tic, the new interval is added to the sum, and the oldest interval is subtracted from the sum.

TIMING:
At the highest wheel speeds, the motor step interval approaches 10 microseconds. Therefore, even small computational delays can severely limit the maximum accomplishable speed.
Measruing (in microseconds) the duration of certain events was important for troubleshooting the codes. Here are some notes on the timing of certain pieces of code:
-27us to update the motor speed and delay algebraically. This approach was abandoned in favor of the lookup table for this reason, which only takes 3us to update.
-in void main(), it takes 2us to perform the algebra necessary to compare wheel and motor Tics on the fly. A faster approach would be to convert these ahead of time, but that would introduce more variables. I opted for this slightly slower, but simpler implementation.
-getWheelSpeed() takes 12us. Therefore, startTracking does not check the wheel speed after every accelerating step, but instead after every 'stepsBtwnChecks' steps, which is still a very fast rate.

CALIBRATION:
The position of the obstacle is inferred from the commands sent to the motor. However, steps will inevitably lost as the motor fails to faithfully implement the commands sent to it.
Therefore, the position is recallibrated at the end of every obstacle trial: The obstacle moves back until it touches the start limit switch, and motorTics is set to 0 at this point.
The acceleration profile in the speeds lookup table is used during callibration. It allows the obstacle to accelerate as is starts moving from standstill.
Also, it is used to decelerate as it approaches the estimated position of the limit switch, avoiding a loud bang as it hits the switch at high velocity.

*/



// INITIALIZATIONS

// imports
#include "config.h"

// state variables
bool isObsOn = false;        // whether state is water only, or water with obstacles (does NOT keep track of whether obstacle is CURRENTLY moving)
bool isObsTracking = false;  // whether obs is currently tracking wheel movements
bool stepDir = HIGH;         // direction in which stepper motor is moving
int obsInd = 0;              // which obstacle is next
  
// user input
char inputChar;                // user input for characters
char inputBuffer[100];         // user input buffer
int inputInt;                  // user input for ints
float inputFloat;              // user input for floats
bool inputDetected = false;    // whether there is a user input that awaits processing
bool notGettingInput = true;   // keeps track of whether user input is currently being collected

// unit conversions
const double mPerWheelTic = (2*PI*wheelRad) / encoderSteps;                         // meters for tic of the stepper motor
const double mPerMotorTic = (2*PI*timingPulleyRad) / (motorSteps*microStepping);    // meters for tic of the stepper motor
const double motorTicsPerWheelTic = mPerWheelTic / mPerMotorTic;                    // stepper motor tics per tic of wheel rotary encoder

// positions
volatile int wheelTics = 0;  // tics of wheel rotary encoder
int wheelTicsTemp = 0;       // this variable temporarily copies wheelTicks in the main code to avoid having to access it multiple times, potentially colliding with its access in the interrupt
int startingWheelTics;       // wheel tics at the moment obstacle starts tracking the wheel (after initial velocity matching period)
int motorTics = 0;           // tics of stepper motor
int startingMotorTics;       // motor tics at the moment obstacle starts tracking the wheel (after initial velocity matching period)
int targetMotorTics;         // how many tics should the motor have moved to match the movement of the wheel
float obsLocation;           // keeps track of starting location of next obstacle

// motor speed
const int bufferSize = 10000;
float speeds[bufferSize];        // lookup table for motor speeds
int delays[bufferSize];          // lookup table for motor step intervals that correspond to speeds
int speedInd = 0;
int maxSpeedInd = 0;
enum accel {ACCELERATE, DECELERATE};
enum dir {FORWARD=true, REVERSE=false};

// wheel speed
volatile long lastMicros = 0;
const int dtSz = int(wheelSpeedDistance/mPerWheelTic);
volatile int wheelDts[dtSz];          // (microseconds) running list of intervals between wheel rotary encoder tics
volatile int dtInd = 0;               // index for wheelDeltas
volatile int dtSum = 0;               // sum of values in wheelDeltas




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
  pinMode(obsLightPinDig, OUTPUT);
  pinMode(obsLightPin1, OUTPUT);
  pinMode(obsLightPin2, OUTPUT);
  pinMode(obsTrackingPin, OUTPUT);
  pinMode(touchSensorOnPin, OUTPUT);
  
  digitalWrite(stepPin, LOW);
  digitalWrite(stepDirPin, stepDir);
  digitalWrite(waterPin, LOW);
  digitalWrite(motorOffPin, LOW);
  digitalWrite(obsOutPin, LOW);
  digitalWrite(obsTrackingPin, LOW);
  digitalWrite(touchSensorOnPin, LOW);
  switchObsLight(LOW);

  
  // initialize encoder hardware interrupt
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), encoder_isr, CHANGE);


  // initialize lookup table for stepper speeds
  speeds[0] = obsSpeedStart;
  delays[0] = round(getMotorDelay(speeds[0]));
  float prevDelay = getMotorDelay(speeds[0]);
  float maxSpeed = max(obsSpeedMax, callibrationSpeed);
  
  while (speeds[maxSpeedInd]<maxSpeed && maxSpeedInd<(bufferSize-1)){
    maxSpeedInd++;
    speeds[maxSpeedInd] = speeds[maxSpeedInd-1] + prevDelay*pow(10,-6)*obsAcceleration;
    delays[maxSpeedInd] = round(getMotorDelay(speeds[maxSpeedInd]));
    prevDelay = getMotorDelay(speeds[maxSpeedInd]);
  }
  if (speeds[maxSpeedInd]>maxSpeed){maxSpeedInd--;}  // move one below index at which maxSpeed is surpassed
  
  
  // final initializations
  randomSeed(analogRead(0));
  calibrateLimits();
}




void loop(){

  // check for user input
  getUserInput();
  if (!isObsTracking){
    processUserInput();
  }
  
  
  // wheel position update
  noInterrupts();
  wheelTicsTemp = wheelTics;
  interrupts();

  
  // obstacle engage
  if (isObsOn && !isObsTracking && (wheelTicsTemp>=(obsLocation/mPerWheelTic))){
    
    // ramp up obstacle velocity to match wheel velocity
    digitalWrite(motorOffPin, LOW);  // engage stepper motor driver
    digitalWrite(obsTrackingPin, HIGH);
    isObsTracking = true;
    startTracking();

    // turn on obstacle light
    if(random(0,100) < obsLightProbability*100.0){
      switchObsLight(HIGH);
    }

    // turn on touch sensor
    digitalWrite(touchSensorOnPin, HIGH);
  }

  
  // obstacle disengage
  else if (isObsTracking && motorTics>=((trackEndPosition-obsStopPos)/mPerMotorTic)){

    // turn off touch sensor
    digitalWrite(touchSensorOnPin, LOW);
    
    // turn off obstacle
    isObsTracking = false;
    digitalWrite(obsTrackingPin, LOW);
    switchObsLight(LOW);
    obsInd++;
    
    // reset stepper motor driver while disengaging obstacle
    digitalWrite(motorOffPin, HIGH);
    digitalWrite(obsOutPin, LOW);  // swing obstacle out of the way
    delay(servoSwingTime);
    digitalWrite(motorOffPin, LOW);
    
    // find the start limit switch and move back to starting position
    findStartLimit(obsSpeedStart, obsSpeedStop, callibrationSpeed);
    digitalWrite(obsOutPin, HIGH);  // swing obstacle out again
    goToStartPos();
    digitalWrite(motorOffPin, HIGH);

    // determine next obstacle location
    if (obsInd<(sizeof(obsLocations)/4)){  // sizeof(obsLocations)/4 gives number of obstacles, as there are 4 bytes per float
      obsLocation = obsLocations[obsInd] + getJitter(obsLocationJitter);
    }else{
      obsLocation = waterDistance*2;  // this makes the obstacle unreachable until after reward delivery
    }
  }

  
  // water delivery
  else if (wheelTicsTemp >= (waterDistance/mPerWheelTic)){
    giveWater();
  }
  
  
  // obstacle position update
  else if (isObsTracking){
    targetMotorTics = (wheelTicsTemp-startingWheelTics)*motorTicsPerWheelTic + startingMotorTics;
    targetMotorTics = constrain(targetMotorTics, 0, (trackEndPosition-obsStopPos)/mPerMotorTic+1) - motorTics;  // +1 is needed because the motor tics must EXCEED the threshold for the obstacle to be disengaged
    if (targetMotorTics!=0){
      takeStepsFast(targetMotorTics);
    }
  }
}
