
// pin assignments
#define stepPin            4    // steps to drive stepper motor
#define stepDirPin         5    // stepper motor direction
#define motorOffPin        8    // turns on/off stepper motor driver
#define waterPin           7    // solenoid that drives water delivery
#define obsOutPin          13   // signals whether the obstacle flipped outwards // this is sent to an arduino that controls the obstacle servo
#define obsTrackingPin     3    // when the obstacle is tracking the mouse's position
#define startLimitPin      9    // limit switch at start of track // signal is LOW when engaged
#define stopLimitPin       10   // limit switch at end of track // signal is LOW when engaged
#define obsLightPin1       2    // controls when the light for the obstacle turns on
#define obsLightPin2       6    // controls when the light for the obstacle turns on
#define touchSensorOnPin   12   // powers the touch sensor only when obs is within range of paws
#define encoderPinA        20   // pin for wheel rotary encoder (don't change, because direct port manipulation is used that assumes 20 and 21 are used)
#define encoderPinB        21   // pin for wheel rotary encoder (don't change, because direct port manipulation is used that assumes 20 and 21 are used)


// general
const bool debug = true;

// positions
float waterDistance = 5.4;                       // (m) distance between rewards
const float obsLocations[] = {.9, 2.7, 4.5};     // (m) positions at which obstacles are engaged with respect to previous water locations=
const float obsLocationJitter = .1;              // (m) jitter to be applied to obsLocations on every trial
const float startPosJitter = .02;                // (m) jitter in the starting position of the obstacle on the track relative to the start limit

// durations
volatile int waterDuration = 80;                   // (ms) time that water solenoid is open for
const int servoSwingTime = 150;      // (ms) approximate amount of time it takes for the osbtacle to pop out // this is used as a delay bewteen the obstacle reaching the end of the track and it coming back to avoid it whacking the guy in the butt!

// speed
const double obsSpeedMax = 1.5;                // (m/s) maximum obstacle speed
const double obsSpeedMin = .4;                 // (m/s) speed at which obstacle starts moving
const double firstCallibrationSpeed = 1.0;     // (m/s) speed of first callibration that occurs when arduino first turns on
const double callibrationSpeedMax = 1.0;       // (m/s) max speed of obstacle during callibration
const double obsAcceleration = 2.0;            // (m/s^2) obstacle acceleration (used when obstacle is starting up or stopping)
const int speedSamples = 10;                   // successeive samples of wheel soeed to measure (samples are separated by about 500 microseconds)

// obstacle light
float obsLightProbability = 0.5;     // probability of obstacle light turning on
float obstacleBrightness = .5;       // (0->1) fraction of total brightness for obstacle LEDs

// rig characteristics
const double timingPulleyRad = .0152789;         // (mm) radius of timing pulley on stepper motor
const double wheelRad = 0.09525;                 // (m) wheel radius
const double trackStartBuffer = .1;              // (m) distance between start limit switch and position at which obstacle should stop
const double trackEndBuffer = .05;               // (m) distance between end limit switch and position at which obstacle should stop
const int microStepping = 16;                    // stepper motor only moves (1/microStepping) steps per pulse // this should match the setting on the stepper motor driver, which is set by 3 digital inputs
const int motorSteps = 200;                      // steps per revolution for the obstacle stepper motor
const int encoderSteps = 2880;                   // steps per revolution on the wheel rotary encoder (720cpr * 4)
double trackEndPosition = 0.46;                  // (m) estimate of distance obstacle travels between limit switches // this will be updated during the initial callibration
