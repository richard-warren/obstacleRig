// pin assignments
#define stepPin            4
#define stepDirPin         5
#define encoderPinA        20   // don't change encoderPins, because direct port manipulation is used that assumes 20 and 21 are used for encoder pins
#define encoderPinB        21 
#define waterPin           7
#define obstaclePin        13   // signals whether the obstacle is engaged... this is sent to an arduino that controls the obstacle servo
#define motorOffPin        8    // turns on stepper motor driver
#define startLimitPin      9    // signal is LOW when engaged
#define stopLimitPin       10   // signal is LOW when engaged
#define obsLightPin        2    // controls when the light for the obstacle turns on
#define obsLightPin2       6    // controls when the light for the obstacle turns on
#define obsOnPin           3    // when the obstacle is engaged in a trial, i.e. tracking the mouse's position
#define touchSensorOnPin   12   // powers the touch sensor only when obs is within range of paws
#define stimulusPin        14   // this will trigger opto stimulation when obstalce reaches certain position


// user settings
volatile int state = 1; // 1: no platform movement, no obstacles, 2: platform movement, no obstacles, 3: platform movement and obstacles
volatile float obsGain = 1.0;
const int servoSwingTime = 150; // ms, approximate amount of time it takes for the osbtacle to pop out // this is used as a delay bewteen the obstacle reaching the end of the track and it coming back, to avoid it whacking the guy in the butt!
volatile float rewardRotations = 10.01;
const int startPositionMm = 5;
const int endPositionMm = 40;
volatile int waterDuration = 80; // milliseconds
const double maxStepperSpeed = 1.6; // (m/s)
const float acceleration = 8.0; // (m/s^2)
volatile float callibrationSpeed = 1.0; // speed with motor moves plastform during callibration (m/s) // note: changed back from .8 to .6 because anything above .6 fails to initialize to the correct starting position, instead just pausing at the end of the track
volatile float minCallibrationSpeed = .2; // motor slows down to this speed as it approaches the end of the track
//const float obstacleLocations[] = {1.5, 4.5, 7.5, rewardRotations*20}; // expressed in wheel ticks // the last element is a hack... the index goes up and the wheel position will never reach the last value, which is the desired behavior
const float obstacleLocations[] = {2.5, 5.5, 8.5, rewardRotations*20}; // expressed in wheel ticks // the last element is a hack... the index goes up and the wheel position will never reach the last value, which is the desired behavior
const int velocitySamples = 10; // each sample last about 500 microseconds
const int obsPosJitter[] = {-100, 100}; // jitter range for the onset position of obstacles (mm)
const int touchSensorOnLimits[] = {200, 385}; // mm, distance from beginning of track to turn on and off touch sensor
const int startPosJitter = 20; // (mm)
volatile float obsLightProbability = 0.5;
const long delayLookupLength = 20000;

// stimulus settings
volatile float stimulusPosition = 320; // (mm) distance from beginning of track at which opto stimulus should be triggered // this will be overwritten if stimInsteadOfObstacles is true
const bool stimWithObstacles = true; // if true, always delivers stim when obstacles are on (ignores stimulus position)
const float mBefore Obs = .8; // how many meters before obstacle to turn on stimulusPin


// rig characteristics
const int microStepping = 16; // only (1/microStepping) steps per pulse // this should correspond to the setting on the stepper motor driver, which is set by 3 digital inputs
const int motorSteps = 200;
const int encoderSteps = 2880; // 720cpr * 4
const double timingPulleyRad = 15.2789; // (mm)
const double wheelRad = 95.25; // (m)
