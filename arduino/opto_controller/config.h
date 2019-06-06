// STIMULUS GENETER
// this is an arduino uno based stimulus generator that uses Adafruit_MCP4725 for DAC output
// can deliver steps, sin waves, or pulse trains of specified durations and frequencies
// stimuli are triggered by an external input, which can also be used to control the duration of
// the stimulus if constantSignalDuration is true. the power of the signal can be scaled from 0-5V,
// and an optional reference output that does not scale with power can be used to drive a reference LED


// pin assignments
const int triggerPin = 2; // pin that triggers stimulation (must be an interrupt pin on Arduino Uno)
const int referencePin = 3; // PWM output pins that delivers signal modulated the same as DAC, but scaled from 0-5V (use this to power a reference LED that can be captured in videos, for example)

volatile int stimType = 2; // 0: sine, 1: step, 2: pulse train
volatile float hz = 5; // frequency of sine wave
volatile int signalDuration = 1000; // ms (includes ramp up and ramp down times)
volatile int rampUpTime = 0; // ms
volatile int rampDownTime = 0; // ms
const bool constantSignalDuration = false; // if true, stimulus duration is stimDuration+rampDownTime // otherwise, stim continues until triggerPin goes low
volatile int pulseDuration = 10; // ms

volatile float signalProbability = 1; // probability of delivering stimulus when trigger is received
volatile float signalPower = .5; // fraction of light power
