// !!! need to document
// only works for Arduino Uno


// pin assignments
const int triggerPin = 2; // pin that triggers stimulation
const int referencePin = 3; // PWM output pins that delivers signal modulated the same as DAC, but scaled from 0-5V (use this to power a reference LED that can be captured in videos, for example)

volatile int stimType = 0; // 0: sine, 1: step, 2: pulse train
volatile float hz = 2; // frequency of sine wave
volatile int signalDuration = 1000; // ms (includes ramp up and ramp down times)
const int rampDownTime = 0; // ms
const int rampUpTime = 0; // ms
const bool constantSignalDuration = false; // if true, stimulus duration is stimDuration+rampDownTime // otherwise, stim continues until triggerPin goes low
volatile int pulseDuration = 10; // ms

volatile float signalProbability = 1; // probability of delivering stimulus when trigger is received
volatile float signalPower = .5; // fraction of light power
