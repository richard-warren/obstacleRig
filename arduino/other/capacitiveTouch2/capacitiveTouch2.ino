#include <CapacitiveSensor.h>
#include <Filters.h>

// pin assignments
const int touchSendPin = 8;
const int touchReceivePin = 6;
const int isTouchingPin = 12;

// user settings
const int touchThresh = 80; // 525;
const int touchMax = 1023;
const int sensorSmps = 2;
const int maxMeasurementTime = 10; //2; // ms, capacitive touch reading times out after maxMeasurementTime
const float lowpassFreq = 20.0;
const float highpassFreq = 0.1;

// other initializations
CapacitiveSensor touchSensor = CapacitiveSensor(touchSendPin, touchReceivePin);
FilterOnePole lowpassFilter(LOWPASS, lowpassFreq);
FilterOnePole highpassFilter(HIGHPASS, highpassFreq);
volatile int touch = 0;
volatile int touchLowPassed = 0;
volatile int touchBandPassed = 0;



void setup(){
  
  // setup pins
  pinMode(isTouchingPin, OUTPUT);
  digitalWrite(isTouchingPin, LOW);

  // setup touch sensor
  touchSensor.set_CS_Timeout_Millis(maxMeasurementTime);
  
  Serial.begin(115200);
  
}



void loop(){

  // get touch measurement
  touch = touchSensor.capacitiveSensorRaw(sensorSmps);

  // if overflowed, set to max value
  if (touch<0 || touch>touchMax){
    touch = touchMax;
  // otherwise normalize by number of sensor samples
  }else{
    touch = touch / sensorSmps;
  }

  
  // filter
  touchLowPassed = lowpassFilter.input(touch);
  touchBandPassed = highpassFilter.input(touchLowPassed);
//  touchBandPassed = touch;
  
  
  // set isTouchingPin 
  if (touchBandPassed > touchThresh){
    digitalWrite(isTouchingPin, HIGH);
  }else{
    digitalWrite(isTouchingPin, LOW);
  }

//  Serial.print(touch);
//  Serial.print(",");
//  Serial.print(touchLowPassed);
//  Serial.print(",");
  Serial.print(touchThresh);
  Serial.print(",");
  Serial.println(touchBandPassed);
  delay(1);
}







