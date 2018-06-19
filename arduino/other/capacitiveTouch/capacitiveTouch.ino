#include <CapacitiveSensor.h>
#include <Filters.h>

// pin assignments
const int touchSendPin = 8;
const int touchReceivePin = 6;
const int isTouchingPin = 12;

// user settings
const int touchThresh = 720; // 525;
const int sensorSmps = 2;
const int maxMeasurementTime = 10; //2; // ms, capacitive touch reading times out after maxMeasurementTime
const int baselineSmpNum = 10000;
const int medianSmpNum = 19;
const float filterFrequency = 10.0;
const float baselineFilterFrequency = 2.0;

// other initializations
CapacitiveSensor touchSensor = CapacitiveSensor(touchSendPin, touchReceivePin);
FilterOnePole lowpassFilter(LOWPASS, filterFrequency);
FilterOnePole lowpassBaselineFilter(LOWPASS, baselineFilterFrequency);
volatile int touchMeasurement = 0;
volatile int touchMeasurementSmoothed = 0;
volatile int baseline = 0;



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
  touchMeasurement = touchSensor.capacitiveSensorRaw(sensorSmps);

  // if overflowed, set to max value
  if (touchMeasurement<0){
    touchMeasurement = touchThresh+1;
  // otherwise normalize by number of sensor samples
  }else{
    touchMeasurement = touchMeasurement / sensorSmps;
  }

  
  // filter
  touchMeasurementSmoothed = lowpassFilter.input(touchMeasurement);
  baseline = lowpassBaselineFilter.input(touchMeasurement);
  
  
  
  // set isTouchingPin 
  if (touchMeasurement > touchThresh){
    digitalWrite(isTouchingPin, HIGH);
  }else{
    digitalWrite(isTouchingPin, LOW);
  }

  Serial.print(touchMeasurement);
  Serial.print(",");
  Serial.print(touchMeasurementSmoothed);
  Serial.print(",");
  Serial.print(baseline);
  Serial.print(",");
  Serial.println(abs(touchMeasurementSmoothed - baseline));
  delay(1);
}







