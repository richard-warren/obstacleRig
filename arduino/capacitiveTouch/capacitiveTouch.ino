#include <CapacitiveSensor.h>

// pin assignments
const int touchSendPin = 8;
const int touchReceivePin = 6;
const int isTouchingPin = 12;

// user settings
const int touchThresh = 375; // 525;
const int sensorSmps = 2;
const int maxMeasurementTime = 10; //2; // ms, capacitive touch reading times out after maxMeasurementTime

// other initializations
CapacitiveSensor touchSensor = CapacitiveSensor(touchSendPin, touchReceivePin);
volatile int touchMeasurement = 0;



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
  
  
  // set isTouchingPin 
  if (touchMeasurement > touchThresh){
    digitalWrite(isTouchingPin, HIGH);
  }else{
    digitalWrite(isTouchingPin, LOW);
  }

  Serial.println(touchMeasurement);
  delay(1);
}







