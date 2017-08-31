#include "CapacitiveSensor.h"

// pin assignments
const int touchSendPin = 8;
const int touchReceivePin = 10;
const int wheelBreakPin = 4;

// user settings
const bool debugOn = true;
const int wheelBreakDuration = 5000; // ms
const int sensorSmps = 2;
const int touchThresh = 150;
const int touchMax = 255;
const int maxMeasurementTime = 100; // ms

// initializations
CapacitiveSensor touchSensor = CapacitiveSensor(touchSendPin, touchReceivePin);
volatile int touchMeasurement = 0;



void setup(){
  // setup pins
  pinMode(wheelBreakPin, OUTPUT);
  digitalWrite(wheelBreakPin, LOW);

  // setup touch sensor
  touchSensor.set_CS_Timeout_Millis(maxMeasurementTime / sensorSmps);
  if (debugOn){Serial.begin(9600);}
}

void loop(){

  // get touch measurement
  touchMeasurement = (byte) min((touchSensor.capacitiveSensor(sensorSmps) / sensorSmps), touchMax);

  // break wheel if touch detected
  if (touchMeasurement > touchThresh){
    digitalWrite(wheelBreakPin, HIGH);
    delay(wheelBreakDuration);
    digitalWrite(wheelBreakPin, LOW);
  }
  
  // display raw touch sensor values in debug mode
  if (debugOn){
    delay(20);
    Serial.println(touchMeasurement);
  }
}







