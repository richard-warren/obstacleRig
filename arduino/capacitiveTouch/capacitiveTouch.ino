#include "CapacitiveSensor.h"

// pin assignments
const int touchSendPin = 7;
const int touchReceivePin = 8;
const int wheelBreakPin = 2;

// user settings
const int wheelBreakDuration = 500; // ms
const int sensorSmps = 2;
const int touchThresh = 150;
const int touchMax = 255;
const int maxMeasurementTime = 100; // ms

// initializations
CapacitiveSensor touchSensor = CapacitiveSensor(touchSendPin, touchReceivePin);
volatile int touchMeasurement = 0;



void setup(){
  pinMode(wheelBreakPin, OUTPUT);
  digitalWrite(wheelBreakPin, LOW);
  
  touchSensor.set_CS_Timeout_Millis(maxMeasurementTime / sensorSmps);
  Serial.begin(9600);
}

void loop(){
  touchMeasurement = (byte) min((touchSensor.capacitiveSensor(sensorSmps) / sensorSmps), touchMax);
//  Serial.println(touchMeasurement);
  
  if (touchMeasurement > touchThresh){
    digitalWrite(wheelBreakPin, HIGH);
    delay(wheelBreakDuration);
    digitalWrite(wheelBreakPin, LOW);
  }
  delay(20);
  Serial.println(touchMeasurement);
}







