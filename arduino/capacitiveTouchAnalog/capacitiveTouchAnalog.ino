#include <Wire.h>                       // for i2c communication with DAC
#include <Adafruit_MCP4725.h>           // for DAC control
#include <CapacitiveSensor.h>
Adafruit_MCP4725 dac;


// pin assignments
const int touchSendPin = 3;
const int touchReceivePin = 4;
const int isTouchingPin = 12;

// user settings
const int touchThresh = 600;
const int sensorSmps = 2;
const int maxMeasurementTime = 4; // ms, capacitive touch reading times out after maxMeasurementTime
const int maxTouchMeasurement = 4095; // maximum DAC value

// other initializations
CapacitiveSensor touchSensor = CapacitiveSensor(touchSendPin, touchReceivePin);
volatile int touchMeasurement = 0;



void setup(){
  
  // setup pins
  pinMode(isTouchingPin, OUTPUT);
  digitalWrite(isTouchingPin, LOW);

  // setup touch sensor
  touchSensor.set_CS_Timeout_Millis(maxMeasurementTime);

  // initialize DAC
  dac.begin(0x62);
  dac.setVoltage(0, true);

  Serial.begin(115200);
  
}



void loop(){

  // get touch measurement
  touchMeasurement = touchSensor.capacitiveSensorRaw(sensorSmps);

  // if overflowed, set to max value
  if (touchMeasurement<0){
    touchMeasurement = maxTouchMeasurement;
  
  // otherwise normalize by number of sensor samples
  }else{
    touchMeasurement = touchMeasurement / sensorSmps;
  }
     
  // output touch measurement via dac
  dac.setVoltage(touchMeasurement, false);
  
  // set isTouchingPin 
  if (touchMeasurement>touchThresh){
    digitalWrite(isTouchingPin, HIGH);
  }else{
    digitalWrite(isTouchingPin, LOW);
  }
  delayMicroseconds(100);

  Serial.println(touchMeasurement);
}







