#include <Servo.h>



// pin assignments
const int inputPin = 2; // when this is high the servo is engaged; disengage when it goes low
const int servoPin = 8;

// user settings
const int engagedPosition =  62;
const int disengagedPosition = engagedPosition + 45;
const int pwmMin = 553;
const int pwmMax = 2450;

// 128, 128-45

// initializations
volatile bool isServoEngaged = false;
Servo obstacleServo;



void setup() {
  
  // initialize servo
  pinMode(inputPin, INPUT);
  obstacleServo.attach(servoPin, pwmMin, pwmMax);

  // initialize hardware interrupt
  attachInterrupt(digitalPinToInterrupt(inputPin), controlServo, CHANGE);

  // test obstacle positionts
  obstacleServo.write(disengagedPosition);
  delay(1000);
  obstacleServo.write(engagedPosition);
  delay(1000);
  obstacleServo.write(disengagedPosition);

}



void loop(){}


void controlServo(){
  
  // read desired obstacle status
  isServoEngaged = digitalRead(inputPin);

  // set obstacle to desired position
  if (isServoEngaged){
    obstacleServo.write(engagedPosition);
  }else{
    obstacleServo.write(disengagedPosition);
  }
  
}






