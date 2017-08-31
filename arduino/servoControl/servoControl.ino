#include <Servo.h>



// pin assignments
const int inputPin = 2; // when this is high the servo is engaged; disengage when it goes low
const int servoPin = 8;

// user settings
const int servoObsDisengagedPosition =  128;
const int servoObsEngagedPosition = servoObsDisengagedPosition - 45;

// 128, 128-45

// initializations
volatile bool isServoEngaged = false;
Servo obstacleServo;



void setup() {
  
  // initialize servo
  pinMode(inputPin, INPUT);
  obstacleServo.attach(servoPin);

  // initialize hardware interrupt
  attachInterrupt(digitalPinToInterrupt(inputPin), controlServo, CHANGE);

  // test obstacle positionts
  obstacleServo.write(servoObsDisengagedPosition);
  delay(1000);
  obstacleServo.write(servoObsEngagedPosition);
  delay(1000);
  obstacleServo.write(servoObsDisengagedPosition);

}



void loop(){}


void controlServo(){
  
  // read desired obstacle status
  isServoEngaged = digitalRead(inputPin);

  // set obstacle to desired position
  if (isServoEngaged){
    obstacleServo.write(servoObsEngagedPosition);
  }else{
    obstacleServo.write(servoObsDisengagedPosition);
  }
  
}






