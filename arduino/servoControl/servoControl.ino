#include <Servo.h>



// pin assignments
const int inputPin = 2; // when this is high the servo is engaged; disengage when it goes low
const int servoPin = 13; // it's possible the servo library only works if this pin is set to 9 or 10... not sure if this is true though...

// user settings
const int servoObsEngagedPosition = 0;
const int servoObsDisengagedPosition = servoObsEngagedPosition + 90;

// initializations
volatile bool isServoEngaged = false;
Servo obstacleServo;



void setup() {
  
  // initialize servo
  pinMode(inputPin, INPUT);
  obstacleServo.attach(servoPin);
  obstacleServo.write(servoObsDisengagedPosition);

  // initialize hardware interrupt
  attachInterrupt(digitalPinToInterrupt(inputPin), controlServo, CHANGE);

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






