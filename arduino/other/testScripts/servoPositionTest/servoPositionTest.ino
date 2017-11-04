#include <Servo.h>


// pin assignments
const int servoPin = 8;

// user settings
const int servoPosition1 = 11;  // lower numbers point more towards stepper motor
const int servoPosition2 = servoPosition1;
const int pwmMin = 553;
const int pwmMax = 2450;

// initializations
Servo obstacleServo;



// setup
void setup() {
  
  obstacleServo.attach(servoPin, pwmMin, pwmMax);

}



// main loop
void loop() {
  
  obstacleServo.write(servoPosition1);
  delay(1000);
  obstacleServo.write(servoPosition2);
  delay(3000);

}
