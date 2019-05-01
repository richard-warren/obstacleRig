#include <Servo.h>


// pin assignments
const int servoPin = 8;

// user settings
const int pwmMin = 553;
const int pwmMax = 2450;

// initializations
Servo obstacleServo;
volatile int userInput;


// setup
void setup() {  
  obstacleServo.attach(servoPin, pwmMin, pwmMax);
  Serial.begin(9600);
}


// main loop
void loop() {  
  if (Serial.available()){
    userInput = Serial.parseInt();
    obstacleServo.write(userInput);
    Serial.print("position set to: ");
    Serial.println(userInput);
  }
}
