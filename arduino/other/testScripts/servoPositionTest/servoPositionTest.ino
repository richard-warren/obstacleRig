#include <Servo.h>


// pin assignments
const int servoPin = 8;
const int servoPowerPin = 12;

// user settings
const int servoPosition1 = 0;  // lower numbers point more towards stepper motor
const int servoPosition2 = 180;
const int pwmMin = 553;
const int pwmMax = 2450;
// obs1: 50 // mid: // obs2: 124

// initializations
Servo obstacleServo;
volatile int userInput;


// setup
void setup() {
  
  pinMode(servoPowerPin, OUTPUT);
  digitalWrite(servoPowerPin, HIGH);
  obstacleServo.attach(servoPin, pwmMin, pwmMax);
  Serial.begin(9600);

}



// main loop
void loop() {
  
  if (Serial.available()){
    userInput = Serial.parseInt();
//    userInput = constrain(userInput, 0, 180);
    obstacleServo.write(userInput);
    Serial.print("position set to: ");
    Serial.println(userInput);
  }

//  obstacleServo.write(servoPosition1);
//  delay(1000);
//  obstacleServo.write(servoPosition2);
//  delay(3000);

}
