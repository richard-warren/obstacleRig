#include <Servo.h>



// pin assignments
const int servoInputPin = 2; // when this is high the servo is engaged; disengage when it goes low
const int rewardInputPin = 3;
const int servoPin = 8;
const int vidTtlPin = 13;
const int servoPowerPin = 12;


// user settings
const int servoPowerTime = 200; // how many ms to power the motor for when a position change request is made
const float engagedPosition =  10; // smaller numbers aim more towards stepper motor
const int disengagedPosition = engagedPosition + 60;
const int pwmMin = 553;
const int pwmMax = 2450;
const int vidTtlPulseDuration = 1;   // ms
const int vidTtlInterval = 4; // ms
const int rewardTtlOverhang = 500; // ms to continue generating ttls for after reward reached // THIS SHOULD NOT BE DIVISIBLE BY 1000, SO THE TIMESTAMP INTERVAL BETWEEN FRAMES AT ENDAND BEGINNING OF NEXT TRIAL WILL BE DIFFERENT THAN 1/FS
const int rewardTtlGap = 500; // ms pause in ttls after rewardTtlOverhang has passed

// initializations
volatile bool isServoEngaged = false;
volatile bool isTtlOn = false;
volatile bool overHanging = false;
volatile int vidTtlTimer = 0;
volatile int overhangTimer = 0;
volatile int servoPowerTimer = servoPowerTime;
volatile int serialInput = 0;

Servo obstacleServo;





void setup() {
  
  // initialize pins
  pinMode(servoInputPin, INPUT);
  pinMode(rewardInputPin, INPUT);
  pinMode(vidTtlPin, OUTPUT);
  pinMode(servoPowerPin, OUTPUT);
  digitalWrite(vidTtlPin, LOW);

  
  // initialize servo
  obstacleServo.attach(servoPin, pwmMin, pwmMax);

  
  // initialize hardware interrupts
  attachInterrupt(digitalPinToInterrupt(servoInputPin), controlServo, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rewardInputPin), rewardReached, RISING);

  
  // test obstacle positions
  digitalWrite(servoPowerPin, HIGH);
  obstacleServo.write(disengagedPosition);
  delay(servoPowerTime);
  obstacleServo.write(engagedPosition);
  delay(servoPowerTime);
  obstacleServo.write(disengagedPosition);
  delay(servoPowerTime);
  digitalWrite(servoPowerPin, LOW);
  
  
  // initialize 1kHz timer0 interrupt
  noInterrupts();
  TCCR0A = 0;// set entire TCCR0A register to 0
  TCCR0B = 0;// same for TCCR0B
  TCNT0  = 0;//initialize counter value to 0
  // set compare match register for 2khz increments
  OCR0A = 249;// = (16*10^6) / (1000*64) - 1 (must be <256)
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // Set CS01 and CS00 bits for 64 prescaler
  TCCR0B |= (1 << CS01) | (1 << CS00);
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);
  interrupts();
  
    
  // begin serial communication
  Serial.begin(9600);
  Serial.println("1 = start video ttls");
  Serial.println("2 = stop video ttls");
}



void loop(){

  if (Serial.available()){
    
    serialInput = Serial.read()-48; // parseInt was not working here for some reason

    switch (serialInput){
      case 1:
        isTtlOn = true;
        overHanging = false;
        break;
      case 2:
        isTtlOn = false;
        overHanging = false;
        break;
    }
  }
  
}


// servo engage/disengage interrupts
void controlServo(){
  
  // read desired obstacle status
  isServoEngaged = digitalRead(servoInputPin);

  // power servo and reset servoPowerTimer
  digitalWrite(servoPowerPin, HIGH);
  servoPowerTimer = 0;


  // set obstacle to desired position
  if (isServoEngaged){
    obstacleServo.write(engagedPosition);
  }else{
    obstacleServo.write(disengagedPosition);
  }
}



// reward reached interrupt
void rewardReached(){
  if (isTtlOn){
    overHanging = true;
    overhangTimer = 0;
  }
}



// timer0 interrupts controls vidTtl generation
ISR(TIMER0_COMPA_vect) {
    
  // control vidTtl generation
  vidTtlTimer++;
  
  switch (vidTtlTimer){
    case 1:
      if (isTtlOn){
        digitalWrite(vidTtlPin, HIGH);
      }
      break;
    case vidTtlPulseDuration+1:
      digitalWrite(vidTtlPin, LOW);
      break;
    case vidTtlInterval:
      vidTtlTimer = 0;
      break;
  }
  

  // control vidTtl overhang period
  if (overHanging){
    
    overhangTimer++;

    switch (overhangTimer){
      case rewardTtlOverhang:
        isTtlOn = false;
        break;
      case rewardTtlOverhang + rewardTtlGap:
        isTtlOn = true;
        overHanging = false;
        break;
    }
  }


  // control servo power timer
  servoPowerTimer++;

  switch (servoPowerTimer){
    case servoPowerTime:
      digitalWrite(servoPowerPin, LOW);
      break;
    case servoPowerTime+1:
      servoPowerTimer--;
      break;  
  }
  
}




