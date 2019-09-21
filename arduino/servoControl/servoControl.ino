/*

This code is for Arduino Uno does the following:
  - controls the height of the obstacles
  - generates TTLs that trigger frame acquisition
  - controls the servo motor that swings the obstacle in and out

To callibrate obstacle heights and alignment:
  - Set startWithObsOut to true and open the serial monitor. The obstacle will start at a position which should be just barely touching the surface of the wheel.
  - Adjust obsHeightZero such that the obstacle just touches the top of the wheel.
  - Using the obstacleControl script, toggle the obstacle on and off to generate new random heights.
  - Adjust obsOnPosition such that the obstacle is perpendicular to the wheel and obsOffPosition if necessary.
  - Measure the heights with a ruler and check their correspondence with the heights displayed on the Serial Port.
  - Adjust obsOffset so thhe obstacle heights match the intended obstacle heights.

*/

#include <Servo.h> // this takes over timer1 according to the internet


// PINT ASSIGNMENTS
const int servoInputPin = 2;     // input signaling that the servo motor should swing out
const int waterInputPin = 3;     // input signaling that water has been delivered // this causes a brief delay in the video ttls
const int servoPin = 8;          // controls position of servo motor
const int servoPowerPin = 12;    // gates power to servo motor via transistor
const int vidTtlPin = 13;        // generates TTLs that trigger frame acquisition
const int obsHeightPin = 11;     // controls obstacle height via vertical motor // DON'T CHANGE // this code uses timer0 for interrupt functions, while timer1 is used by the servo library, while pins 3 and 11 use timer2 on the Arduino Uno


// SETTINGS

// alignment settings
const bool startWithObsOut = false;          // if true, obstacle starts in the engaged position, which is useful for testing vertical and horizontal alignnment
const float obsOffset = 0;                   // if Arduino thinks the heights are lower than they are, increase this value, and vice versa // these values will be added to the Serial output that reports the obstacle height
const float obsHeightZero = .6;              // (mm) height of obstacle (relative to the lowest position on the vertical motor) when it is flush with the floor of the wheel
const float obsThickness = 3.8;              // the latter term takes care of measurement offset - without it obs is set higher than intended

// vertical motor and obstacle
const bool randomizeHeights = true;          // whether to randomize obstacle heights // if false, default value for obsHeight is used throughout
const float obsHeightMin = 4.0;              // (mm) min obstacle height
const float obsHeightMax = 10.0;             // (mm) max obstacle height
float obsHeight = 5.0;                       // (mm) height of obstacle, measured from top of wheel to top of obstacle
const float obsHeightTravel = 15.0;          // (mm) DON'T CHANGE // the total travel of the vertical motor

// servo motor
const float obsOnPosition = 34;      // (unitless) position of obstacle when engaged // smaller number are further away from the mouse
const float obsOffPosition = 100;    // (unitless) position of obstacle when desengaged
const int servoPowerTime = 1000;     // (ms) how many ms to power the motor for when a position change request is made
const int pwmMin = 553;              // DON'T CHANGE // min PWM rate, which is a characteristic of the servo motor
const int pwmMax = 2450;             // DON'T CHANGE // max PWM rate, which is a characteristic of the servo motor

// video ttls
const int vidTtlInterval = 4;        // (ms) interval between TTLs that trigger frame acquisition
const int vidTtlPulseDuration = 1;   // (ms) duration of TTLs
const int rewardTtlOverhang = 500;   // (ms) how long to continue generating TTLs after reward is reached before pausing TTLs for rewardTtlGap milliseconds // THIS SHOULD NOT BE DIVISIBLE BY 1000, SO THE TIMESTAMP INTERVAL BETWEEN FRAMES AT ENDAND BEGINNING OF NEXT TRIAL WILL BE DIFFERENT THAN 1/FS
const int rewardTtlGap = 500;        // (ms) how long to pause TTL generation after rewardTtlOverhang has passed



// INITIALIZATIONS
bool isServoEngaged = false;
bool isTtlOn = false;
bool overHanging = false;
int vidTtlTimer = 0;
int overhangTimer = 0;
int serialInput = 0;
bool disableAfterSteps = false;
int servoPowerTimer = servoPowerTime;
Servo obstacleServo;




void setup() {

  // initialize pins
  pinMode(servoInputPin, INPUT);
  pinMode(waterInputPin, INPUT);
  pinMode(vidTtlPin, OUTPUT);
  pinMode(obsHeightPin, OUTPUT);
  pinMode(servoPowerPin, OUTPUT);
  digitalWrite(vidTtlPin, LOW);


  // initialize random seed
  randomSeed(analogRead(0));


  // initialize hardware interrupts
  attachInterrupt(digitalPinToInterrupt(servoInputPin), controlServo, CHANGE);
  attachInterrupt(digitalPinToInterrupt(waterInputPin), rewardReached, RISING);


  // initialize servo
  obstacleServo.attach(servoPin, pwmMin, pwmMax);


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
  
  // initialize servo position
  turnOnServo();
  if (startWithObsOut) {
    obstacleServo.write(obsOnPosition);
  } else {
    obstacleServo.write(obsOffPosition);
  }
  setObsHeight(obsThickness);

}



void loop() {

  if (Serial.available()) {

    serialInput = Serial.read() - 48; // parseInt was not working here for some reason

    switch (serialInput) {
      
      // Bonsai sends '1' or '2' via the serial port
      // 1 starts TTL generation, and 2 stops TTL generation (at the end of the recording)
      
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
void controlServo() {

  // read desired obstacle status
  isServoEngaged = digitalRead(servoInputPin);

  // set obstacle to desired position
  turnOnServo();
  
  if (isServoEngaged) {

    // set obstacle height
    if (randomizeHeights) {
      obsHeight = random(obsHeightMin * 10, obsHeightMax * 10) / 10.0;
    }
    setObsHeight(obsHeight);
    obstacleServo.write(obsOnPosition);

    // disengage obstacle
  } else {
    obstacleServo.write(obsOffPosition);
  }
}



// reward reached interrupt
void rewardReached() {
  if (isTtlOn) {
    overHanging = true;
    overhangTimer = 0;
  }
}



// set obstacle height
void setObsHeight(float obsHeight) {
  int obsHeight8 = round((obsHeightZero + obsHeight - obsThickness) * (255.0 / obsHeightTravel));  // height of obstacle expressed as fraction of total motor range and scaled from 0->255
  Serial.println(obsHeight + obsOffset);
  analogWrite(obsHeightPin, 255 - constrain(obsHeight8, 0, 255));
}



// timer0 interrupts controls vidTtl generation
ISR(TIMER0_COMPA_vect) {

  // control vidTtl generation
  vidTtlTimer++;

  switch (vidTtlTimer) {
    case 1:
      if (isTtlOn) {
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
  if (overHanging) {

    overhangTimer++;

    switch (overhangTimer) {
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

  switch (servoPowerTimer) {
    case servoPowerTime:
      digitalWrite(servoPowerPin, LOW);
      break;
    case servoPowerTime+1:
      servoPowerTimer--;
      break;
  }
}



// turn on servo power
void turnOnServo() {
  digitalWrite(servoPowerPin, HIGH);
  servoPowerTimer = 0;  // this starts the servo timer
}
