


// pin assignments
const int stepperInputPin = 2; // when this is high the stepper is engaged; disengage when it goes low
const int rewardInputPin = 3;
const int endStopPin = 7;
const int stepperDirectionPin = 6;
const int stepperStepPin = 5;
const int stepperDisablePin = 4;
const int vidTtlPin = 13;
const int obsHeightPin = 11; // don't change - i hack into timer0, timer1 is used by stepper library, and pins 3,11 use timer2 on arduino uno
const int minObsHeight = 2.5; // (mm) height of obs when it is flush with the floor of the wheel


// user settings
const float obsOffset = .9;
const float obsThickness = 3.175 + obsOffset; // the latter term takes care of measurement offset - without it obs is set higher than intended
const bool randomizeHeights = true;
const float randObsHeightMin = obsThickness;
const float randObsHeightMax = 10.0;
volatile float obsHeight = 8.5;  // (mm), height of obs (top of wheel to top of obs)
volatile float tallShortProbability = 0.5; // probability that the obstacle will be high or low
const float obsOnSteps = 204; // number of steps to take from the end stop (use to adjust obstacle alignment)
const float obsOffSteps = -180;
const int vidTtlPulseDuration = 1;   // ms
const int vidTtlInterval = 4; // ms
const int rewardTtlOverhang = 500; // ms to continue generating ttls for after reward reached // THIS SHOULD NOT BE DIVISIBLE BY 1000, SO THE TIMESTAMP INTERVAL BETWEEN FRAMES AT ENDAND BEGINNING OF NEXT TRIAL WILL BE DIFFERENT THAN 1/FS
const int rewardTtlGap = 500; // ms pause in ttls after rewardTtlOverhang has passed
const float obsHeightTravel = 15.0; // mm
const int stepTtlDuration = 1; // ms
const int stepTtlInterval = 1; // ms


// initializations
volatile bool isStepperEngaged = false;
volatile bool isTtlOn = false;
volatile bool overHanging = false;
volatile int vidTtlTimer = 0;
volatile int stepperTtlTimer = 0;
volatile int overhangTimer = 0;
volatile int serialInput = 0;
volatile int stepsToTake = 0;
volatile int stepsTaken = 0;
volatile bool isZeroing = false;
volatile bool disableAfterSteps = false;




void setup() {

  // initialize pins
  pinMode(stepperInputPin, INPUT);
  pinMode(rewardInputPin, INPUT);
  pinMode(vidTtlPin, OUTPUT);
  pinMode(stepperDisablePin, OUTPUT);
  pinMode(stepperDirectionPin, OUTPUT);
  pinMode(stepperStepPin, OUTPUT);
  pinMode(obsHeightPin, OUTPUT);
  pinMode(endStopPin, INPUT_PULLUP);
  
  digitalWrite(vidTtlPin, LOW);
  digitalWrite(stepperDisablePin, HIGH);
  digitalWrite(stepperStepPin, LOW);
  digitalWrite(stepperDirectionPin, HIGH);
  setObsHeight(obsHeight);


  // initialiez random seed
  randomSeed(analogRead(0));

  // initialize hardware interrupts
  attachInterrupt(digitalPinToInterrupt(stepperInputPin), controlstepper, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rewardInputPin), rewardReached, RISING);



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
//  Serial.println("1: start video ttls");
//  Serial.println("2: stop video ttls");
//  Serial.println("3: set obstacle height");
}



void loop() {

  if (Serial.available()) {

    serialInput = Serial.read() - 48; // parseInt was not working here for some reason

    switch (serialInput) {
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


// stepper engage/disengage interrupts
void controlstepper() {
  
  // read desired obstacle status
  isStepperEngaged = digitalRead(stepperInputPin);

  // set obstacle to desired position
  if (isStepperEngaged) {

    // set obstacle height
    if (randomizeHeights){
      obsHeight = (random(randObsHeightMin*10.0, randObsHeightMax*10.0) / 10.0);
      setObsHeight(max(0, obsHeight));
    }
     Serial.println(obsHeight);

    // engage obstacle
    takeStep(obsOnSteps, true);
    
  } else {
    // disengage obstacle
    takeStep(obsOffSteps, false);
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
  int obsHeight8 = round((obsHeight-obsThickness+minObsHeight) * (255.0 / obsHeightTravel));
  analogWrite(obsHeightPin, 255 - constrain(obsHeight8,0,255));
//  Serial.print("obstacle height set to: ");
 
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


  // control stepper ttl timer
  if (stepsTaken<stepsToTake){    
    stepperTtlTimer++;
  
    switch (stepperTtlTimer) {
      case stepTtlDuration:
        digitalWrite(stepperStepPin, LOW);
        break;
      case stepTtlDuration+stepTtlInterval:
        digitalWrite(stepperStepPin, HIGH);
        stepperTtlTimer = 0;
        if (isZeroing){
          if (!digitalRead(endStopPin)){
            isZeroing = false;
            digitalWrite(stepperDirectionPin, HIGH);
          }
        }else{
          stepsTaken++;
        }
        break;
    }
  }else if (disableAfterSteps){ // disable motor only after going home
    digitalWrite(stepperDisablePin, HIGH);
  }
}



void takeStep(volatile int steps, bool homeFirst){
  
  // turn on motor
  digitalWrite(stepperDisablePin, LOW);
  
  // set direction
  if (homeFirst){
    digitalWrite(stepperDirectionPin, LOW);
    digitalWrite(stepperStepPin, HIGH);
    isZeroing = true;
    disableAfterSteps = false;
  }else{
    digitalWrite(stepperDirectionPin, steps>0);
    digitalWrite(stepperStepPin, HIGH);
    isZeroing = false;
    disableAfterSteps = true;
  }
  
  // reset step counter parameters
  stepsToTake = abs(steps);
  stepsTaken = 0;
  stepperTtlTimer = 0;
}



