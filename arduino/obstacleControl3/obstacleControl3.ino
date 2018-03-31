// OBSTACLE CONTROL
#include <RunningMedian.h>


// pin assignments
#define stepPin         4
#define stepDirPin      5
#define encoderPinA     20   // don't change encoderPins, because direct port manipulation is used that assumes 20 and 21 are used for encoder pins
#define encoderPinB     21
#define waterPin        7
#define obstaclePin     13   // signals whether the obstacle is engaged... this is sent to an arduino that controls the obstacle servo
#define motorOffPin     8    // turns on stepper motor driver
#define startLimitPin   9    // signal is LOW when engaged
#define stopLimitPin    10   // signal is LOW when engaged
#define obsLightPin     2    // controls when the light for the obstacle turns on
#define obsOnPin        3    // when the obstacle is engaged in a trial, i.e. tracking the mouse's position


// user settings
volatile int state = 3; // 1: no platform movement, no obstaacles, 2: platform movement, no obstacles, 3: platform movement and obstacles
const int servoSwingTime = 200; // ms, approximate amount of time it takes for the osbtacle to pop out // this is used as a delay bewteen the obstacle reaching the end of the track and it coming back, to avoid it whacking the guy in the butt!
const float acceleration = 5.0; // (m/(s^2))
volatile float rewardRotations = 9.01;
const int startPositionMm = 5;
const int endPositionMm = 25;
const float minSpeed = .01; // (m/s) can't use zero speed, b/c corresponding delay is inf, so start motor off at minSpeed
const int waterDuration = 80; // milliseconds
const double maxStepperSpeed = 1.5; // (m/s)
volatile float callibrationSpeed = .6; // speed with which motor moves platform during callibration (m/s)
const float obstacleLocations[] = {1.5, 4.5, 7.5, rewardRotations*20}; // expressed in wheel ticks // the last element is a hack... the index goes up and the wheel position will never reach the last value, which is the desired behavior
const int velocitySamples = 10; // average velocity of wheel over this many samples // each sample last about 500 microseconds
const int obsPosJitter[] = {-100, 100}; // jitter range for the onset position of obstacles (mm)
const int startPosJitter = 20; // (mm)
const float obsLightProbability = 0.5;


// rig characteristics
const int microStepping = 16; // only (1/microStepping) steps per pulse // this should correspond to the setting on the stepper motor driver, which is set by 3 digital inputs
const int motorSteps = 200;
const int encoderSteps = 2880; // 720cpr * 4
const double timingPulleyRad = 15.2789; // (mm)
const double wheelRad = 95.25; // (m)


// initializations
const int startPositionBuffer = startPositionMm * ((motorSteps*microStepping) / (2*PI*timingPulleyRad));
const int endPositionBuffer = endPositionMm  * ((motorSteps*microStepping) / (2*PI*timingPulleyRad));; // motor stops endPositionBuffer steps before the beginning and end of the track
char* conditionNames[] = {"rewards only", "platform, no obstacles", "platform, with obstacles"};
volatile int userInput;
volatile int wheelTicks = 0;
volatile int wheelTicksTemp = 0; // this variable temporarily copies wheelTicks in the main code to avoid having to access it multiple times, potentially colliding with its access in the interrupt
volatile int obstacleInd = 0; // keeps track of which obstacle is being delivered for each reward trial
volatile int stepperTicks = 0;
volatile int stepperStopPosition; // value to be determined by call to initializeLimits in setup()
volatile int targetStepperTicks = startPositionBuffer;
volatile int stepsToTake = 0; // when driving the motor, stepsToTake is how many motor ticks required to get to target position
volatile int rewardPosition = rewardRotations * encoderSteps; // expressed in wheelTicks
const double conversionFactor = (wheelRad / timingPulleyRad) * (float(motorSteps) / encoderSteps) * microStepping; // this converts from analogRead reading of wheel encoder to desired number of steps in stepper driver
volatile bool stepDir = HIGH;
volatile bool obstacleEngaged = false;
volatile float stepSpeed;
volatile float targetSpeed;
volatile float stepDelay;
volatile float targetStepSpeed;
volatile long currentMicros = 0;
volatile long lastMicros = 0;
volatile int deltaMicros = 0;
volatile int callibrationDelay; // tbd in setup
volatile int maxSpeedDelay; // tbd in setup
volatile int startingWheelTics = 0;
volatile int startingStepperTics = 0;
volatile int obstacleLocationSteps[sizeof(obstacleLocations)];
volatile int obsPos; // position of obstacle, in wheel tics, on a given trial
RunningMedian deltaMicroSmps = RunningMedian(5);



void setup() {
  
  // prepare ins and outs
  pinMode(stepPin, OUTPUT);
  pinMode(stepDirPin, OUTPUT);
  pinMode(waterPin, OUTPUT);
  pinMode(motorOffPin, OUTPUT);
  pinMode(obstaclePin, OUTPUT);
  pinMode(startLimitPin, INPUT_PULLUP);
  pinMode(stopLimitPin, INPUT_PULLUP);
  pinMode(obsLightPin, OUTPUT);
  pinMode(obsOnPin, OUTPUT);
  
  digitalWrite(stepPin, LOW);
  digitalWrite(stepDirPin, stepDir);
  digitalWrite(waterPin, LOW);
  digitalWrite(motorOffPin, LOW);
  digitalWrite(obstaclePin, LOW);
  digitalWrite(obsLightPin, LOW);
  digitalWrite(obsOnPin, LOW);


  // initialiez random seed
  randomSeed(analogRead(0));
  

  // initialize encoder hardware interrupt
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), encoder_isr, CHANGE);

  
  // begin serial communication
  Serial.begin(115200);
  printMenuAndSettings();
  
  
  // initialize delays for max and callibration speeds
  callibrationDelay = getMotorDelayFromSpeed(callibrationSpeed);
  maxSpeedDelay = getMotorDelayFromSpeed(maxStepperSpeed);

  
  // initialize locations of obstacles
  for (int i=0; i<sizeof(obstacleLocations); i++){
    obstacleLocationSteps[i] = obstacleLocations[i] * encoderSteps;
  }
  setObsPos(0); // set first obstacle position
   
  
  // initialize track limits and move to starting position
  initializeLimits();
}




void loop(){

  // check for user input
  getUserInput();
  
  
  // store current wheelTicks value with interrupts disabled
  noInterrupts();
  wheelTicksTemp = wheelTicks;
  interrupts();

  
  // check if obstacle should be engaged or disengaged  
  
  // engage obstacle:
  if (!obstacleEngaged & (wheelTicksTemp >= obsPos)){

    switch (state){
      
      // platform, no obstacles
      case 2:
        obstacleEngaged = true;
        digitalWrite(motorOffPin, LOW); // engages stepper motor driver
        startTracking();
        break;

      // platform, with obstacles
      case 3:
        obstacleEngaged = true;
        digitalWrite(motorOffPin, LOW); // engages stepper motor driver
        digitalWrite(obsOnPin, HIGH);
        startTracking();
        break;
      }
  }
   
  // disengage:
  else if (obstacleEngaged & (stepperTicks >= stepperStopPosition)){
    
    obstacleEngaged = false;
    digitalWrite(obstaclePin, LOW);
    digitalWrite(obsLightPin, LOW);
    digitalWrite(obsOnPin, LOW);
    obstacleInd++;
    setObsPos(obstacleInd);
    recalibrateLimits();
    
  }
       


  // give water if reward location reached
  if (wheelTicksTemp > rewardPosition){
    giveReward();
  }
  

  
  // compute and move to target stepper position
  if (obstacleEngaged){
    
    targetStepperTicks = ((wheelTicksTemp - startingWheelTics) * conversionFactor) + startingStepperTics;
    targetStepperTicks = constrain(targetStepperTicks, startPositionBuffer, stepperStopPosition);

    stepsToTake = targetStepperTicks - stepperTicks;
    
    if (stepsToTake!=0){
      takeStep(stepsToTake);
    }
  }
}




// begin tracking by accelerating obs until speed matches that of wheel
void startTracking(){

  // set motor direction to forward
  digitalWrite(stepDirPin, HIGH);

  targetSpeed = getSpeedFromWheelDelay(deltaMicroSmps.getAverage());
  setMotorSpeedToMin();

  while (stepSpeed <= targetSpeed){
    
    // get step speed and delay
//    stepSpeed = constrain(stepSpeed + acceleration, 0, maxStepperSpeed);
    stepSpeed = constrain(stepSpeed + acceleration*stepDelay*pow(10,-6), 0, targetSpeed);
    stepDelay = getMotorDelayFromSpeed(stepSpeed);

    // take step
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
    stepperTicks++;

    // update target delay based on current wheel speed
    targetSpeed = getSpeedFromWheelDelay(deltaMicroSmps.getAverage());
  }

  // record wheel and motor positions at the start of positional tracking  
  noInterrupts();
  wheelTicksTemp = wheelTicks;
  interrupts();
  startingWheelTics = wheelTicksTemp;
  startingStepperTics = stepperTicks;

  // turn on obstacle light
  if (state==3){
    if(random(0,100) < obsLightProbability*100.0){
      digitalWrite(obsLightPin, HIGH);
    }
  }
}




// move stepper one step in stepDirection
void takeStep(int stepsToTake){

  // set motor direction
  digitalWrite(stepDirPin, (stepsToTake>0));
  delayMicroseconds(1); // TCM2100 stepper motor driver requires 20 nanoseconds between stepDirectionPin change and stepPin input // the driver was messing up occasionally, so hopefully this will fix the problem

  // take steps
  for (int i = 0; i < abs(stepsToTake); i++){
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(maxSpeedDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(maxSpeedDelay);
  }

  stepperTicks += stepsToTake;
}



// move stepper one step in stepDirection
void takeAcceleratingStep(int stepsToTake, float acceleration, float minSpeed, float maxSpeed){

  // set motor direction
  digitalWrite(stepDirPin, (stepsToTake>0));
  delayMicroseconds(1); // TCM2100 stepper motor driver requires 20 nanoseconds between stepDirectionPin change and stepPin input // the driver was messing up occasionally, so hopefully this will fix the problem

  for (int i = 0; i < abs(stepsToTake); i++){

    // get motor speed and delay
    stepSpeed = constrain(stepSpeed + acceleration*stepDelay*pow(10,-6), minSpeed, maxSpeed);
//    Serial.println(stepSpeed + acceleration*stepDelay*pow(10,-6));
    stepDelay = getMotorDelayFromSpeed(stepSpeed);

    // take step
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
  }

  stepperTicks += stepsToTake;
}




// read rotary encoder
void encoder_isr() {
    
    currentMicros = micros();
    deltaMicroSmps.add(currentMicros - lastMicros);
    lastMicros = currentMicros;
    
    static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
    static uint8_t enc_val = 0;
    
    enc_val = enc_val << 2;
    enc_val = enc_val | ((REG_PIOB_PDSR & (0b11 << 12)) >> 12); // read all pins from port B, and bitmask to get just inputs 12 and 13 from port B, which correspond to Due pins 20 and 21
 
    wheelTicks = wheelTicks - lookup_table[enc_val & 0b1111];

}



void getStartLimit(){

  setMotorSpeedToMin(); // start at lowest velocity
  
  // find start limit
  while (digitalRead(startLimitPin)){
    takeAcceleratingStep(-1, acceleration, 0, callibrationSpeed);
  }
  stepperTicks = 0;
  
}



void getEndLimit(){

  setMotorSpeedToMin(); // start at lowest velocity

  // find stop limit
  while (digitalRead(stopLimitPin)){
    takeAcceleratingStep(1, acceleration, 0, callibrationSpeed);
  }
  stepperStopPosition = stepperTicks - endPositionBuffer;

}



void initializeLimits(){

  setMotorSpeedToMin();
  getStartLimit();
  getEndLimit();
  getStartLimit();
  setMotorSpeedToMin();
  takeAcceleratingStep(startPositionBuffer + getStartJitter(startPosJitter), acceleration, 0, callibrationSpeed); // move back to startPositionBuffer
  digitalWrite(motorOffPin, HIGH);
  
}




void recalibrateLimits(){

  // reset stepper motor driver
  digitalWrite(motorOffPin, HIGH);
  delay(50);
  digitalWrite(motorOffPin, LOW);

  // return to beginning of track
  delay(servoSwingTime); // delay before returning the platform to avoid whacking the mouse in the butt
  getStartLimit();
  if (state==3){
    digitalWrite(obstaclePin, HIGH);
  }

  // return to starting position
  setMotorSpeedToMin();
  takeAcceleratingStep(startPositionBuffer + getStartJitter(startPosJitter), acceleration, 0, callibrationSpeed); // move back to startPositionBuffer
  digitalWrite(motorOffPin, HIGH); // disengage stepper motor driver
  
}




void printMenuAndSettings(){

  // print settings
  Serial.println("CURRENT SETTINGS:");
  Serial.print("condition: ");
  Serial.println(conditionNames[state-1]);
  Serial.print("reward rotations: ");
  Serial.println(rewardRotations);
  Serial.println("");
  delay(500);

  // print menu
  Serial.println("MENU:");
  Serial.println("1: set condition to rewards only");
  Serial.println("2: set condition to platform, no obstacles");
  Serial.println("3: set condition to platform, with obstacles");
  Serial.println("4: set reward rotation number");
  Serial.println("5: get current settings");
  Serial.println("");
  delay(500);
  
}




void getUserInput(){

  // check for user input
  if (Serial.available()){
    
    userInput = Serial.parseInt();
    
    switch (userInput){

      // set condition to rewards only
      case 1:
        state = userInput;
        printMenuAndSettings();
        digitalWrite(obstaclePin, LOW);

        // reset wheel ticks
        noInterrupts();
        wheelTicks = 0;
        interrupts();
        break;
        
      // set condition to platform movement, no obstacles
      case 2:
        state = userInput;
        printMenuAndSettings();
        digitalWrite(obstaclePin, LOW);
        break;
        
      // set condition to platform movement with obstacles
      case 3:
        state = userInput;
        printMenuAndSettings();
        digitalWrite(obstaclePin, HIGH);

        // reset wheel ticks
        noInterrupts();
        wheelTicks = 0;
        interrupts();

        // prepare first obstacle position
        obstacleInd = 0;
        obsPos = setObsPos(obstacleInd);
        break;
        
      // enter reward rotation number
      case 4:
        Serial.print("enter reward rotations...\n\n");
        while (Serial.available() == 0)  {}
        rewardRotations = Serial.parseFloat();
        printMenuAndSettings();
        rewardPosition = rewardRotations * encoderSteps;
        break;
        
      // print settings
      case 5:
        printMenuAndSettings();
        break;
    }
  }
}




void giveReward(){
  
    wheelTicksTemp = 0;
    
    noInterrupts();
    wheelTicks = 0;
    interrupts();

    obstacleInd = 0;
    obsPos = setObsPos(obstacleInd);
    digitalWrite(waterPin, HIGH);
    delay(waterDuration);
    digitalWrite(waterPin, LOW);

}




void setMotorSpeedToMin(){

  setMotorSpeedToMin();
  stepDelay = getMotorDelayFromSpeed(stepSpeed);;
  
}




long getMotorDelayFromSpeed(float motorSpeed){

    long ticsPerSecond = motorSpeed * (motorSteps*microStepping* 1000) / (2*PI*timingPulleyRad);
    int microSecondsPerTic = (1 / (ticsPerSecond / pow(10,6))) / 2;
    return microSecondsPerTic;

}



double getSpeedFromWheelDelay(int wheelDelay){

  return ((2*PI*wheelRad)/encoderSteps) / (wheelDelay) * 1000; // speed of 
  
}



int setObsPos(int index){

  static double ticsPerMm = (encoderSteps / (2*PI*wheelRad));
  obsPos =  obstacleLocationSteps[index] + random(obsPosJitter[0]*ticsPerMm, obsPosJitter[1]*ticsPerMm); // initialize first position)

}



int getStartJitter(int jitterMax){

  static double ticsPerMm = ((motorSteps*microStepping) / (2*PI*timingPulleyRad));
  return random(0, ticsPerMm*jitterMax);
  
}


