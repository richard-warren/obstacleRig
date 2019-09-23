
// get motor step delay that causes motor to move at desired speed
float getMotorDelay(float motorSpeed){
  return mPerMotorTic / motorSpeed * pow(10,6);
}




float getJitter(float jitter){
  return random(-jitter*pow(10,3), jitter*pow(10,3)) * pow(10,-3);
}




// move motor
void takeSteps(int stepsToTake, accel a, float speedMin, float speedMax){

  // change motor direction if necessary
  if (digitalRead(stepDirPin)!=(stepsToTake>0)){
    digitalWrite(stepDirPin, (stepsToTake>0));
    delayMicroseconds(1); // TCM2100 stepper motor driver requires 20 nanoseconds between stepDirectionPin change and stepPin input // the driver was messing up occasionally, so hopefully this will fix the problem
  }

  for (int i = 0; i < abs(stepsToTake); i++){
    
    // take step
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delays[speedInd]);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delays[speedInd]);

    // update speed
    switch (a){
      case ACCELERATE:
        if (speeds[speedInd]<speedMax){speedInd++;}
        break;

      case DECELERATE:
        if (speeds[speedInd] > speedMin){speedInd--;}
        break;
    }
  }
  motorTics += stepsToTake;
}




// move motor // like takeSteps, but faster computationally because doesn't check for acceleration changes and only operates at max speed
void takeStepsFast(int stepsToTake){

  // change motor direction if necessary
  if (digitalRead(stepDirPin)!=(stepsToTake>0)){
    digitalWrite(stepDirPin, (stepsToTake>0));
    delayMicroseconds(1); // TCM2100 stepper motor driver requires 20 nanoseconds between stepDirectionPin change and stepPin input // the driver was messing up occasionally, so hopefully this will fix the problem
  }

  for (int i = 0; i < abs(stepsToTake); i++){
    
    // take step
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delays[maxSpeedInd]);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delays[maxSpeedInd]);
  }
  motorTics += stepsToTake;
}




// give water reward
void giveWater(){
  digitalWrite(waterPin, HIGH);
  delay(waterDuration);
  digitalWrite(waterPin, LOW);
  resetState();
}




// read wheel rotary encoder
void encoder_isr() {
    
    if (notGettingInput){  // don't update wheel position while collecting user input
      // get time elapsed from last wheel tic
      volatile long currentMicros = micros();
      wheelDts[dtInd] = currentMicros-lastMicros;
      lastMicros = currentMicros;
      dtSum += wheelDts[dtInd];  // add the newest delta
      dtInd++;
      if (dtInd==dtSz){dtInd = 0;}
      dtSum-= wheelDts[dtInd];  // remove the oldest delta
      
      // read encoder state
      static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
      static uint8_t enc_val = 0;
      enc_val = enc_val << 2;
      enc_val = enc_val | ((REG_PIOB_PDSR & (0b11 << 12)) >> 12); // read all pins from port B, and bitmask to get just inputs 12 and 13 from port B, which correspond to Due pins 20 and 21
   
      // update wheelTics
      wheelTics = wheelTics - lookup_table[enc_val & 0b1111];
    }
}




// start tracking (ramp up obstacle velocity until it matches velocity of wheel)
void startTracking(){

  static int stepsBtwnChecks = 20;         // how many accelerating steps to take between checking the wheel velocity // getWheelSpeed() takes ~12 microseconds, so computing this every time can be cumbersome
  static float speedCompensation = 0.4;    // this code over-estimates the motor speed due to computational delays // estimated speed is multiplied by slowDownFactor to compensate for that

  speedInd = 0;  // start at lowest speed
  while (speeds[speedInd]*speedCompensation < getWheelSpeed() && digitalRead(stopLimitPin)){
    takeSteps(stepsBtwnChecks, ACCELERATE, obsSpeedStart, trackingSpeed);
  }

  // record wheel and motor positions at the start of positional tracking  
  noInterrupts();
  wheelTicsTemp = wheelTics;
  interrupts();
  startingWheelTics = wheelTicsTemp;
  startingMotorTics = motorTics;
}




// get wheel speed
float getWheelSpeed(){
  return (dtSz*mPerWheelTic) / (dtSum*pow(10,-6));  // (m/s)
}




void calibrateLimits(){
  
  Serial.println(F("Calibrating limits..."));
  static float slowDown = 0.5;  // (0->1) how much to slow down initial limit check relative to callibrationSpeed

  digitalWrite(obsOutPin, LOW);
  isObsOn = false;
  isObsTracking = false;
  
  digitalWrite(motorOffPin, LOW);
  findStartLimit(obsSpeedStop, obsSpeedStop, obsSpeedStop);  // move at constant, slow speed
  findStopLimit(obsSpeedStart, obsSpeedStop, callibrationSpeed*slowDown);
  findStartLimit(obsSpeedStart, obsSpeedStop, callibrationSpeed*slowDown);
  takeSteps(max((obsStartPos+getJitter(obsStartPosJitter))/mPerMotorTic,0), ACCELERATE, obsSpeedStart, callibrationSpeed);  // move back to start position
  digitalWrite(motorOffPin, HIGH);

  printInitializations();
  printMenu();
  resetState();
}




// find start or stop limit switch
void findStartLimit(float speedStart, float speedStop, float speedMax){
  
  speedInd = 0;  // start at lowest speed
  float slowDownDistance = (pow(speedMax,2)-pow(speedStop,2)) / (2*obsAcceleration);  // distance before end stop to start slowing down (in meters)
  int targetMotorTics = slowDownDistance/mPerMotorTic;  // distance before end stop to start slowing down (in motor tics)
  
  while (digitalRead(startLimitPin)){
      if (motorTics > targetMotorTics){
        takeSteps(-1, ACCELERATE, speedStart, speedMax);  // speed up
      }else{
        takeSteps(-1, DECELERATE, speedStop, speedMax);  // slow down
      }
  }
  motorTics = 0;  // set motorTics to 0, corresponding to location of start limit switch
}




// find start or stop limit switch
void findStopLimit(float speedStart, float speedStop, float speedMax){
  
  speedInd = 0;  // start at lowest speed /// !!! should actually check where speedStart is in lookup table
  float slowDownDistance = (pow(speedMax,2)-pow(speedStop,2)) / (2*obsAcceleration);  // distance before end stop to start slowing down (in meters)
  int targetMotorTics = (trackEndPosition-slowDownDistance)/mPerMotorTic;  // distance before end stop to start slowing down (in motor tics)
  
  while (digitalRead(stopLimitPin)){
      if (motorTics < targetMotorTics){
        takeSteps(1, ACCELERATE, speedStart, speedMax);  // speed up
      }else{
        takeSteps(1, DECELERATE, speedStop, speedMax);  // slow down
      }
  }
  trackEndPosition = motorTics*mPerMotorTic;
}




// reset state variables
void resetState(){
  
  // reset wheel ticks
  noInterrupts();
  wheelTics = 0;
  wheelTicsTemp = 0;
  interrupts();

  // set initial obstacle location
  obsInd = 0;
  obsLocation = obsLocations[obsInd] + getJitter(obsLocationJitter);
}




// turn lights on/off
void switchObsLight(bool lightState){

  digitalWrite(obsLightPinDig, lightState);
  
  if (lightState){
    analogWrite(obsLightPin1, 255*obstacleBrightness);
    analogWrite(obsLightPin2, 255*obstacleBrightness);
  }else{
    digitalWrite(obsLightPin1, LOW);
    digitalWrite(obsLightPin2, LOW);
  }
}
