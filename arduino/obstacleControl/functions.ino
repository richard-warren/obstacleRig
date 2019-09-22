
// get motor step delay that causes motor to move at desired speed
int getMotorDelay(float motorSpeed){
  return round(mPerMotorTic / motorSpeed * pow(10,6));
}




// get wheel speed
float getWheelSpeed(){
  return (dtSz*mPerWheelTic) / (dtSum*pow(10,-6));  // (m/s)
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
  noInterrupts();
  wheelTics = 0;
  interrupts();

  obsInd = 0;
  obsLocation = obsLocations[obsInd];

  digitalWrite(waterPin, HIGH);
  delay(waterDuration);
  digitalWrite(waterPin, LOW);
}




// read wheel rotary encoder
void encoder_isr() {
    
    // get time elapsed from last wheel tic
    long currentMicros = micros();
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




// start tracking (ramp up obstacle velocity until it matches velocity of wheel)
void startTracking(){

  static int stepsBtwnChecks = 5;  // how many accelerating steps to take between checking the wheel velocity // getWheelSpeed() takes about 12 microseconds, so computing this every time can be cumbersome

  speedInd = 0;  // start at lowest speed
  while (speeds[speedInd] < getWheelSpeed()){
    takeSteps(stepsBtwnChecks, ACCELERATE, obsSpeedMin, trackingSpeed);
  }
  speedInd = maxSpeedInd;  // set speed to maximum for subsequent tracking

  // record wheel and motor positions at the start of positional tracking  
  noInterrupts();
  wheelTicsTemp = wheelTics;
  interrupts();
  startingWheelTics = wheelTicsTemp;
  startingMotorTics = motorTics;
}




void initializeLimits(){
    
  static float slowDown = 1.0;  // (0->1) how much to slow down initial limit check relative to callibrationSpeed
  findStartLimit(obsSpeedMin*slowDown, callibrationSpeed*slowDown);
  findStopLimit(obsSpeedMin*slowDown, callibrationSpeed*slowDown);
  findStartLimit(obsSpeedMin*slowDown, callibrationSpeed*slowDown);
  takeSteps(obsStartPos/mPerMotorTic, ACCELERATE, obsSpeedMin, callibrationSpeed);  // moving back to start position
  digitalWrite(motorOffPin, HIGH);
}




// find start or stop limit switch
void findStartLimit(double speedMin, double speedMax){
  
  speedInd = 0;  // start at lowest speed
  float slowDownDistance = (pow(speedMax,2)-pow(speedMin,2)) / (2*obsAcceleration);  // distance before end stop to start slowing down (in meters)
  int targetMotorTics = slowDownDistance/mPerMotorTic;  // distance before end stop to start slowing down (in motor tics)
  
  while (digitalRead(startLimitPin)){
      if (motorTics > targetMotorTics){
        takeSteps(-1, ACCELERATE, speedMin, speedMax);  // speed up
      }else{
        takeSteps(-1, DECELERATE, speedMin, speedMax);  // slow down
      }
  }
  motorTics = 0;  // set motorTics to 0, corresponding to location of start limit switch
}




// find start or stop limit switch
void findStopLimit(double speedMin, double speedMax){
  
  speedInd = 0;  // start at lowest speed
  float slowDownDistance = (pow(speedMax,2)-pow(speedMin,2)) / (2*obsAcceleration);  // distance before end stop to start slowing down (in meters)
  int targetMotorTics = (trackEndPosition-slowDownDistance)/mPerMotorTic;  // distance before end stop to start slowing down (in motor tics)
  
  while (digitalRead(stopLimitPin)){
      if (motorTics < targetMotorTics){
        takeSteps(1, ACCELERATE, speedMin, speedMax);  // speed up
      }else{
        takeSteps(1, DECELERATE, speedMin, speedMax);  // slow down
      }
  }
  trackEndPosition = motorTics*mPerMotorTic;
}
