

// set motor speed
void setMotorSpeed(float newSpeed){
  motorSpeed = newSpeed;
  motorDelay = round(mPerMotorTic/motorSpeed*pow(10,6));
}



// get wheel speed
float getWheelSpeed(){
  return (dtSz*mPerWheelTic) / (dtSum*pow(10,-6));  // (m/s)
}




// move stepper motor stepsToTake steps (negative values moves motor backward, positive values move motor forward)
void takeSteps(int stepsToTake){

  // change motor direction if necessary
  if (digitalRead(stepDirPin)!=(stepsToTake>0)){
    digitalWrite(stepDirPin, (stepsToTake>0));
    delayMicroseconds(1); // TCM2100 stepper motor driver requires 20 nanoseconds between stepDirectionPin change and stepPin input // the driver was messing up occasionally, so hopefully this will fix the problem
  }

  for (int i = 0; i < abs(stepsToTake); i++){
    
    // take step
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(motorDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(motorDelay);
  }
  motorTics += stepsToTake;
}




void takeAcceleratingSteps(int stepsToTake, double acceleration, float speedMin, float speedMax){

  // change motor direction if necessary
  if (digitalRead(stepDirPin)!=(stepsToTake>0)){
    digitalWrite(stepDirPin, (stepsToTake>0));
    delayMicroseconds(1); // TCM2100 stepper motor driver requires 20 nanoseconds between stepDirectionPin change and stepPin input // the driver was messing up occasionally, so hopefully this will fix the problem
  }

  for (int i = 0; i < abs(stepsToTake); i++){
    
    // take step
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(motorDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(motorDelay);

    // update speed
    motorSpeed = motorSpeed + acceleration*motorDelay*pow(10,-6);
    motorSpeed = constrain(motorSpeed, speedMin, speedMax);
    setMotorSpeed(motorSpeed);
  }
  motorTics += stepsToTake;
}




// give water reward
void giveWater(){
  noInterrupts();
  wheelTics = 0;
  interrupts();

  obsInd = 0;

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
    
    
    static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
    static uint8_t enc_val = 0;
    
    enc_val = enc_val << 2;
    enc_val = enc_val | ((REG_PIOB_PDSR & (0b11 << 12)) >> 12); // read all pins from port B, and bitmask to get just inputs 12 and 13 from port B, which correspond to Due pins 20 and 21
 
    wheelTics = wheelTics - lookup_table[enc_val & 0b1111];
}




// find start or stop limit switch
void findStartLimit(double speedMin, double speedMax){
  
  setMotorSpeed(speedMin);
  float slowDownDistance = (pow(speedMax,2)-pow(speedMin,2)) / (2*obsAcceleration);  // distance before end stop to start slowing down
  
  while (digitalRead(startLimitPin)){
      if (motorTics*mPerMotorTic > slowDownDistance){
        takeAcceleratingSteps(-1, obsAcceleration, speedMin, speedMax);  // speed up
      }else{
        takeAcceleratingSteps(-1, -obsAcceleration, speedMin, speedMax);  // slow down
      }
  }
  motorTics = 0;  // set motorTics to 0, corresponding to location of start limit switch
}




// find start or stop limit switch
void findStopLimit(double speedMin, double speedMax){
  
  setMotorSpeed(speedMin);
  float slowDownDistance = (pow(speedMax,2)-pow(speedMin,2)) / (2*obsAcceleration);  // distance before end stop to start slowing down
  
  while (digitalRead(stopLimitPin)){
      if (motorTics*mPerMotorTic < (trackEndPosition-slowDownDistance)){
        takeAcceleratingSteps(1, obsAcceleration, speedMin, speedMax);  // speed up
      }else{
        takeAcceleratingSteps(1, -obsAcceleration, speedMin, speedMax);  // slow down
      }
  }
  trackEndPosition = motorTics*mPerMotorTic;
}




// start tracking (ramp up obstacle velocity until it matches velocity of wheel)
void startTracking(){

  // set motor direction to forward
  digitalWrite(stepDirPin, HIGH);
  delayMicroseconds(1); // TCM2100 stepper motor driver requires 20 nanoseconds between stepDirectionPin change and stepPin input

//  wheelSpeed = mPerWheelTic / deltaMicroSmps.getMedian() * pow(10,6);  // (m/s)
//  motorSpeed = trackingSpeeds[1];
//  motorDelay = round(mPerMotorTic/motorSpeed*pow(10,6));
//  
//  while (motorSpeed < wheelSpeed){
//
//    // take step
//    digitalWrite(stepPin, HIGH);
//    delayMicroseconds(motorDelay);
//    digitalWrite(stepPin, LOW);
//    delayMicroseconds(motorDelay);
//    motorTics++;
//
//    // update motor speed
//    motorSpeed = motorSpeed + obsAcceleration*motorDelay*pow(10,-6);
//    motorSpeed = constrain(motorSpeed, trackingSpeeds[0], trackingSpeeds[1]);
//    motorDelay = round(mPerMotorTic/motorSpeed*pow(10,6));
//
//    // update target delay based on current wheel speed
//    wheelSpeed = mPerWheelTic / deltaMicroSmps.getMedian() * pow(10,6);  // (m/s)
//  }

  // record wheel and motor positions at the start of positional tracking  
  noInterrupts();
  wheelTicsTemp = wheelTics;
  interrupts();
  startingWheelTics = wheelTicsTemp;
  startingMotorTics = motorTics;
  setMotorSpeed(trackingSpeeds[1]);
}



void initializeLimits(){
  
  static float slowDown = .4;
  findStartLimit(callibrationSpeeds[0]*slowDown, callibrationSpeeds[1]*slowDown);
  findStopLimit(callibrationSpeeds[0]*slowDown, callibrationSpeeds[1]*slowDown);
  findStartLimit(callibrationSpeeds[0]*slowDown, callibrationSpeeds[1]*slowDown);
  takeAcceleratingSteps(obsStartPos/mPerMotorTic, obsAcceleration, callibrationSpeeds[0], callibrationSpeeds[1]);  // moving back to start position
  digitalWrite(motorOffPin, HIGH);
  
}
