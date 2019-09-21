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
    
    currentMicros = micros();
    // !!! add delta micros to running list here
    lastMicros = currentMicros;
    
    static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
    static uint8_t enc_val = 0;
    
    enc_val = enc_val << 2;
    enc_val = enc_val | ((REG_PIOB_PDSR & (0b11 << 12)) >> 12); // read all pins from port B, and bitmask to get just inputs 12 and 13 from port B, which correspond to Due pins 20 and 21
 
    wheelTics = wheelTics - lookup_table[enc_val & 0b1111];
}




// find start or stop limit switch
void findLimit(motorDirection dir, double speedMin, double speedMax){

  // set motor direction
  digitalWrite(stepDirPin, dir);
  delayMicroseconds(1); // TCM2100 stepper motor driver requires 20 nanoseconds between stepDirectionPin change and stepPin input

  // set initial motor speed !!! should really start at speedMin and accelerate
  motorDelay = round(mPerMotorTic*pow(10,6)/speedMin);
  motorSpeed = speedMin;

  bool foundLimit = false;
  bool slowingDown = false;
  float slowDownDistance = (pow(speedMax,2)-pow(speedMin,2)) / (2*obsAcceleration);  // distance before end stop to start slowing down
  
  while (!foundLimit){
    switch (dir){
      
      // find end limit
      case FORWARD:
        takeSteps(1);
        if (!digitalRead(stopLimitPin)){foundLimit = true;}
        if (motorTics*mPerMotorTic > (trackEndPosition-slowDownDistance)){slowingDown = true;}
        
        break;

      // find start limit
      case REVERSE:
        takeSteps(-1);
        if (!digitalRead(startLimitPin)){foundLimit = true;}
        if (motorTics*mPerMotorTic < slowDownDistance){slowingDown = true;}
        break;
    }
    
    // update velocity
    if (slowingDown){
      motorSpeed = motorSpeed - obsAcceleration*motorDelay*pow(10,-6);
    }else{
      motorSpeed = motorSpeed + obsAcceleration*motorDelay*pow(10,-6);
    }
    motorSpeed = constrain(motorSpeed, speedMin, speedMax);
    motorDelay = round(mPerMotorTic/motorSpeed*pow(10,6));
    motorDelay = constrain(motorDelay-velComputationTime, 1, motorDelay); // a temporary hack
  }

  // update limit switch locations
  if (dir==FORWARD){
    trackEndPosition = motorTics * mPerMotorTic;  // update location of end limit switch
  }else if (dir==REVERSE){
    motorTics = 0;  // set motorTics to 0, corresponding to location of start limit switch
  }

  motorDelay = round(mPerMotorTic*pow(10,6)/trackingSpeeds[0]); // !!! reset motor speed to slow...
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
}



void initializeLimits(){
  
  static float slowDown = .25;
  findLimit(REVERSE, callibrationSpeeds[0]*slowDown, callibrationSpeeds[1]*slowDown);  // find start limit
  findLimit(FORWARD, callibrationSpeeds[0]*slowDown, callibrationSpeeds[1]*slowDown);  // find end limit
  findLimit(REVERSE, callibrationSpeeds[0]*slowDown, callibrationSpeeds[1]*slowDown);  // find start limit
  takeSteps(obsStartPos/mPerMotorTic);  // moving back to start position
  digitalWrite(motorOffPin, HIGH);
  
}
