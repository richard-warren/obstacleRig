


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




// read rotary encoder
void encoder_isr() {
    
    currentMicros = micros();
    deltaMicroSmps.add(currentMicros - lastMicros);
    lastMicros = currentMicros;
    
    static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
    static uint8_t enc_val = 0;
    
    enc_val = enc_val << 2;
    enc_val = enc_val | ((REG_PIOB_PDSR & (0b11 << 12)) >> 12); // read all pins from port B, and bitmask to get just inputs 12 and 13 from port B, which correspond to Due pins 20 and 21
 
    wheelTics = wheelTics - lookup_table[enc_val & 0b1111];
}




// take accelerating or decelerating steps starting at startSpeed
void takeSteps(int stepsToTake, double acceleration, double startSpeed, double speedMin, double speedMax, bool checkEndSwitches){  // accelDirection is whether to accelerate (1) or decelerate (-1)

  // set motor direction if it has changed since last time
  if (digitalRead(stepDirPin)!=(stepsToTake>0)){
    digitalWrite(stepDirPin, (stepsToTake>0));
    delayMicroseconds(1); // TCM2100 stepper motor driver requires 20 nanoseconds between stepDirectionPin change and stepPin input
  }

  motorDelay = round(mPerMotorTic/startSpeed*pow(10,6));
  motorSpeed = startSpeed;

  for (int i = 0; i < abs(stepsToTake); i++){

    // take step
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(motorDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(motorDelay);
    motorTics++;
    
    // update velocity
    if (obsAcceleration!=1){
      motorSpeed = motorSpeed + acceleration*motorDelay*pow(10,-6);
      motorSpeed = constrain(motorSpeed, speedMin, speedMax);
      motorDelay = round(mPerMotorTic/motorSpeed*pow(10,6));
    }

    // stop moving if end switches have been reached
    if (checkEndSwitches){
      
      // if moving forward, look for end limit switch
      if (stepsToTake>0){
        if (!digitalRead(stopLimitPin)){break;}
      
      // if moving backward, look for start limit switch
      }else{
        if (!digitalRead(startLimitPin)){break;}
      }
    }
  }
}




// find limit switch at start of track
void findStartLimit(double speedMax){
  if (debug){Serial.print("finding start limit switch... ");}

  digitalWrite(motorOffPin, HIGH); delay(50); digitalWrite(motorOffPin, LOW);
  takeSteps(-(trackEndPosition-trackStartBuffer)/mPerMotorTic, obsAcceleration, obsSpeedMin, obsSpeedMin, speedMax, true);  // take a meter of steps towards the start limit
  takeSteps(-round(10/mPerMotorTic), -obsAcceleration, motorSpeed, obsSpeedMin, speedMax, true);  // decelerate until you touch the start limit switch
  motorTics = 0;  // set motorTics to 0, corresponding to location of start limit switch

  if (debug){Serial.println("start limit switch found");}
}




// find limit switch at start of track
void findEndLimit(double speedMax){
  if (debug){Serial.print("finding end limit switch... ");}
  
  digitalWrite(motorOffPin, HIGH); delay(50); digitalWrite(motorOffPin, LOW);
  takeSteps(round(1/mPerMotorTic), obsAcceleration, obsSpeedMin, obsSpeedMin, speedMax, true);  // take a meter of steps towards the start limit
  trackEndPosition = motorTics * mPerMotorTic;  // set motorTics to 0, corresponding to location of start limit switch

  if (debug){Serial.print("end limit switch found at "); Serial.println(trackEndPosition);}
}




// start tracking (ramp up obstacle velocity until it matches velocity of wheel)
void startTracking(){

  // set motor direction to forward
  digitalWrite(stepDirPin, HIGH);
  delayMicroseconds(1); // TCM2100 stepper motor driver requires 20 nanoseconds between stepDirectionPin change and stepPin input

  wheelSpeed = mPerWheelTic / deltaMicroSmps.getMedian() * pow(10,6);  // (m/s)
  motorSpeed = obsSpeedMin;
  motorDelay = round(mPerMotorTic/motorSpeed*pow(10,6));
  
  while (motorSpeed < wheelSpeed){

    // take step
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(motorDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(motorDelay);
    motorTics++;

    // update motor speed
    motorSpeed = motorSpeed + obsAcceleration*motorDelay*pow(10,-6);
    motorSpeed = constrain(motorSpeed, .01, obsSpeedMax);
    motorDelay = round(mPerMotorTic/motorSpeed*pow(10,6));

    // update target delay based on current wheel speed
    wheelSpeed = mPerWheelTic / deltaMicroSmps.getMedian() * pow(10,6);  // (m/s)
  }

  // record wheel and motor positions at the start of positional tracking  
  noInterrupts();
  wheelTicsTemp = wheelTics;
  interrupts();
  startingWheelTics = wheelTicsTemp;
  startingMotorTics = motorTics;
}



void initializeLimits(){

  findStartLimit(firstCallibrationSpeed);
  findEndLimit(firstCallibrationSpeed);
  findStartLimit(firstCallibrationSpeed);
  
  // todo: step to starting position
  digitalWrite(motorOffPin, HIGH);
  
}
