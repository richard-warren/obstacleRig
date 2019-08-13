


// move stepper one step in stepDirection
void startTracking(){

  // set motor direction to forward
  digitalWrite(stepDirPin, HIGH);

  targetStepDelay = getMotorDelayFromWheelDelay(deltaMicroSmps.getAverage()) / obsGain;
  stepperDelayInd = 0;
  stepDelay = stepperDelays[0];

  while (stepDelay >= targetStepDelay){
    
    // increment motor speed
    stepperDelayInd = min(stepperDelayInd+1, maxStepperDelayInd);
    stepDelay = max(stepperDelays[stepperDelayInd], targetStepDelay);

    // take step
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
    stepperTicks++;

    // update target delay based on current wheel speed
    targetStepDelay = getMotorDelayFromWheelDelay(deltaMicroSmps.getAverage());
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
      digitalWrite(obsLightPin2, HIGH);
    }
  }
}


// computes distance in (mm) needed to get from velStart to velEnd at a certain acceleration
float getDecellerationDistance(float velStart, float velEnd, float acceleration){

  float dt = (velEnd - velStart) / acceleration;
  return (velStart*dt + 0.5*acceleration*pow(dt,2)) * 1000;
  
}


// move stepper one step in stepDirection
void takeStep(int stepsToTake){

  // set motor direction
  digitalWrite(stepDirPin, (stepsToTake>0));
  delayMicroseconds(1); // TCM2100 stepper motor driver requires 20 nanoseconds between stepDirectionPin change and stepPin input // the driver was messing up occasionally, so hopefully this will fix the problem

  for (int i = 0; i < abs(stepsToTake); i++){

    // take step
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(maxSpeedDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(maxSpeedDelay);
  }

  stepperTicks += stepsToTake;
}



// move stepper one step in stepDirection
void takeAcceleratingStep(int stepsToTake, int accelDirection, int delayLimit){

  // set motor direction
  digitalWrite(stepDirPin, (stepsToTake>0));
  delayMicroseconds(1); // TCM2100 stepper motor driver requires 20 nanoseconds between stepDirectionPin change and stepPin input // the driver was messing up occasionally, so hopefully this will fix the problem

  for (int i = 0; i < abs(stepsToTake); i++){

    switch (accelDirection){

      // slow down
      case -1:
        if (stepperDelays[stepperDelayInd]<delayLimit){
          stepperDelayInd--;
          stepDelay = min(stepperDelays[stepperDelayInd], delayLimit);
        }
        break;

      // speed up
      case 1:
        if (stepperDelays[stepperDelayInd]>delayLimit){
          stepperDelayInd++;
          stepDelay = max(stepperDelays[stepperDelayInd], delayLimit);
        }
        break;
    }

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



void getStartLimit(float slowDownFactor, bool onlyRampUp){

  stepperDelayInd = 0; // start at lowest velocity
  
  // find start limit
  while (digitalRead(startLimitPin)){
    
    // speed up at the beginning
    if (stepperTicks>ticsBeforeStartLimitToSlowDown || onlyRampUp){
      takeAcceleratingStep(-1, 1, long(callibrationDelay/slowDownFactor));
    }
    
    // slow down towards the end
    else{
      takeAcceleratingStep(-1, -1, long(maxCallibrationDelay/slowDownFactor));
    }
  }
  stepperTicks = 0;
  
}



void getEndLimit(float slowDownFactor){

  stepperDelayInd = 0; // start at lowest velocity

  // find stop limit
  while (digitalRead(stopLimitPin)){
    takeAcceleratingStep(1, 1, long(callibrationDelay/slowDownFactor));
  }
  stepperStopPosition = stepperTicks - endPositionBuffer;

}



void initializeLimits(float slowDownFactor){

  getStartLimit(slowDownFactor, true);
  digitalWrite(motorOffPin, HIGH); delay(50); digitalWrite(motorOffPin, LOW); // turni
  getEndLimit(slowDownFactor);
  digitalWrite(motorOffPin, HIGH); delay(50); digitalWrite(motorOffPin, LOW);
  getStartLimit(slowDownFactor, false);
  digitalWrite(motorOffPin, HIGH); delay(50); digitalWrite(motorOffPin, LOW);
  stepperDelayInd = 0; // start at lowest velocity
  takeAcceleratingStep(startPositionBuffer + getStartJitter(startPosJitter), 1, callibrationDelay); // move back to startPositionBuffer
  digitalWrite(motorOffPin, HIGH);
  
}



void recalibrateLimits(){

  // reset stepper motor driver
  digitalWrite(motorOffPin, HIGH);
  delay(50);
  digitalWrite(motorOffPin, LOW);

  // return to beginning of track
  delay(servoSwingTime); // delay before returning the platform to avoid whacking the mouse in the butt
  getStartLimit(1.0, false);
  if (state==3){
    digitalWrite(obstaclePin, HIGH);
  }

  // return to starting position
  stepperDelayInd = 0; // start at lowest velocity
  takeAcceleratingStep(startPositionBuffer + getStartJitter(startPosJitter), 1, callibrationDelay); // move back to startPositionBuffer
  digitalWrite(motorOffPin, HIGH); // disengage stepper motor driver
  
}




void printMenuAndSettings(){

  // print settings
  Serial.println("CURRENT SETTINGS:");
  Serial.print("condition: ");
  Serial.println(conditionNames[state-1]);
  Serial.print("reward rotations: ");
  Serial.println(rewardRotations);
  Serial.print("osbtacle gain: ");
  Serial.println(obsGain);
  Serial.print("water duration: ");
  Serial.println(waterDuration);
  Serial.print("light on probability: ");
  Serial.println(obsLightProbability);
  Serial.println("");
  delay(500);

  // print menu
  Serial.println("MENU:");
  Serial.println("1: set condition to rewards only");
  Serial.println("2: set condition to platform, no obstacles");
  Serial.println("3: set condition to platform, with obstacles");
  Serial.println("4: set reward rotation number");
  Serial.println("5: set obstacle gain");
  Serial.println("6: set water duration");
  Serial.println("7: set obstacle light on probability");
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
        rewardPosition = (rewardRotations * encoderSteps) / obsGain;
        break;
        
      // set gain
      case 5:
        Serial.print("enter obstacle gain...\n\n");
        while (Serial.available() == 0)  {}
        obsGain = Serial.parseFloat();
        initializeObsLocations();
        printMenuAndSettings();
        break;

      // enter water duration
      case 6:
        Serial.print("enter water duration...\n\n");
        while (Serial.available() == 0)  {}
        waterDuration = Serial.parseInt();
        printMenuAndSettings();
        break;
      
      // enter obs light on probability
      case 7:
        Serial.print("enter light on probability...\n\n");
        while (Serial.available() == 0)  {}
        obsLightProbability = Serial.parseFloat();
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




long getMotorDelayFromSpeed(float motorSpeed){

    long ticsPerSecond = motorSpeed * (motorSteps*microStepping* 1000) / (2*PI*timingPulleyRad);
    int microSecondsPerTic = (1 / (ticsPerSecond / pow(10,6))) / 2;
    return microSecondsPerTic;

}



int getMotorDelayFromWheelDelay(int wheelDelay){

  return (wheelDelay * (mmPerMotorTic / ((2*PI*wheelRad) / encoderSteps))) / 2;
  
}



int setObsPos(int index){

  static double ticsPerMm = (encoderSteps / (2*PI*wheelRad));
  return obstacleLocationSteps[index] + random(obsPosJitter[0]*(1/obsGain)*ticsPerMm, obsPosJitter[1]*ticsPerMm); // initialize first position)

}



int getStartJitter(int jitterMax){

  return random(0, jitterMax/mmPerMotorTic);
  
}



void initializeObsLocations(){

  for (int i=0; i<sizeof(obstacleLocations); i++){
    obstacleLocationSteps[i] = (obstacleLocations[i] * encoderSteps) / obsGain;
  }
  obsPos = setObsPos(0); // set first obstacle position
//  if (stimulusAtObsPositions){stimulusPosition = obsPos;}
  
}
