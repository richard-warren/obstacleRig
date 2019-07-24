// t0 interrupt (every 1 ms monitor stimulus duration and update stimulus power)
ISR(TIMER0_COMPA_vect){

  // update sin table index
  if (isSignalOn){

    // update output based on type of stimulus
    oldValue = newValue;

    switch (stimType){
      
      // sine wave
      case 0:
        newValue = sinLookup[min(round(sinIndex), sinSmps-1)]; // get new value for stimulus power from sin lookup table
        sinIndex += deltaIndex;
        if (sinIndex>sinSmps){sinIndex -= sinSmps;}
        break;

      // step
      case 1:
        newValue = 255;
        break;

      // pulse
      case 2:
        if (pulseTimer<pulseDuration){newValue = 255;}
        else{newValue = 0;}
        
        pulseTimer++;
        if (pulseTimer>=interPulseInterval){pulseTimer = 0;}
        break;
    }
  
    // ramp up and down
    if (rampUpTime>0){
      newValue = newValue * constrain(float(signalTimer)/rampUpTime, 0, 1); // ramp up
    }
    if (rampDownTime>0){
      newValue = newValue * constrain(float(signalDuration-signalTimer)/rampDownTime, 0, 1); // ramp down
    }

    // check whether light should be turned off
    signalTimer++;
    if (signalTimer>signalDuration){
      isSignalOn = false;
      newValue = 0;
    }

    // record whether value has been updated
    if (oldValue != newValue){updated = true;}
  }
}



// turns on light when pin goes high, and turns off light when pin goes low
void stimulusOnOff(){
  if (digitalRead(triggerPin)){
    if (random(0,100)<(signalProbability*100.0)){
      if (randomizeTriggeredPower){
        signalPowerTemp = signalPowers[random(sizeof(signalPowers)/4)];  // divide by 4 because 4 bytes in float
      }else{
        signalPowerTemp = signalPower;
      }
      startSignal();
    }
  }else if (signalTimer<(signalDuration-rampDownTime) && !constantSignalDuration){ // only do this when triggerPin is low
    signalTimer = signalDuration-rampDownTime; // begin ramp down
  }
}



// begins light delivery
void startSignal(){
  signalTimer = 0;
  sinIndex = 0;
  isSignalOn = true;
}



void setFrequency(){
   deltaIndex = sinSmps*hz/fs; // how much to advance in the sin lookup table every ms
   interPulseInterval = round(1.0/hz*1000);
}
