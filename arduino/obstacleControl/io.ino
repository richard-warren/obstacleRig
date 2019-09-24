
void printMenu(){

  // print settings
  Serial.println("\nCURRENT SETTINGS:");

  // condition
  Serial.print("1: obstacle status: ");
  if (isObsOn){
    Serial.println("ON");
  }else{
    Serial.println("OFF");
  }
  
  // water distance
  Serial.print("2: water distance (meters): ");
  Serial.println(waterDistance);

  // water duration
  Serial.print("3: water duration (milliseconds): ");
  Serial.println(waterDuration);
  
  // light on probability
  Serial.print("4: light on probability: ");
  Serial.println(obsLightProbability);

  // recallibrate limits
  Serial.println("5: recallibrate");
}




// get initial menu selection
void getUserInput(){

  // check for user input
  if (Serial.available()){
    inputChar = Serial.read();
    Serial.readBytesUntil('\n', inputBuffer, 100);  // throw away all but the first byte, stopping at the newline character
    
    // recallibrate limits immediately if requested
    if (inputChar=='5'){
      calibrateLimits();
      
    
    // otherwise set flag so input can be processed when obstacle is not moving
    }else{
      inputDetected = true;
    }
  }
}




// process menu selection
void processUserInput(){

  if (inputDetected){
    
    notGettingInput = false;
    
    switch (inputChar){
  
      // turn obstacle on/off
      case '1':
        isObsOn = !isObsOn;
        digitalWrite(obsOutPin, isObsOn);
        resetState();
        printMenu();
        break;
        
      
      // set water distance
      case '2':
        Serial.println(F("Enter water distance (meters)..."));
        while (Serial.available() == 0)  {}
        inputFloat = Serial.parseFloat();
        while (Serial.available()){Serial.read();} // throw away extra bytes
        if (inputFloat>0 && inputFloat<=100){
          waterDistance = inputFloat;
          printMenu();
        }else{
          Serial.println(F("ERROR: please enter an integer or decimal >0 and <=100!"));
        }
        break;
        
      
      // set water duration
      case '3':
        Serial.println(F("Enter water time (milliseconds)..."));
        while (Serial.available() == 0)  {}
        inputInt = Serial.parseInt();
        while (Serial.available()){Serial.read();} // throw away extra bytes
        if (inputInt>0 && inputInt<=1000){
          waterDuration = inputInt;
          printMenu();
        }else{
          Serial.println(F("ERROR: please enter an integer >0 and <=1000!"));
        }
        break;
  
      
      // set light on probability
      case '4':
        Serial.println(F("Enter light on probability..."));
        while (Serial.available() == 0)  {}
        inputFloat = Serial.parseFloat();
        while (Serial.available()){Serial.read();} // throw away extra bytes
        if (inputFloat>=0 && inputFloat<=1){
          obsLightProbability = inputFloat;
          printMenu();
        }else{
          Serial.println(F("ERROR: please enter a decimal >=0 and <=1!"));
        }
        break;
    }

    inputDetected = false;
    notGettingInput = true;
  }
}




void printInitializations(){
  Serial.println("\nINITIALIZATIONS");
  Serial.println("---------------");
  
  Serial.print("track length (meters): ");
  Serial.println(trackEndPosition);

  Serial.print("deceleration distance (meters): ");  // distance from limit switch at which obstcle starts slowing down
  Serial.print((pow(callibrationSpeed,2)-pow(obsSpeedStop,2)) / (2*obsAcceleration), 3);
  Serial.print("\n");

  Serial.print("smallest interstep motor delay (microseconds): ");
  Serial.println(delays[maxSpeedInd]);
  
  Serial.print("motor speed lookup table size: ");
  Serial.println(maxSpeedInd+1);

  Serial.print("wheel speed buffer size: ");
  Serial.println(dtSz);

  if (maxSpeedInd==(bufferSize-1)){
    Serial.print("WARNING! 'speeds' + 'delays' lookup tables could not fit in buffer of size: ");
    Serial.print(bufferSize);
    Serial.print("\n  To fix, increase 'bufferSize', increase 'obsAcceleration', or reduce velocity range.\n");
  }
}
