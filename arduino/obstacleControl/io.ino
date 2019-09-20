


void printMenu(){

  // print settings
  Serial.println("CURRENT SETTINGS:");

  // condition
  Serial.print("(o)bstacle status: ");
  if (isObsOn){
    Serial.println("ON");
  }else{
    Serial.println("OFF");
  }
  
  // water distance
  Serial.print("water (d)istance: ");
  Serial.println(waterDistance);

  // water duration
  Serial.print("water (t)ime: ");
  Serial.println(waterDuration);
  
  // light on probability
  Serial.print("(l)ight on probability: ");
  Serial.println(obsLightProbability);
  Serial.println("");
  
}




void getUserInput(){

  // check for user input
  if (Serial.available()){

    inputChar = Serial.read();
    Serial.readBytesUntil('\n', inputBuffer, 100);  // throw away all but the first byte, stopping at the newline character

    
    switch (inputChar){

      // turn obstacle on/off
      case 'o':
        isObsOn = !isObsOn;
        digitalWrite(obsOutPin, isObsOn);

        // reset wheel ticks
        noInterrupts();
        wheelTics = 0;
        interrupts();

        printMenu();
        break;
        
      
      // set water distance
      case 'd':
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
      case 't':
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
      case 'l':
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
  }
}
