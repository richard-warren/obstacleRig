
void getUserInput(){
  // check for user input
  
  if (Serial.available()){
    
    userInput = Serial.read() - 48;
    
    switch (userInput){

      // deliver stimulus
      case 1:
        signalPowerTemp = signalPower;
        startSignal();
        break;

      // set light power
      case 2:
        Serial.println("enter signal power (0-1)...");
        while (Serial.available() == 0)  {}
        signalPower = Serial.parseFloat();
        showMenu();
        break;

      // set light on probability
      case 3:
        Serial.println("enter signal probability (0-1)...");
        while (Serial.available() == 0)  {}
        signalProbability = Serial.parseFloat();
        showMenu();
        break;

      // set signal duration
      case 4:
        Serial.println("enter signal duration (ms)...");
        while (Serial.available() == 0)  {}
        signalDuration = Serial.parseInt();
        showMenu();
        break;

      // set stimulus type
      case 5:
        Serial.println("enter stimulus type (0: sin, 1: step, 2: pulse)...");
        while (Serial.available() == 0)  {}
        stimType = Serial.parseInt();
        showMenu();
        break;

      // set stimulus frequency
      case 6:
        Serial.println("enter stimulus frequency (hz)...");
        while (Serial.available() == 0)  {}
        hz = Serial.parseInt();
        setFrequency();
        showMenu();
        break;

      // set pulse duration
      case 7:
        Serial.println("enter pulse duration (ms)...");
        while (Serial.available() == 0)  {}
        pulseDuration = Serial.parseInt();
        showMenu();
        break;

      // set ramp up time
      case 8:
        Serial.println("enter ramp up time (ms)...");
        while (Serial.available() == 0)  {}
        rampUpTime = Serial.parseInt();
        showMenu();
        break;

      // set ramp down time
      case 9:
        Serial.println("enter ramp down time (ms)...");
        while (Serial.available() == 0)  {}
        rampDownTime = Serial.parseInt();
        showMenu();
        break;
    }
  }
}



void showMenu(){
  // shows serial menu
  
  Serial.println("\n-------------------------");
  Serial.println("1: deliver stimulus");
  Serial.print("2: set stimulus power: ");
  Serial.println(signalPower, 3); // second argument is number of decimals
  Serial.print("3: set signal probability: ");
  Serial.println(signalProbability, 3);
  Serial.print("4: set signal duration: ");
  Serial.println(signalDuration);
  Serial.print("5: set stimulus type: ");
  Serial.println(stimTypes[stimType]);
  Serial.print("6: set stimulus frequency: ");
  Serial.println(hz);
  Serial.print("7: set pulse duration: ");
  Serial.println(pulseDuration);
  Serial.print("8: set ramp up time: ");
  Serial.println(rampUpTime);
  Serial.print("9: set ramp down time: ");
  Serial.println(rampDownTime);
}
