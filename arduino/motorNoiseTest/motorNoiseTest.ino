// pin assignments
const int stepPin = 4;
const int stepDirPin = 5;
const int motorOnPin = 8; // turns on stepper motor driver

// user settings
const int stepNumbers[] = {1, 2, 4, 8, 16, 32, 64, 128, 256}; // the arduino will take successive steps in these amounts
const int interStepInterval = 1000; // ms
const int microStepping = 2;
const int stepperSpeed = 400 * microStepping;

// other initializations
const int pulseDuration = (pow(10, 6) / stepperSpeed) / 2;
volatile bool stepDir = true;
volatile bool isRunning = false;

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(stepDirPin, OUTPUT);
  pinMode(motorOnPin, OUTPUT);

  digitalWrite(stepPin, LOW);
  digitalWrite(stepDirPin, stepDir);
  digitalWrite(motorOnPin, LOW);

  Serial.begin(115200);
  Serial.println(pulseDuration);
}

void loop() {

  // get user input
  if (Serial.available()){
    Serial.read(); // clear buffer
    isRunning = !isRunning;
    digitalWrite(motorOnPin, isRunning);
  }
  
  if (isRunning){
    
    // iteraute through all stepNumbers
    for (int i=0; i < (sizeof(stepNumbers) / sizeof(int)); i++){
  
      // check for user input
      if (Serial.available()){ break; }
      
      // display current step number
      Serial.println(stepNumbers[i]);
      
      for (int j=0; j<stepNumbers[i]; j++){
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(pulseDuration);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(pulseDuration);
      }
      
      delay(interStepInterval);
    }
  }


  // reverse motor direction
  stepDir = !stepDir;
  digitalWrite(stepDirPin, stepDir);
}
