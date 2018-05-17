const int obsHeightPin = 10;
const float travel = 15.0; // mm

volatile float userInput = 0;
volatile int obsHeight = 0;


void setup() {
  pinMode(obsHeightPin, OUTPUT);
  analogWrite(obsHeightPin, userInput);
  Serial.begin(9600);
}

void loop() {

  if (Serial.available()){
    userInput = Serial.parseFloat();
    obsHeight = round(userInput * (255.0 / travel));
    analogWrite(obsHeightPin, obsHeight);
    
    Serial.print("height set to ");
    Serial.print(userInput);
    Serial.print("mm, analogWrite ");
    Serial.println(obsHeight);
  }

}
