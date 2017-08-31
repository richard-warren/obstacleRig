const int startLimitPin = 9; // signal is LOW when engaged
const int stopLimitPin = 10; // signal is LOW when engaged


void setup() {
  pinMode(startLimitPin, INPUT_PULLUP);
  pinMode(stopLimitPin, INPUT_PULLUP);

  Serial.begin(115200);
}

void loop() {
  Serial.print(" start: ");
  Serial.print(digitalRead(startLimitPin));
  Serial.print(" stop: ");
  Serial.println(digitalRead(stopLimitPin));
  

}
