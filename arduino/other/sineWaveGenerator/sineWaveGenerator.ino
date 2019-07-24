
#include "Adafruit_MCP4725.h"

// settings
const int obstaclePin = 2; // must be either 2 or 3 (these are interrupt pins on arduino)
const float hz = 40; // frequency of sine wave
const int maxLightTime = 3000; // ms
const int rampDownTime = 100; // ms
const int rampUpTime = 100; // ms
const int testStimDuration = 1000; // ms
const bool stopWhenPinLow = false; // should you discontinue light when the triggering pin goes low?
volatile float lightProbability = 0;
volatile float lightPower = .01;


// initializations
const uint8_t sinLookup[] = {0,0,0,0,0,0,0,0,1,1,1,1,1,2,2,2,3,3,3,4,4,4,5,5,6,6,7,7,8,8,9,10,10,11,12,12,13,14,14,15,16,17,17,18,19,20,21,22,23,24,24,25,26,27,28,29,30,31,33,34,35,36,37,38,39,40,42,43,44,45,46,48,49,50,51,53,54,55,57,58,59,61,62,64,65,66,68,69,71,72,73,75,76,78,79,81,82,84,85,87,88,90,91,93,95,96,98,99,101,102,104,106,107,109,110,112,113,115,117,118,120,121,123,125,126,128,130,131,133,134,136,138,139,141,142,144,146,147,149,150,152,153,155,157,158,160,161,163,164,166,167,169,170,172,173,175,176,178,179,181,182,184,185,187,188,189,191,192,194,195,196,198,199,200,202,203,204,205,207,208,209,210,212,213,214,215,216,218,219,220,221,222,223,224,225,226,227,228,229,230,231,232,233,234,235,236,236,237,238,239,240,240,241,242,242,243,244,244,245,246,246,247,247,248,248,249,249,250,250,251,251,252,252,252,253,253,253,253,254,254,254,254,254,255,255,255,255,255,255,255,255,255,255,255,255,255,255,254,254,254,254,254,253,253,253,253,252,252,252,251,251,250,250,249,249,248,248,247,247,246,246,245,244,244,243,242,242,241,240,240,239,238,237,236,236,235,234,233,232,231,230,229,228,227,226,225,224,223,222,221,220,219,218,216,215,214,213,212,210,209,208,207,205,204,203,202,200,199,198,196,195,194,192,191,189,188,187,185,184,182,181,179,178,176,175,173,172,170,169,167,166,164,163,161,160,158,157,155,153,152,150,149,147,146,144,142,141,139,138,136,134,133,131,130,128,126,125,123,121,120,118,117,115,113,112,110,109,107,106,104,102,101,99,98,96,95,93,91,90,88,87,85,84,82,81,79,78,76,75,73,72,71,69,68,66,65,64,62,61,59,58,57,55,54,53,51,50,49,48,46,45,44,43,42,40,39,38,37,36,35,34,33,31,30,29,28,27,26,25,24,24,23,22,21,20,19,18,17,17,16,15,14,14,13,12,12,11,10,10,9,8,8,7,7,6,6,5,5,4,4,4,3,3,3,2,2,2,1,1,1,1,1,0,0,0,0,0,0,0,0};
const int fs = 1000; // sampling frequency (don't change this dummy!)
const int sinSmps = sizeof(sinLookup);
const float deltaIndex = sinSmps*hz/fs;
volatile float sinIndex = 0;
volatile float bitConversion = 4095 / 255;
volatile bool updated = false;
volatile uint16_t newValue = 0;
volatile long lightTimer = maxLightTime + rampDownTime;
volatile bool isLightOn = true;
volatile int currentStimDuration = maxLightTime;
volatile int userInput;
volatile bool constantLightOn = false;
volatile bool isStepping = false;
Adafruit_MCP4725 dac;




void setup() {

  pinMode(obstaclePin, INPUT);
  
  // INITIALIZE TIMER INTERRUPTS
  cli();//disable interrupts
  TCCR0A = 0;// set entire TCCR0A register to 0
  TCCR0B = 0;// same for TCCR0B
  TCNT0  = 0;//initialize counter value to 0
  OCR0A = 249; // set compare match register for 2khz increments
  TCCR0A |= (1 << WGM01); // turn on CTC mode
  TCCR0B |= (1 << CS01) | (1 << CS00); // Set CS01 and CS00 bits for 64 prescaler
  TIMSK0 |= (1 << OCIE0A); // enable timer compare interrupt
  sei();//enable interrupts

  TWBR = 12; // speed up i2c communication
  dac.begin(0x62); // begin communication with DAC
  dac.setVoltage(0, false);
  randomSeed(analogRead(0)); // initialiez random seed
  attachInterrupt(digitalPinToInterrupt(obstaclePin), lightOnOff, CHANGE);
  Serial.begin(115200);
  showMenu();
  
}



// t0 interrupt
ISR(TIMER0_COMPA_vect){

  // update sin table index
  if (isLightOn){
    
    newValue = constrain(round(sinLookup[min(round(sinIndex), sinSmps-1)] * bitConversion * lightPower), 0, 4095);
    
    // ramp up
    newValue = newValue * constrain(float(lightTimer)/rampUpTime, 0, 1);
    
    // ramp down
    newValue = newValue * float(constrain(currentStimDuration+rampDownTime - lightTimer,0,rampDownTime)) / rampDownTime; // ramp that shit down!
    updated = true;

    lightTimer++;
    sinIndex += deltaIndex;
    if (sinIndex>sinSmps){sinIndex -= sinSmps;}
    if (lightTimer>(currentStimDuration+rampDownTime)){ // if the ramp is over
      isLightOn = false;
    } 
  }

  else if (isStepping){
    lightTimer++;
    if (lightTimer>testStimDuration){
      newValue = 0;
      updated = true;
      isStepping = false;
    }
  }
}



void loop(){
  
  // update LED output
  if (updated){
    dac.setVoltage(newValue, false);
    updated = false;
  }

  // check for user input
  if (Serial.available()){
    
    userInput = Serial.read() - 48; // parseInt freezes for some reason
    
    switch (userInput){

      // test light output
      case 1:
        currentStimDuration = testStimDuration;
        deliverLight(testStimDuration);
        break;

      // set light power
      case 2:
        Serial.println("enter light power (0-1)...");
        while (Serial.available() == 0)  {}
        lightPower = Serial.parseFloat();
        showMenu();
        break;

      // set light on probability
      case 3:
        Serial.println("enter light probability (0-1)...");
        while (Serial.available() == 0)  {}
        lightProbability = Serial.parseFloat();
        showMenu();
        break;

      // turn constant light on/off
      case 4:
        if (constantLightOn){
          newValue = 0;
          constantLightOn = false;
        }else{
          newValue = 4095.0 * lightPower;
          constantLightOn = true;
        }
        updated = true;
        showMenu();
        break;

      // deliver step pulse of light
      case 5:
        newValue = 4095 * lightPower;
        updated = true;
        isStepping = true;
        lightTimer = 0;
        showMenu();
        break;
    }
    
  }
}



// begins light delivery
void deliverLight(int duration){
  lightTimer = 0;
  sinIndex = 0;
  isLightOn = true;
  currentStimDuration = duration;
}


// turns on light when pin goes high, and turns off light when pin goes low
void lightOnOff(){
  if ((digitalRead(obstaclePin) && random(0,100)<(lightProbability*100.0))){
    deliverLight(maxLightTime);
    
  }else if (lightTimer<currentStimDuration && stopWhenPinLow){ // only do this when stopWhenPinLow
    lightTimer = currentStimDuration;
  }
}


// shows serial menue
void showMenu(){
  Serial.println("\n-------------------------");
  Serial.println("1: test light stimulation");
  Serial.print("2: set fraction light power: ");
  Serial.println(lightPower, 3); // second argument is number of decimals
  Serial.print("3: set light probability: ");
  Serial.println(lightProbability);
  if (constantLightOn){
    Serial.println("4: turn constant light OFF");
  }else{
    Serial.println("4: turn constant light ON");
  }
  Serial.println("5: step pulse");
}









