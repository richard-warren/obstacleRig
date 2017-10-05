#include <CapacitiveSensor.h>
#include <RunningMedian.h>

// note: this touch sensor using a running median calculation to measure and subtract baseline
// the capacitive touch library disables interrupts, so I can't use timer interrupts to properly
// printing the values to the serial port will approximately double the time of the loop, so length of baseline measurement will double while printing to serial port unfortunately

// pin assignments
const int touchSendPin = 3;
const int touchReceivePin = 4;
const int isTouchingPin = 13;

// user settings
const int baselineLng = 30; // s, the amount of time to compute moving median for baseline subtraction
const int sensorSmps = 2;
const int touchThresh = 50;
const int maxMeasurementTime = 100; // ms, capacitive touch reading times out after 100ms
const int medianSmps = 100;
const int maxTouchMeasurement = 32767; // constain to fit in an int

// other initializations
CapacitiveSensor touchSensor = CapacitiveSensor(touchSendPin, touchReceivePin);
RunningMedian runningMed = RunningMedian(medianSmps);
volatile int touchMeasurement = 0;
const int runningMedTics = (float(baselineLng)/medianSmps * 1000) / sensorSmps; // number of loop cycles between successive sampling of touch signal for running median computation... assumes 1ms loop time
volatile int runningMedCounter = 0;
volatile long currentBaseline = 0;




void setup(){
  
  // setup pins
  pinMode(isTouchingPin, OUTPUT);
  digitalWrite(isTouchingPin, LOW);

  // setup touch sensor
  touchSensor.set_CS_Timeout_Millis(maxMeasurementTime / sensorSmps);

  // begin serial communication
  Serial.begin(115200);
}



void loop(){

  // timing notes: at baseline, executing this loop with serial communication takes ~1ms, but longer when something is actually touching the sensor

  // get touch measurement
  touchMeasurement = touchSensor.capacitiveSensorRaw(sensorSmps);

  // increment running median and add samples if proper time interval has passed
  runningMedCounter++;
  if (runningMedCounter == runningMedTics){
    runningMed.add(touchMeasurement);
    currentBaseline = runningMed.getMedian();
    runningMedCounter = 0;
  }

  if (touchMeasurement >= 0){ // capacitiveSensorRaw returns negative value if timeout is reached, will will only happen if capacitance is very high

    // subtract running median and scale
    touchMeasurement = constrain((touchMeasurement - currentBaseline) / sensorSmps, 0, maxTouchMeasurement);
//    touchMeasurement = constrain((touchMeasurement) / sensorSmps, 0, maxTouchMeasurement);
  
    // write isTouching pin HIGH if threshold crossed
    digitalWrite(isTouchingPin, (touchMeasurement > touchThresh));
  }
  else{
    digitalWrite(isTouchingPin, HIGH); // previous touchMeasurement is maintained in this case
  }
     
  // display raw touch sensor values
  Serial.println(touchMeasurement);
  delay(1);
}







