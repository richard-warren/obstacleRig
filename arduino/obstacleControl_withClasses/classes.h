

class MovingObject{
  
  public:
    
    static const int maxBufSz = 1000;  // this should actually be (wheelSpeedDistance / mPerTic)
    
    // object settings
    double mPerTic;
    double umPerTic;
    int bufSz;
    
    // tic interval buffer
    volatile long currentMicros = micros();
    volatile long lastMicros = micros();
    volatile int dts[maxBufSz];  // !!! need smarter initialization
    volatile int dtInd = 0;
    long dtSum = 0;  // !!! need smarter initialization

    // constructor
    MovingObject(double mpt, double speedDistance){
      mPerTic = mpt;
      umPerTic = mPerTic * pow(10,6);
      bufSz = min(int(speedDistance/mPerTic), maxBufSz);
    }

    // set speed
    void setObjectSpeed(double s){
      dtSum = 0;
      s = max(s, .005);  // odd behavior emerges at very slow speeds, which have very long dts
      for (int i=0; i<bufSz; i++){
        dts[i] = round(mPerTic / s * pow(10,6));
        dtSum += dts[i];
      }
    }
    
    // add tic interval to buffer
    void addInterval(){
      currentMicros = micros();
      dts[dtInd] = currentMicros-lastMicros;
      lastMicros = currentMicros;
      dtSum += dts[dtInd];  // add the newest dt
      dtInd++;
      if (dtInd==bufSz){dtInd = 0;}
      dtSum -= dts[dtInd];  // remove the oldest dt
    }

    // get speed
    float getSpeed(){
      return (bufSz*mPerTic) / (dtSum*pow(10,-6));  // (m/s)
    }

        // print all dts for debugging purposes
    void printDts(){
      for (int i=1; i<(bufSz+1); i++){
        Serial.print(dts[i]); Serial.print(" ");
      }
      Serial.println();
    }    
};
