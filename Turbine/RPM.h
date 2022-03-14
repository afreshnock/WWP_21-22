#define Sensor 2

volatile unsigned long Htime1, Htime2;
int Raw_RPM, Avg_RPM;
unsigned long Timer;
int SampleInterval = 100;
int outputRPM = 0;
unsigned long TempStore = 0;
bool FirstRead = true;
unsigned long Saved_RPM [10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned long RPMSum = 0;
int i = 0;
int F = 0;

void RPMRead() {
  TempStore = micros();
  if (digitalRead(Sensor) == HIGH) {
    if (FirstRead) {
      Htime1 = TempStore; //capture first rising edge time
      FirstRead = false;
    }
    else {
      Htime2 = TempStore; //capture second rising edge time
      FirstRead = true;
      //noInterrupts();
      detachInterrupt(digitalPinToInterrupt(Sensor));
      Raw_RPM = 15000000 / (Htime2 - Htime1); //calculate rpm from period
      if (Raw_RPM < 10000) {
        if (abs(Raw_RPM - outputRPM) < (SampleInterval * 3)) { //compare new rpm to previous
          if ( i < 9 ) { //cycle through array
            i++;
          }
          else {
            i = 0;
          }
          outputRPM = Raw_RPM; //set comparative value
          RPMSum = RPMSum - Saved_RPM [i]; //subtract oldest value from sum
          Saved_RPM [i] = Raw_RPM; //insert new value into array
          RPMSum = RPMSum + Raw_RPM; //add new value to sum
          Avg_RPM = (RPMSum / 10); 
          F = 0; //reset number of fails
        }
        else {
          F ++; //increment failed compared value
          if ( F > 10 ) {
            outputRPM = Raw_RPM; //reset the comparative value
          }
        }
      }
    }
  }
}
void setup_RPM() {
  pinMode(Sensor, INPUT);
  Timer = millis();
  attachInterrupt(digitalPinToInterrupt(Sensor), RPMRead, RISING); //run ISR on rising edge
  //Interrupts();
}

void refresh_RPM() {
  if (millis() - Timer >= SampleInterval) {
    attachInterrupt(digitalPinToInterrupt(Sensor), RPMRead, RISING); //run ISR on rising edge
    Timer = millis();
  }
}
