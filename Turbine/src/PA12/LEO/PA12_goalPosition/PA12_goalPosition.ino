#include <PA12.h>
#define ID_NUM 0

PA12 myServo(&Serial1,         2,         1);
//        (&Serial ,enable_pin,  Tx Level)

void setup() {
  // Initialize the PA12 bus:
  // Baudrate -> 128: 9600, 32: 57600, 16: 115200 
  myServo.begin(32);  
}

void loop() {
  myServo.goalPosition(ID_NUM, 0); 
  delay(1000);
  myServo.goalPosition(ID_NUM, 4095);
  delay(1000);
}



