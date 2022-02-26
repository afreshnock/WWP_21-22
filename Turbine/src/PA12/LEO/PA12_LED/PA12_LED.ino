#include <PA12.h>
#define ID_NUM 0

PA12 myServo(&Serial1,         2,         1);
//        (&Serial ,enable_pin,  Tx Level)

void setup() {
  myServo.begin(32);  
}

void loop() {    
  myServo.ledOn(ID_NUM,RED);
  delay(1000);  
  myServo.ledOn(ID_NUM,GREEN);  
  delay(1000);
  myServo.ledOn(ID_NUM,BLUE);  
  delay(1000);
}
