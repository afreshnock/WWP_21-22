#include <PA12.h>
#define ID_NUM 0

PA12 myServo(&Serial1,         2,  1);
//        (&Serial ,enable_pin,  Tx Level)

void setup() { 
  myServo.begin(32);  
}

void loop() {
  myServo.StrokeLimit(ID_NUM,Long,3095);  
  myServo.goalPosition(ID_NUM,4095);      
  delay(1000);
   
  myServo.StrokeLimit(ID_NUM,Short,100);  
  myServo.goalPosition(ID_NUM,0);         
  delay(1000);   
}
