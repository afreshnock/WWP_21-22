#include <PA12.h>
#define ID_NUM 0

PA12 myServo(&Serial1,         2,         1);
//        (&Serial ,enable_pin,  Tx Level)

int ForceLimit;

void setup() {
 Serial.begin(9600);
  myServo.begin(32);  
  while (! Serial);
  Serial.print("* Max Force[0~1023] : ");
}

void loop() {     
  myServo.goalPosition(ID_NUM,0);
  delay(1000);  
  myServo.goalPosition(ID_NUM,4015);
  delay(1000); 
  
  if(Serial.available()) {
    ForceLimit = Serial.parseInt();
    myServo.forceLimit(ID_NUM,ForceLimit); 
    Serial.println(ForceLimit);
    Serial.print("* Max Force[0~1023] : ");  
  }
}
