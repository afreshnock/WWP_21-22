#include <PA12.h>
#define ID_NUM 0

PA12 m_zap(&Serial1,         2,         1);
//        (&Serial ,enable_pin,  Tx Level)

int Speed, Display=1;

void setup() {
  Serial.begin(9600); 
  myServo.begin(32);  
  while (! Serial);
}

void loop() {  
  if(Display ==1) {                  
    myServo.goalPosition(ID_NUM,0);
    delay(500);
    while(myServo.Moving(ID_NUM));  
    
    myServo.goalPosition(ID_NUM,4095);
    delay(500);
    while(myServo.Moving(ID_NUM));
    Serial.print("New Speed[0~1023] : ");
    Display=0;
  }  
  if(Serial.available()) {
    Speed = Serial.parseInt();
    myServo.movingSpeed(ID_NUM,Speed);
    Serial.println(myServo.movingSpeed(ID_NUM));
    delay(500);
    Display=1;
  }
}
