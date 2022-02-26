#include <PA12.h>
#define ID_NUM 0


PA12 myServo(&Serial1,         2,       1);
//        (&Serial ,enable_pin,Tx Level)

int alarm;
int Display = 1;

void setup() {
  Serial.begin(9600);    
  myServo.begin(32);  
  while (! Serial);  
  Serial.print("*Alarm LED State : ");
  Serial.println(myServo.alarmLed(ID_NUM));
  Serial.print("*Alarm LED State : ");
}

void loop() {     
  if(Serial.available()) {
    alarm = Serial.parseInt();
    myServo.alarmLed(ID_NUM,alarm);    
    Serial.println(myServo.alarmLed(ID_NUM));    
    Serial.print("*Alarm LED State : ");
  }  
}