#include <PA12.h>
#define ID_NUM 0

PA12 myServo(&Serial1,         2,      1);
//        (&Serial ,enable_pin,  Tx Level)

int Temperature;

void setup() {
  Serial.begin(9600);    
  myServo.begin(32);      
  while (! Serial);
  Serial.print("*LimitTemperature : ");
  Serial.println(myServo.maxTemperature(ID_NUM));
}

void loop() {     
  if(Serial.available()) {
    Temperature = Serial.parseInt();
    myServo.maxTemperature(ID_NUM,Temperature);
    Serial.print("*LimitTemperature : ");
    Serial.println(myServo.maxTemperature(ID_NUM));
    Serial.print("*presentTemperature : ");
    Serial.println(myServo.presentTemperature(ID_NUM));    
  }  
}
