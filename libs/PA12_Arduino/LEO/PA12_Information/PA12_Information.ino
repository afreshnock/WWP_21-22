#include <PA12.h>
#define ID_NUM 0

PA12 myServo(&Serial1,         2,         1);
//        (&Serial ,enable_pin,  Tx Level)

void setup() {
  Serial.begin(9600);    // Monitoring PORT
  myServo.begin(32);  
  while (! Serial);
}
void loop() {  
  if(Serial.available())  {    
    char ch = Serial.read();
    if(ch=='d')    {
      Serial.print("Model Number        : ");  Serial.println(myServo.getModelNumber(ID_NUM));
      Serial.print("Firmware Version    : ");  Serial.println(myServo.Version(ID_NUM));      
      Serial.print("CAL Short Stroke    : ");  Serial.println(myServo.CalStroke(ID_NUM,Short));
      Serial.print("CAL Long Stroke     : ");  Serial.println(myServo.CalStroke(ID_NUM,Long));
      Serial.print("CAL Center Stroke   : ");  Serial.println(myServo.CalStroke(ID_NUM,Center));
      Serial.print("Present Volt        : ");  Serial.println(myServo.presentVolt(ID_NUM)/10);
      Serial.print("Present Temperature : ");  Serial.println(myServo.presentTemperature(ID_NUM));
    }
  }
}
