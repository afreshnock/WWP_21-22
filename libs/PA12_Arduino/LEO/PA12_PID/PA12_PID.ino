#include <PA12.h>
#define ID_NUM 0

PA12 myServo(&Serial1,         2,  1);
//        (&Serial ,enable_pin,  Tx Level)

int pData,iData,dData;
int Display=1;

void setup() {
  Serial.begin(9600);
  myServo.begin(32); 
  while (! Serial);
}

void loop() {  
  if(Display==1){
    Serial.print("* PID Gain = "); 
    pData = myServo.pidGain(ID_NUM,pGain);
    iData = myServo.pidGain(ID_NUM,iGain);
    dData = myServo.pidGain(ID_NUM,dGain);
    Serial.print(pData);  Serial.print(", ");
    Serial.print(iData);  Serial.print(", ");
    Serial.println(dData);
    
    myServo.goalPosition(ID_NUM,4095); 
    delay(1000);
    myServo.goalPosition(ID_NUM,0); 
    delay(1000);  
    Display = 0;
  }
  if(Serial.available()) {    
    pData = Serial.parseInt();       
    iData = Serial.parseInt();       
    dData = Serial.parseInt();    
    myServo.pidGain(ID_NUM,pGain,pData);
    myServo.pidGain(ID_NUM,iGain,iData);
    myServo.pidGain(ID_NUM,dGain,dData);
    Display = 1;
  }
}