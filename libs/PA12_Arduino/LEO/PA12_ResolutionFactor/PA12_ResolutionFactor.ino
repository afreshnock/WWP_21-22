#include <PA12.h>
#define ID_NUM 0

PA12 myServo(&Serial1,         2,  1);
//        (&Serial ,enable_pin,  Tx Level)

int Resolution;
int Display=1;

void setup() {
  Serial.begin(9600);
  myServo.begin(32);  
  while (! Serial);   
}

void loop() {  
  if(Display==1){                
    myServo.goalPosition(ID_NUM,512);
    delay(1000);  
    myServo.goalPosition(ID_NUM,0);
    delay(1000);  
    //  Factor 1: Resolution 4096
    //  Factor 2: Resolution 2048
    //  Factor 3: Resolution 1024
    //  Factor 4: Resolution 512
    Serial.print("Input Resolution Factor(1~4) : ");
    Display=0;
  } 
  if(Serial.available()) {
    Resolution = Serial.parseInt();
    myServo.resolutionFactor(ID_NUM,Resolution);
    Serial.println(Resolution);    
    delay(500);                
    Display=1;
  }
}
