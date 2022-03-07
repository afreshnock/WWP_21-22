#include <PA12.h>
#define ID_NUM 0

PA12 myServo(&Serial1,         2,         1);
//        (&Serial ,enable_pin,  Tx Level)

int Margin, Position, Display=1;

void setup() {
  Serial.begin(9600);  
  myServo.begin(32);    
  while (! Serial);
}
void loop() {  
  if(Display == 1) {      
    myServo.goalPosition(ID_NUM,0);
    delay(1000);  
    Position = myServo.presentPosition(ID_NUM);
    Serial.print("  - Short Position : ");
    Serial.println(Position);
   
    myServo.goalPosition(ID_NUM,4095);
    delay(1000);  
    Position = myServo.presentPosition(ID_NUM);
    Serial.print("  - Long Position : ");
    Serial.println(Position);   
    Serial.println(" ");    
    Serial.print("* New Compliance Margin : ");
    Display = 0;
  }  
  if(Serial.available()) {
     Margin = Serial.parseInt();
    myServo.complianceMargin(ID_NUM,Short,Margin);
    myServo.complianceMargin(ID_NUM,Long,Margin);       
    Serial.println(Margin);
    Display = 1;
  }
}
