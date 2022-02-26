#include <PA12.h>
#define ID_NUM 0

PA12 myServo(&Serial1,         2,  1);
//               (&Serial ,enable_pin,  Tx Level)

int Position;
int cPosition;
int Display =1;

void setup() {
  Serial.begin(9600);    
  myServo.begin(32);  
  while (! Serial);  
  myServo.movingSpeed(ID_NUM,1023);
}
void loop() {  
  if(Display == 1){
    Serial.print("*New Position[0~4095] : ");
    Display = 0;
  }
  if(Serial.available())  {
    Position = Serial.parseInt(); 
    Serial.println(Position);    

    myServo.goalPosition(ID_NUM,Position);

    while(1) {
      cPosition = myServo.quick_presentPosition(ID_NUM);
      Serial.print("  - Position : ");
      Serial.println(cPosition);
      if(abs(cPosition-Position)<5)
        break;
    }   
  
    cPosition = myServo.presentPosition(ID_NUM);
    Serial.print("  - final Position : ");
    Serial.println(cPosition);
    Display = 1;
  }  
}
