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
  myServo.movingSpeed(ID_NUM,50);
}
void loop() {  
  if(Display == 1){
    Serial.print("*New Position[0~4095] : ");
    Display = 0;
  }
  if(Serial.available())  {
    Position = Serial.parseInt(); 
    Serial.println(Position);    
    delay(200);

    myServo.goalPosition(ID_NUM,Position);
    delay(200);
    while(myServo.Moving(ID_NUM)) {
      cPosition = myServo.presentPosition(ID_NUM);
      Serial.print("  - Position : ");
      Serial.println(cPosition);
    }   
    delay(200);
    cPosition = myServo.presentPosition(ID_NUM);
    Serial.print("  - final Position : ");
    Serial.println(cPosition);
    Display = 1;
  }  
}