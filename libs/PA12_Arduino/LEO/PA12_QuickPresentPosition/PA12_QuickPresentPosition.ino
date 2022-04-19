#include <PA12.h>
#define ID_R 0
#define ID_L 0
PA12 myServo(&Serial2, 8, 1); 
//               (&Serial ,enable_pin,  Tx Level)

int Position;
int cPosition;
int Display =1;

void setup() {
  Serial.begin(9600);    
  myServo.begin(32);  
  myServo.movingSpeed(ID_R,750);
  myServo.movingSpeed(ID_L,750);
  while (! Serial);  

}
void loop() {  
  if(Display == 1){
    Serial.print("*New Position[0~4095] : ");
    Display = 0;
  }
  if(Serial.available())  {
    Position = Serial.parseInt(); 
    Serial.println(Position);    

    myServo.goalPosition(ID_R,Position);
    myServo.goalPosition(ID_L,4095 - Position);

    while(1) {
      cPosition = myServo.quick_presentPosition(ID_R);
      Serial.print("  - Position_R : ");
      Serial.println(cPosition);
      if(abs(cPosition-Position)<5)
        break;
    }   
  
    cPosition = myServo.presentPosition(ID_R);
    Serial.print("  - final Position : ");
    Serial.println(cPosition);
    Display = 1;
  }  
}
