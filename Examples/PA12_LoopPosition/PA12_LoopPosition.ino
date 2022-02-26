#include <PA12.h>
#define ID_NUM 0

PA12 myServo(&Serial2,         8,  1);
//               (&Serial ,enable_pin,  Tx Level)

int Position;
int Position1;
int cPosition;
int cPosition1;
bool Retract;

void setup() {
  Serial.begin(9600); 
  delay(2000);   
  myServo.begin(32);  
  myServo.movingSpeed(ID_NUM,50);
  Retract = false;
}
void loop() {
  delay(200);
  if(!myServo.Moving(ID_NUM)){
    if(myServo.alarmShutdown(ID_NUM) == 1){
      myServo.alarmShutdown(ID_NUM, 0);
    }
    if(Retract)
    {
      Position = 0;
      Position1 = 4000;
    }
    else
    {
      Position = 4000;
      Position1 = 0;
    }
    Retract = !Retract;
    myServo.goalPosition(ID_NUM,Position);
    delay(200);
    while(myServo.Moving(ID_NUM)) {
      cPosition = myServo.presentPosition(ID_NUM);
      Serial.print("  - Position 0: ");
      Serial.println(cPosition);
      
    }   
    delay(200);
    cPosition = myServo.presentPosition(ID_NUM);
    Serial.print("  - final Position 0: ");
    Serial.println(cPosition);
  }
}
