#include <PA12.h>
#define ID_NUM 0

PA12 myServo(&Serial1,         2,       1);
//        (&Serial ,enable_pin,  Tx Level)

int LimitVolt;
int Display = 1;

void setup() {
  Serial.begin(9600);    // Monitoring PORT
  myServo.begin(32);  
  Serial.print("*Limit Lowest Volt : ");
  Serial.println(myServo.limitVolt(ID_NUM,Lowest));
  while (! Serial);
}

void loop() {     
  if(Display == 1){
    Serial.print("*Limit Lowest Volt : ");
    Serial.println(myServo.limitVolt(ID_NUM,Lowest));
    Display = 0;
  }
  if(Serial.available()) {
    LimitVolt = Serial.parseInt();
    myServo.limitVolt(ID_NUM,Lowest,LimitVolt);
    Display = 1;
  }  
}
