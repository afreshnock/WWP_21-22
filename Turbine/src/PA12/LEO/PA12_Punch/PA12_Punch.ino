#include <PA12.h>
#define ID_NUM 0

PA12 myServo(&Serial1,         2,  1);
//               (&Serial ,enable_pin,  Tx Level)

int punch_data;
int Display = 0;

void setup() {
  Serial.begin(9600);    
  myServo.begin(32);    
  while (! Serial);  
  Serial.print("Present Punch : ");
  Serial.println(myServo.Punch(ID_NUM)); 
  Serial.print("Input Punch : ");
}

void loop() {    
  if(Display ==1 ){                  
    myServo.goalPosition(ID_NUM,0); 
    delay(1000);
    myServo.goalPosition(ID_NUM,4095); 
    delay(1000);
    Serial.print("Input Punch : ");
    Display = 0;
  }  
  if(Serial.available()) {
    punch_data = Serial.parseInt();
    myServo.Punch(ID_NUM,punch_data);
    Serial.println(myServo.Punch(ID_NUM)); 
    delay(500);
    Display = 1;
  }
}

