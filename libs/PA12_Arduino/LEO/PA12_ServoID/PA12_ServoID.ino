#include <PA12.h>

PA12 myServo(&Serial1,         2,  1);
//               (&Serial ,enable_pin,  Tx Level)

int ID_Sel =0;

void setup() {   
  Serial.begin(9600);
  myServo.begin(32);  
  while (! Serial);  
  Serial.print("Input ID : ");  
}

void loop() {    
  myServo.ledOn(1,RED);
  myServo.ledOn(2,GREEN);
  myServo.ledOn(3,BLUE);
  
  if(Serial.available()) {  
    ID_Sel = Serial.parseInt();
    Serial.println(ID_Sel);
    Serial.print("Input_ID [0~3] : ");
    myServo.ServoID(0,ID_Sel);
    myServo.ServoID(1,ID_Sel);
    myServo.ServoID(2,ID_Sel);
    myServo.ServoID(3,ID_Sel);
  }
}
