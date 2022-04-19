#include <PA12.h>

PA12 myServo(&Serial2, 8, 1); 
//               (&Serial ,enable_pin,  Tx Level)

int ID_Sel =0;

void setup() {   
  Serial.begin(9600);  
  myServo.begin(32);  
  myServo.movingSpeed(ID_Sel,750);

  while (!Serial);  

  scan_Bus();
  Serial.print("Input ID : ");  
}

void loop() {    
  myServo.ledOn(1,RED);
  myServo.ledOn(2,GREEN);
  myServo.ledOn(3,BLUE);
  
  if(Serial.available()) {  
    ID_Sel = Serial.parseInt();
    if(Serial.read() == ':')
    {
      myServo.ServoID(ID_Sel, Serial.parseInt());
      scan_Bus();
    }
  }
}

void scan_Bus()
{
  Serial.println("--Servo ID's Present--");
  for(int i = 0; i < 255; i++)
  {
    if(myServo.ServoID(i) != 0xff)
    {
      Serial.print("ID");
      Serial.print(i);
    }
  }
}
