#include <PA12.h>
#define ID_NUM 0

PA12 myServo(&Serial1,         2,  1);
//        (&Serial ,enable_pin,  Tx Level)

int PunchInitial;
int Punch;
int Display=1;

void setup() {
  Serial.begin(9600);    
  myServo.begin(32);  
  while (! Serial);  
}

void loop() {  
  if(Display==1){
	  PunchInitial = myServo.PunchInitial(ID_NUM);  
	  Serial.print("Punch Initial Value: ");
	  Serial.println(PunchInitial);  
	  Punch = myServo.Punch(ID_NUM);
	  Serial.print("Punch : ");
	  Serial.println(Punch);  	  
	  Display =0;
  }
  if(Serial.available()) {  
    PunchInitial = Serial.parseInt(); 
    myServo.PunchInitial(ID_NUM,PunchInitial); 
    Display=1;
  }
}

