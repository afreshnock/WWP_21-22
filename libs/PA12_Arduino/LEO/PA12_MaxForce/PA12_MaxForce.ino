#include <PA12.h>
#define ID_NUM 0

PA12 myServo(&Serial1,         2,         1);
//        (&Serial ,enable_pin,  Tx Level)

int MaxForce;
int ForceLimit;
int Display=1;

void setup() {
  Serial.begin(9600);   
  myServo.begin(32);  
  while (! Serial);  
}

void loop() {  
  if(Display==1){
	  MaxForce = myServo.maxForce(ID_NUM);  
	  Serial.print("Max Force   : ");
	  Serial.println(MaxForce);  
	  ForceLimit = myServo.forceLimit(ID_NUM);
	  Serial.print("Force Limit : ");
	  Serial.println(ForceLimit);  
	  Serial.println("-------------------------");
	  Display =0;
  }
  if(Serial.available()) {
    MaxForce = Serial.parseInt();    
    myServo.maxForce(ID_NUM,MaxForce); 
    Display =1;
   }
}

