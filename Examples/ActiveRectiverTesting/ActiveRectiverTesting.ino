int gate1 = A5;
int gate2 = A0;
int gate3 = A3;
int gate4 = A4;
int gate5 = A1;
int gate6 = A2;
int RESET;

bool State1, State2, State3, State4, State5, State6 = false;

unsigned long Timer_250;

void setup() {
  // put your setup code here, to run once:
  
  pinMode(gate1,OUTPUT);
  pinMode(gate2,OUTPUT);
  pinMode(gate3,OUTPUT);
  pinMode(gate4,OUTPUT);
  pinMode(gate5,OUTPUT);
  pinMode(gate6,OUTPUT);
   


  /*
  pinMode(gate1,INPUT);
  pinMode(gate2,INPUT);
  pinMode(gate3,INPUT);
  pinMode(gate4,INPUT);
  pinMode(gate5,INPUT);
  pinMode(gate6,INPUT);
  //pinMode(RESET,OUTPUT);
  */

    /*
  pinMode(gate1,INPUT_PULLUP);
  pinMode(gate2,INPUT_PULLUP);
  pinMode(gate3,INPUT_PULLUP);
  pinMode(gate4,INPUT_PULLUP);
  pinMode(gate5,INPUT_PULLUP);
  pinMode(gate6,INPUT_PULLUP);
  pinMode(RESET,OUTPUT);
  */

  
  Serial.begin(9600);
  Timer_250 = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available() > 0)
  {
    uint8_t cmd = Serial.read();

    switch(cmd)
    {
      case '1':
        State1 = !State1;
        digitalWrite(gate1, State1);

      break;

      case '2':
        State2 = !State2;
        digitalWrite(gate2, State2);

      break;

      case '3':
        State3 = !State3;
        digitalWrite(gate3, State3);

      break;

      case '4':
        State4 = !State4;
        digitalWrite(gate4, State4);

      break;

      case '5':
        State5 = !State5;
        digitalWrite(gate5, State5);

      break;

      case '6':
        State6 = !State6;
        digitalWrite(gate6, State6);

      break;
      /*
      case 'r':
        digitalWrite(RESET, LOW);
        delay(100);
        digitalWrite(RESET, HIGH);
      break;
      */



      default:
        Serial.println("Command not recognized");
      break;
    }
  }
  if(millis() - Timer_250 >= 250){
    Timer_250 = millis();
    if(State1){
    Serial.println((String) gate1 + ": HIGH");
    }
    else{
    Serial.println((String) gate1 + ": LOW");
    }
    if(State2){
    Serial.println((String) gate2 + ": HIGH");
    }
    else{
    Serial.println((String) gate2 + ": LOW");
    }
    if(State3){
    Serial.println((String) gate3 + ": HIGH");
    }
    else{
    Serial.println((String) gate3 + ": LOW");
    }
    if(State4){
    Serial.println((String) gate4 + ": HIGH");
    }
    else{
    Serial.println((String) gate4 + ": LOW");
    }
    if(State5){
    Serial.println((String) gate5 + ": HIGH");
    }
    else{
    Serial.println((String) gate5 + ": LOW");
    }
    if(State6){
    Serial.println((String) gate6 + ": HIGH");
    }
    else{
    Serial.println((String) gate6 + ": LOW");
    }

    Serial.println();
  }
}
