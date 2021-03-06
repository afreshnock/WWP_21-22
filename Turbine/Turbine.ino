#if defined(__IMXRT1062__)
extern "C" uint32_t set_arm_clock(uint32_t frequency);
#endif
#include "src/INA260/Adafruit_INA260.h"
#include "src/PA12/PA12.h"
//Local libraries need to either be directly in the project folder, or in the src folder. 
//src linker only works in Arduino IDE 1.5+ I believe.
#include "RPM.h"

#define ID_NUM 0

PA12 myServo(&Serial5, 8, 1); 
//          (&Serial, enable_pin,  Tx Level)
Adafruit_INA260 ina260 = Adafruit_INA260();

enum Switching_States {S_Wait, Toggled_H, Toggled_L};
Switching_States Switching_State = S_Wait;

enum States {Wait, Normal, Regulate, Safety1, Safety2};
States State = Wait;

//Load Variables
uint16_t L_Power;   //Load Power (mW)
uint16_t L_Voltage; //Load Voltage (mV)

//Turbine Variables
uint16_t T_Power;   //Turbine Power (mW)
uint16_t T_Voltage; //Turbine Voltage (mV)
uint16_t RPM;       //Turbine RPM   (r/min)
uint8_t alpha;      //Active Rectifier phase angle  (degrees)
uint8_t old_alpha;
uint16_t theta = 2000;      //Active Pitch angle   
bool last_E_Switch;
bool E_Switch;      //Bool indicating switch open

//IDK Variables
uint16_t Peak_Power;  //(mW)
uint16_t Peak_RPM;    //(r/min)
const int Safety_SW = 17;

unsigned PCC_Relay_Pulse_Interval = 100;
unsigned long Timer_PCC_Relay;
unsigned long Timer_50;
unsigned long Timer_Slow;

int PCC_Relay_Set_Pin = 15;
int PCC_Relay_Reset_Pin = 14;

bool PCC_Relay = false;
bool last_PCC_Relay = false;
//---------------------------------------------------------------------------------------
void setup()
{
  delay(1000);
  pinMode(13, OUTPUT); //debug pin
  digitalWrite(13, HIGH);
  //UART1 (to turbine)
  pinMode(1, OUTPUT); //TX1
  pinMode(0, INPUT);  //RX1
  #if defined(__IMXRT1062__)
    set_arm_clock(24000000);
    Serial.print("F_CPU_ACTUAL=");
    Serial.println(F_CPU_ACTUAL);
  #endif
  
  
  //UART2 (to linear actuators)
  pinMode(20, OUTPUT); //TX1
  pinMode(21, INPUT);  //RX1

  //I2C (to INA260)
  pinMode(18, OUTPUT); //SDA1
  pinMode(19, OUTPUT); //SCL1

  pinMode(PCC_Relay_Reset_Pin, OUTPUT);
  pinMode(PCC_Relay_Set_Pin, OUTPUT);

  pinMode(Safety_SW, INPUT);
  E_Switch = digitalRead(Safety_SW);
  last_E_Switch = E_Switch;

  //start comms with active rectifier
  Wire.begin();
  Wire.setClock(400000);
  
  //start comms with INA260
  ina260.begin();

  //Turbine-Load UART
  Serial1.begin(9600);

  //Set up coms with PC
  Serial.begin(9600);
  
  //linear actuator
  myServo.begin(32);  
  myServo.movingSpeed(ID_NUM,750);
  
  //initialize RPM
  setup_RPM();

  //set up timers
  Timer_50 = millis();
  Timer_Slow = millis();
}

//---------------------------------------------------------------------------------------
void loop()
{
  refresh_RPM();
  uart_RX();
  if(millis() - Timer_50 >= 50)
  {
    Timer_50 = millis();
    //*********Code that runs all the time independent of the State**********
    RPM = outputRPM;
    read_Sensors();
    update_PCC_Relay();
    //***********************************************************************
    uart_TX();
    if(old_alpha != alpha)
    {
      old_alpha = alpha;
      AR_TX();
      AR_RX();
    }
    myServo.goalPosition(ID_NUM, theta);
  }
  
  if(millis() - Timer_Slow >= 250)
  {
    Timer_Slow = millis();
    pc_coms();
  }

}

//---------------------------------------------------------------------------------------
void pc_coms()
{
  if(Serial.available() > 0){
    char cmd = Serial.read();
    switch(cmd){
      case 'a':
        alpha = Serial.parseInt();
        break;
      
      case 'p':
        PCC_Relay = !PCC_Relay;
        break;

      default:
      break;
    }
  }
  if(Serial) // check performance cost on checking if serial is active
  {
    Serial.print("RPM: ");
    Serial.println(RPM);

    Serial.print("PCC Relay: ");
    Serial.println(PCC_Relay);

    Serial.print("Alpha: ");
    Serial.println(alpha);

    Serial.print("Theta: ");
    Serial.println(theta);
    
    Serial.print("Tubine Voltage: ");
    Serial.print(T_Voltage);
    Serial.println(" mV");
  
    Serial.print("Turbine Power: ");
    Serial.print(T_Power);
    Serial.println(" mW");

    Serial.print("State: ");
    switch(State)
    {
      case Wait:
        Serial.println("Wait");
      break;
      
      case Normal:
        Serial.println("Normal");
      break;
      
      case Regulate:
        Serial.println("Regulate");
      break;
      
      case Safety1:
        Serial.println("Safety1");
      break;
      
      case Safety2:
        Serial.println("Safety2");
      break;
      
      default:
        Serial.println("Error");
      break;
    }

    Serial.println();
  }
}

//---------------------------------------------------------------------------------------
void update_PCC_Relay()
{
  switch(Switching_State)
  {
    case S_Wait:
      if(last_PCC_Relay != PCC_Relay)
      {
        //Serial.println("toggled");
        last_PCC_Relay = PCC_Relay;
        Timer_PCC_Relay = millis();

        if(PCC_Relay)
        {
          //Serial.println("toggledH");
          digitalWrite(PCC_Relay_Set_Pin, HIGH);
          Switching_State = Toggled_H;
        }
        else
        {
          //Serial.println("toggledL");
          digitalWrite(PCC_Relay_Reset_Pin, HIGH);
          Switching_State = Toggled_L;
        }
      }
      break;

    case Toggled_H:
      if(millis() - Timer_PCC_Relay >= PCC_Relay_Pulse_Interval)
      {
        //Serial.println("WaitH");
        digitalWrite(PCC_Relay_Set_Pin, LOW);
        Switching_State = S_Wait;
      }
      break;

    case Toggled_L:
      if(millis() - Timer_PCC_Relay >= PCC_Relay_Pulse_Interval)
      {
        //Serial.println("WaitL");
        digitalWrite(PCC_Relay_Reset_Pin, LOW);
        Switching_State = S_Wait;
      }
      break;
  }
}

//---------------------------------------------------------------------------------------
void read_Sensors()
{
  E_Switch = digitalRead(Safety_SW);
  T_Voltage = ina260.readBusVoltage();
  T_Power = ina260.readPower();
}

//---------------------------------------------------------------------------------------
void AR_TX()
{
  Wire.beginTransmission(0x08);
  Wire.write('a');             //Start byte
  Wire.write(alpha);
  
  Wire.endTransmission();
}

//---------------------------------------------------------------------------------------
void AR_RX()
{
  Wire.requestFrom(0x08,3);
  while(Wire.available())
  {
    uint8_t byte_0 = Wire.read();
    uint8_t byte_1 = Wire.read();

    if(byte_0 == 'a')
    {
      Serial.print("Active Rectifier Responded: ");
      Serial.println(byte_1); //might not need to subtract 0;
    }
  }
}

//---------------------------------------------------------------------------------------
void uart_TX()
{
  Serial1.write('S');                 //Start byte
  Serial1.write(highByte(RPM));       //RPM
  Serial1.write(lowByte(RPM));        //RPM
  Serial1.write(highByte(T_Power));   //T_Power
  Serial1.write(lowByte(T_Power));    //T_Power
  Serial1.write(highByte(T_Voltage)); //Turbine Voltage
  Serial1.write(lowByte(T_Voltage));  //Turbine Voltage
  Serial1.write(E_Switch);            //E_Switch state
  Serial1.write('E');           //End byte
}

//---------------------------------------------------------------------------------------
void uart_RX()
{
  // ** | Start | RPM_H | RPM_L | Power_H | Power_L | End | ** //
  //Six byte minimum needed in RX buffer
  if (Serial1.available() >= 7)
  {
    //Check for start byte
    if (Serial1.read() == 'S')
    {
      //Read bytes, store in temp
      uint8_t temp1 = Serial1.read();
      uint16_t temp2_h = Serial1.read();
      uint16_t temp2_l = Serial1.read();
      uint8_t temp3 = Serial1.read();
      uint8_t temp4 = Serial1.read();

      //Check for end byte
      if (Serial1.read() == 'E')
      {
        //Save off
        alpha = temp1;
        theta = ((temp2_h << 8) | temp2_l);
        State = temp3;
        PCC_Relay = temp4;
      }
    }
  }
} //hello 
