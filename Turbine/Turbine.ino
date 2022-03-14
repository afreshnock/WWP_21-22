#include "src/INA260/Adafruit_INA260.h"
#include "src/PA12/PA12.h"
//Local libraries need to either be directly in the project folder, or in the src folder. 
//src linker only works in Arduino IDE 1.5+ I believe.
#include "RPM.h"

#define ID_NUM 0

PA12 myServo(&Serial2, 8, 1); 
//          (&Serial, enable_pin,  Tx Level)
Adafruit_INA260 ina260 = Adafruit_INA260();


enum States {Wait, Normal, Regulate, Safety1, Safety2};
States State = Normal;

//Load Variables
uint16_t L_Power;   //Load Power (mW)
uint16_t L_Voltage; //Load Voltage (mV)

//Turbine Variables
uint16_t T_Power;   //Turbine Power (mW)
uint16_t T_Voltage; //Turbine Voltage (mV)
uint16_t RPM;       //Turbine RPM   (r/min)
uint8_t alpha;      //Active Rectifier phase angle  (degrees)
uint16_t theta;      //Active Pitch angle   
bool E_Switch = true;      //Bool indicating switch open

//IDK Variables
uint16_t Peak_Power;  //(mW)
uint16_t Peak_RPM;    //(r/min)

unsigned long Timer_50;
unsigned long Timer_250;

//---------------------------------------------------------------------------------------
void setup()
{
  //UART1 (to turbine)
  pinMode(1, OUTPUT); //TX1
  pinMode(0, INPUT);  //RX1

  //UART2 (to linear actuators)
  pinMode(9, OUTPUT); //TX1
  pinMode(10, INPUT);  //RX1

  //I2C (to INA260)
  pinMode(18, OUTPUT); //SDA1
  pinMode(19, OUTPUT); //SCL1

  //start comms with active rectifier
  Wire.begin();

  //start comms with INA260
  ina260.begin();

  //Turbine-Load UART
  Serial1.begin(31250);

  //Set up coms with PC
  Serial.begin(9600);
  
  //linear actuator
  myServo.begin(32);  
  
  //initialize RPM
  setup_RPM();

  //set up timers
  Timer_50 = millis();
  Timer_250 = millis();
}

//---------------------------------------------------------------------------------------
void loop()
{
  uart_RX();
  refresh_RPM();
  if(millis() - Timer_50 >= 50)
  {
    Timer_50 = millis();
    //*********Code that runs all the time independent of the State**********
    RPM = outputRPM;
    read_Sensors();
    //***********************************************************************
  
  }
  
  if(millis() - Timer_250 >= 250)
  {
    Timer_250 = millis();
    
    uart_TX();
    AR_TX();
    AR_RX();
    pc_coms();
    myServo.goalPosition(ID_NUM, theta);
  }

}

//---------------------------------------------------------------------------------------
void pc_coms()
{
  if(Serial) // check performance cost on checking if serial is active
  {
    Serial.print("RPM: ");
    Serial.println(RPM);

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
void read_Sensors()
{
  T_Voltage = ina260.readBusVoltage();
  T_Power = ina260.readPower();
}

//---------------------------------------------------------------------------------------
void AR_RX()
{
  Wire.beginTransmission(0x08);
  Wire.write('a');             //Start byte
  Wire.write(alpha);           //Alpha
  Wire.endTransmission();
}

//---------------------------------------------------------------------------------------
void AR_TX()
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
  if (Serial1.available())
  {
    //Check for start byte
    if (Serial1.read() == 'S')
    {
      //Read bytes, store in temp
      uint8_t temp1 = Serial1.read();
      uint16_t temp2_h = Serial1.read();
      uint16_t temp2_l = Serial1.read();
      uint8_t temp3 = Serial1.read();

      //Check for end byte
      if (Serial1.read() == 'E')
      {
        //Save off
        alpha = temp1;
        theta = ((temp2_h << 8) | temp2_l);
        State = temp3;
      }
      
    }
  }
} //hello 
