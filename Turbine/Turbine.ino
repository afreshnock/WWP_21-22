#include "src/INA260/Adafruit_INA260.h"
#include "src/PA12/PA12.h"
//Local libraries need to either be directly in the project folder, or in the src folder. 
//src linker only works in Arduino IDE 1.5+ I believe.
#include "RPM.h"

Adafruit_INA260 ina260 = Adafruit_INA260();


enum States {Normal, Safety};
States State = Normal;

//Load Variables
uint16_t L_Power;   //Load Power (mW)
uint16_t L_Voltage; //Load Voltage (mV)

//Turbine Variables
uint16_t T_Power;   //Turbine Power (mW)
uint16_t T_Voltage; //Turbine Voltage (mV)
uint16_t RPM;       //Turbine RPM   (r/min)
uint8_t alpha;      //Active Rectifier phase angle  (degrees)
uint8_t theta;      //Active Pitch angle   
bool E_Switch;      //Bool indicating switch open         (degrees)

//IDK Variables
uint16_t Peak_Power;  //(mW)
uint16_t Peak_RPM;    //(r/min)

unsigned long Timer_50;
unsigned long Timer_250;

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

  //start comms with INA260
  ina260.begin();

  //Turbine-Load UART
  Serial1.begin(9600);

  //Set up coms with PC
  Serial.begin(9600);

  //initialize RPM
  setup_RPM();

  //set up timers
  Timer_50 = millis();
  Timer_250 = millis();
}

void loop()
{
  uart_RX();
  if(millis() - Timer_50 >= 50)
  {
    Timer_50 = millis();
    //*********Code that runs all the time independent of the State**********
    refresh_RPM();
    read_Sensors();
    //***********************************************************************
  
  }
  if(millis() - Timer_250 >= 250)
  {
    Timer_250 = millis();
    
    uart_TX();


  }



/* -- not sure if this state machine is needed yet.
  //*********Code that runs dependent of the current machine State*********

  switch (State)
  {
    case Normal:


      //Discontinuity Condition
      if ((L_Power < T_Power * 0.9) && (RPM >= 100))
      {

        //Do something

        //Move to Safety State
        State = Safety;
      }

      break;

    case Safety:
      break;

    default:
      break;
  }
  //***********************************************************************
*/

}

void read_Sensors()
{
  T_Voltage = ina260.readCurrent();
  T_Power = ina260.readPower();
}


void uart_TX()
{
  Serial1.write('S');           //Start byte
  Serial1.write(RPM);           //RPM
  Serial1.write(T_Power);       //Turbine Power
  Serial1.write(T_Voltage);     //Turbine Voltage
  Serial1.write(E_Switch);      //E_Switch state
  Serial1.write('E');           //End byte
}

void uart_RX()
{
  // ** | Start | RPM_H | RPM_L | Power_H | Power_L | End | ** //

  //Six byte minimum needed in RX buffer
  if (Serial1.available() >= 6)
  {
    //Check for start byte
    if (Serial1.read() == 'S')
    {
      //Read bytes, store in temp
      uint16_t temp1_h = Serial1.read();
      uint16_t temp1_l = Serial1.read();
      uint16_t temp2_h = Serial1.read();
      uint16_t temp2_l = Serial1.read();
      uint16_t temp3 = Serial1.read();

      //Check for end byte
      if (Serial1.read() == 'E')
      {
        //Save off
        alpha = ((temp1_h << 8) + temp1_l);
        theta = ((temp2_h << 8) + temp2_l);
        State = temp3;
      }
      else
      {
        //Dump buffer
        while (Serial1.available())
        {
          Serial1.read();
        }
      }
    }
    else
    {
      //Dump buffer
      while (Serial1.available())
      {
        Serial1.read();
      }
    }
  }
} //hello 
