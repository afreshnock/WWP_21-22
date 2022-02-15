#include <Adafruit_INA260.h>

Adafruit_INA260 ina260 = Adafruit_INA260();


enum States {Wait, Normal, Regulate, Safety1, Safety2};
States State = Wait;

//Load Variables
uint16_t L_Power;   //Load Power (mW)
uint16_t L_Voltage; //Load Voltage (mV)

//Turbine Variables
uint16_t T_Power;   //Turbine Power (mW)
uint16_t T_Voltage; //Turbine Power (mW)
uint16_t RPM;       //Turbine RPM   (r/min)
uint8_t load_Val;   //Load resistance value (1-256)
uint8_t alpha;      //Active Rectifier phase angle  (degrees)
uint8_t theta;      //Active Pitch angle            (degrees)
bool E_Switch;   //Bool indicating switch open   (normally closed)

//IDK Variables
uint16_t Peak_Power;  //(mW)
uint16_t Peak_RPM;    //(r/min)
uint16_t k1, k2, k3, thresh;  //(coefficients for Normal/regulate state break)


unsigned long Timer_50;
unsigned long Timer_250;

void setup()
{
  //UART1 (to turbine)
  pinMode(1, OUTPUT); //TX1
  pinMode(0, INPUT);  //RX1

  //I2C (to INA260)
  pinMode(18, OUTPUT); //SDA1
  pinMode(19, OUTPUT); //SCL1

  //Heatsink Fan Enable
  pinMode(20, OUTPUT);

  //PCC I/O Relay
  pinMode(24, OUTPUT);

  //Load control I/O
  pinMode(25, OUTPUT);  //MSB
  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(28, OUTPUT);
  pinMode(29, OUTPUT);
  pinMode(30, OUTPUT);
  pinMode(31, OUTPUT);
  pinMode(32, OUTPUT);  //LSB

  //5V monitor (analog)
  pinMode(41, INPUT);

  //start comms with INA260
  ina260.begin();
  load_Val = 256;
  set_Load(load_Val);
  //Turbine-Load UART
  Serial1.begin(9600);
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
    fan_ctrl();
    track_peaks();
    read_Sensors();
    //***********************************************************************
  
    //*********Code that runs dependent of the current machine State*********
    manage_State();
    //***********************************************************************
  }
  if(millis() - Timer_250 >= 250)
  {
    Timer_250 = millis();
    //log and tx data
    if(Serial.available() > 0){
      if(Serial.read() == 's'){
        //toggle datalog
      }
    }
  }
}


void manage_State(){
    switch (State)
  {
    case Wait:
      //If load recieves data from turbine, enter normal operation
      if (Serial1.available() >= 6)
      {
        //Do something
        State = Normal;
      }
      break;
    
    case Normal:
      //Power Tracking/Optimization
      

      //If overspeed
      if(RPM + k1*theta + k2*alpha + k3*load_Val > thresh)
      {
        State = Regulate;
      }
      //Discontinuity Condition
      if ((L_Voltage < (T_Voltage * 0.9)) && (RPM >= 100))
      {
        //Move to Safety2
        State = Safety2;
      }
      //Emergency switch condition
      if(!E_Switch)
      {
        //Move to Safety1
        State = Safety2;
      }
      break;

    case Regulate:


      //back to normal run state
      if(RPM + k1*theta + k2*alpha + k3*load_Val < thresh)
      {
        State = Normal;
      }
      //Discontinuity Condition
      if ((L_Voltage < (T_Voltage * 0.9)) && (RPM >= 100))
      {
        //Move to Safety2
        State = Safety2;
      }
      //Emergency switch condition
      if(!E_Switch)
      {
        //Move to Safety1
        State = Safety1;
      }
      break;
      
    case Safety1:
    //Emergency switch condition
      if(E_Switch)
      {
        //Move to Safety1
        State = Normal;
      }
      break;

    case Safety2:
      //Discontinuity Condition
      if ((L_Voltage < (T_Voltage * 0.9)) && (RPM >= 100))
      {
        //Move to Safety2
        State = Regulate;
      }
      break;

    default:
      State = Wait;
      break;
  }
}


void read_Sensors()
{
  L_Voltage = ina260.readCurrent();
  L_Power = ina260.readPower();
}

void set_Load(uint8_t val)
{
  digitalWriteFast(32, bitRead(val, 0));  //LSB
  digitalWriteFast(31, bitRead(val, 1));
  digitalWriteFast(30, bitRead(val, 2));
  digitalWriteFast(29, bitRead(val, 3));
  digitalWriteFast(28, bitRead(val, 4));
  digitalWriteFast(27, bitRead(val, 5));
  digitalWriteFast(26, bitRead(val, 6));
  digitalWriteFast(25, bitRead(val, 7));  //MSB
}

uint8_t fan_ctrl()
{
  //This function turns the fan on when load power exceeds 10W.

  if (L_Power >= 10000)
  {
    digitalWrite(20, HIGH); //pin 20 high
    return 1;
  }
  else
  {
    digitalWrite(20, LOW); //pin 20 low
    return 0;
  }
}

void track_peaks()
{
  if (L_Power > Peak_Power)
  {
    Peak_Power = L_Power;
  }

  if (RPM > Peak_RPM)
  {
    Peak_RPM = RPM;
  }
}

void uart_TX()
{
  Serial1.write('S');             //Start byte
  Serial1.write(alpha);           //Alpha
  Serial1.write(theta);           //Theta
  Serial1.write((byte)State);     //State
  Serial1.write('E');             //End byte
}

void uart_RX()
{
  // ** | Start | RPM_H | RPM_L | Power_H | Power_L | End | ** //
  // ** | Start | RPM_H | RPM_L | T_Power_H | T_Power_L | T_Voltage_H | T_Voltage_L | E_Switch | End | ** // 
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
      uint16_t temp3_h = Serial1.read();
      uint16_t temp3_l = Serial1.read();
      uint16_t temp4 = Serial1.read();

      //Check for end byte
      if (Serial1.read() == 'E')
      {
        //Save off
        RPM = ((temp1_h << 8) | temp1_l);
        T_Power = ((temp2_h << 8) | temp2_l);
        T_Voltage = ((temp3_h << 8) | temp3_l);
        E_Switch = temp4;
      }
      else
      {
        //Dump buffer
        while (Serial1.available())
        {
          char dumpy = Serial1.read();
        }
      }
    }
    else
    {
      //Dump buffer
      while (Serial1.available())
      {
        char dumpy = Serial1.read();
      }
    }
  }
} //hello 