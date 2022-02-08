#include <Adafruit_INA260.h>

Adafruit_INA260 ina260 = Adafruit_INA260();


enum States {Normal, Safety};
States State = Normal;

//Load Variables
uint16_t L_Power;   //Load Power (mW)


//Turbine Variables
uint16_t T_Power;   //Turbine Power (mW)
uint16_t RPM;       //Turbine RPM   (r/min)
uint8_t alpha;      //Active Rectifier phase angle  (degrees)
uint8_t theta;      //Active Pitch angle            (degrees)

//IDK Variables
uint16_t Peak_Power;  //(mW)
uint16_t Peak_RPM;    //(r/min)

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

  //Turbine-Load UART
  Serial1.begin(9600);

}

void loop()
{
  //*********Code that runs all the time independent of the State**********
  fan_ctrl();
  track_peaks();

  //***********************************************************************





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


}

void set_load(uint8_t val)
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

      //Check for end byte
      if (Serial1.read() == 'E')
      {
        //Save off
        RPM = ((temp1_h << 8) + temp1_l);
        T_Power = ((temp2_h << 8) + temp2_l);
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
