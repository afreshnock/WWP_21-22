#include "src/INA260/Adafruit_INA260.h"
//Local libraries need to either be directly in the project folder, or in the src folder. 
//src linker only works in Arduino IDE 1.5+ I believe.
#include "src/WP_SD.h"

Adafruit_INA260 ina260 = Adafruit_INA260();

enum States {Wait, Normal, Regulate, Safety1, Safety2};
States State = Wait;

//Preset Go-to resistances
float cutin_r = 5;
float med_r = 2.5;
float maxpwr_r = 1.5;

//Preset Go-to theta angles
float cutin_t = 1400;
float maxpwr_t = 800;
float brake_t = 3200;
float reg_t = 1400;

//Preset alpha angles
uint8_t norm_a = 0;
uint8_t pwrreg_a = 30;

uint16_t min_pitch_pwr = 2000;
uint16_t min_turb_v = 3300;


//Load Variables
uint16_t L_Power;   //Load Power (mW)
uint16_t L_Voltage; //Load Voltage (mV)
uint16_t L_Current; //Load Current (mA)

//Turbine Variables
uint16_t T_Power;   //Turbine Power (mW)
uint16_t T_Voltage; //Turbine Power (mW)
uint16_t RPM;       //Turbine RPM   (r/min)
uint8_t load_Val;   //Load resistance value (1-255)
float resistance;
uint8_t alpha;      //Active Rectifier phase angle  (degrees)
uint16_t theta;      //Active Pitch angle            (degrees)
uint8_t tunnel_setting;
double windspeed;
bool E_Switch;      //Bool indicating switch open   (normally closed)

//IDK Variables
uint16_t Peak_Power;  //(mW)
uint16_t Peak_RPM;    //(r/min)

unsigned long Timer_Fast;
unsigned long Timer_Slow;
unsigned long Timer_Log;
unsigned long Comms_Timeout;

unsigned Fast_Interval = 50;
unsigned Slow_Interval = 250;
unsigned Log_Interval = 500;

bool Turbine_Comms;
bool PCC_Relay;

//---------------------------------------------------------------------------------------
void setup()
{
  init_pins();
  init_coms();
  init_timers();

  set_load(cutin_r);
  PCC_Relay = false;
  theta = cutin_t;
  alpha = norm_a;
}

//---------------------------------------------------------------------------------------
void loop()
{
  uart_RX();
  if(millis() - Timer_Fast >= Fast_Interval)
  {
    Timer_Fast = millis();

    fan_ctrl();
    track_peaks();
    read_sensors();
    manage_state();
  }
  if(millis() - Timer_Slow >= Slow_Interval)
  {
    Timer_Slow = millis();

    uart_TX();
    pc_coms();
  }
  
  if(millis() - Timer_Log >= Log_Interval)
  {
    Timer_Log = millis();

    try_Log_Data((String)
              RPM
      + "," + windspeed
      + "," + E_Switch 
      + "," + alpha 
      + "," + theta 
      + "," + resistance 
      + "," + load_Val 
      + "," + L_Voltage 
      + "," + L_Current 
      + "," + L_Power
      + "," + T_Voltage
      + "," + T_Power
      + "," + State
    );
  }
}

//---------------------------------------------------------------------------------------
void manage_state(){
  switch (State)
  {
    case Wait:
      //If load recieves data from turbine, enter normal operation

      if(Turbine_Comms)
      {
        State = Normal;
      }
      break;
    
    case Normal:
      //Optimize for power
      
      if(T_Power >= min_pitch_pwr)
      {
        if(PCC_Relay) PCC_Relay = false;
        theta = maxpwr_t; // to be optimized
        set_load(med_r);
      }
      if(T_Voltage >= min_turb_v)
      {
        set_load(maxpwr_r); // to be optimized
      }
      if(RPM > 3000)
      {
        State = Regulate;
      }

      //Emergency switch condition
      if(!E_Switch)
      {
        //Move to Safety1
        State = Safety1;
      }
      //Discontinuity Condition
      if ((L_Voltage < (T_Voltage * 0.9)) && (RPM >= 100))
      {
        //Move to Safety2
        State = Safety2;
      }
      break;

    case Regulate:
      //Regulate RPM at 11m/s val (PID? keep at val)
      
      if(RPM > 3000)
      {
        theta--; // to be optimized
        set_load(med_r);
        if(RPM > 4000)
        {
          alpha = pwrreg_a;
        }
        else
        {
          alpha = norm_a;
        }
      }
      else
      {
        State = Normal;
      }
      //Emergency switch condition
      if(!E_Switch)
      {
        //Move to Safety1
        State = Safety1;
      }
      //Discontinuity Condition
      if ((L_Voltage < (T_Voltage * 0.9)) && (RPM >= 100))
      {
        //Move to Safety2
        State = Safety2;
      }
      break;
      
    case Safety1:
      //do safety1 stuff
      theta = brake_t;
      PCC_Relay = true;

      //Emergency switch condition
      if(E_Switch)
      {
        //Move to Safety1
        theta = cutin_t;
        State = Wait;
      }
      break;

    case Safety2:
      //Do safety2 stuff?
      theta = brake_t;
      PCC_Relay = true;

      //Discontinuity Condition
      if ((L_Voltage > (T_Voltage * 0.9)) && (RPM >= 100))
      {
        //Move to Safety2
        theta = cutin_t;
        State = Wait;
      }
      break;

    default:
      State = Wait;
      break;
  }
}

//---------------------------------------------------------------------------------------
void pc_coms()
{
  if(Serial.available() > 0)
  {
    uint8_t cmd = Serial.read();

    switch(cmd)
    {
      case 's':
        if(SDConnected)
        {
          toggle_Logging();
        }
      break;

      case 'r':
        resistance = Serial.parseFloat();
        set_load(resistance);
      break;

      case 'p':
        PCC_Relay = !PCC_Relay;
        digitalWrite(24, PCC_Relay);
      break;

      case 't':
        theta = Serial.parseInt();
      break;

      case 'a':
        alpha = Serial.parseInt();
      break;
      
      case 'w':
        set_windspeed(Serial.parseFloat());
       
      default:
        Serial.println("Command not recognized");
      break;
    }
  }

  if(Serial) // check performance cost on checking if serial is active
    {
        Serial.print("RPM: ");
        Serial.println(RPM);

        Serial.print("Load Voltage: ");
        Serial.print(L_Voltage);
        Serial.println(" mV");

        Serial.print("Load Current: ");
        Serial.print(L_Current);
        Serial.println(" mA");
      
        Serial.print("Load Power: ");
        Serial.print(L_Power);
        Serial.println(" mW");

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
        
        Serial.print("Emergency Switch: ");
        Serial.println(E_Switch);
        
        Serial.print("(w) Tunnel Set: ");
        Serial.println(tunnel_setting);
        
        Serial.print("(a) Alpha: ");
        Serial.println(alpha);

        Serial.print("(t) Theta (100 - 3380): ");
        Serial.println(theta);

        Serial.print("(r) Load: ");
        Serial.print((float)load_Val/255*63.75);
        Serial.println(" Ohms");
        
        Serial.print("(s) Logging: ");
        if(Logging){
          Serial.println("True");
        }
        else{
          Serial.println("False");
        }
        
        Serial.println();
    }
}

//---------------------------------------------------------------------------------------
void set_windspeed(double ws)
{
  windspeed = ws;
  double temp = (0.303 + 18.7 * windspeed + -0.67 * pow(windspeed,2) + 0.0317 * pow(windspeed,3));
  if(temp > 255){
    temp = 255;
  }
  tunnel_setting = (uint8_t)temp;
}

//---------------------------------------------------------------------------------------
void init_pins()
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

  analogWriteFrequency(6, 200000);
  analogWriteResolution(8);
}

//---------------------------------------------------------------------------------------
void init_coms()
{
  ina260.begin();
  Serial1.begin(9600);
  Serial.begin(9600);
  try_SD_begin(BUILTIN_SDCARD);
}

//---------------------------------------------------------------------------------------
void init_timers()
{
  Timer_Fast = millis();
  Timer_Slow = millis();
  Timer_Log = millis();
}

//---------------------------------------------------------------------------------------
void read_sensors()
{
  L_Voltage = ina260.readBusVoltage();
  L_Power = ina260.readPower();
  L_Current = ina260.readCurrent();
}

//---------------------------------------------------------------------------------------
void set_load(float r)
{
  resistance = r;
  if(r > 63.75) resistance = 63.75;
  load_Val = (int)(resistance*4);
  digitalWriteFast(32, bitRead(load_Val, 0));  //LSB
  digitalWriteFast(31, bitRead(load_Val, 1));
  digitalWriteFast(30, bitRead(load_Val, 2));
  digitalWriteFast(29, bitRead(load_Val, 3));
  digitalWriteFast(28, bitRead(load_Val, 4));
  digitalWriteFast(27, bitRead(load_Val, 5));
  digitalWriteFast(26, bitRead(load_Val, 6));
  digitalWriteFast(25, bitRead(load_Val, 7));  //MSB
}

//---------------------------------------------------------------------------------------
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

//---------------------------------------------------------------------------------------
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

//---------------------------------------------------------------------------------------
void uart_TX()
{
  Serial1.write('S');             //Start byte
  Serial1.write(alpha);           //Alpha
  Serial1.write(highByte(theta));           //Theta
  Serial1.write(lowByte(theta));           //Theta
  Serial1.write((byte)State);     //State
  Serial1.write(PCC_Relay);
  Serial1.write('E');             //End byte
}

//---------------------------------------------------------------------------------------
void uart_RX()
{
  // ** | Start | RPM_H | RPM_L | Power_H | Power_L | End | ** //
  // ** | Start | RPM_H | RPM_L | T_Power_H | T_Power_L | T_Voltage_H | T_Voltage_L | E_Switch | End | ** // 
  //Six byte minimum needed in RX buffer
  if (Serial1.available() >= 9)
  {
    //Check for start byte
    if (Serial1.read()  == 'S')
    {
      Comms_Timeout = millis();
      Turbine_Comms = true;
      //Read bytes, store in temp255
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
    }
  }
  //If no data is recieved in the time that 2 packets are expected to be recieved, the turbine is assumed disconnected/off.
  if(millis() - Comms_Timeout >= 2*Slow_Interval)
  {
    Comms_Timeout = millis();
    Turbine_Comms = false;
    RPM = 0;
    T_Power = 0;
    T_Voltage = 0;
  }
} //hello 
