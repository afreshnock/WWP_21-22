#include "src/INA260/Adafruit_INA260.h"
//Local libraries need to either be directly in the project folder, or in the src folder. 
//src linker only works in Arduino IDE 1.5+ I believe.
#include "src/WP_SD.h"

Adafruit_INA260 ina260 = Adafruit_INA260();

enum States {Wait, Normal, Regulate, Safety1, Safety2};
States State = Wait;

//Load Variables
uint16_t L_Power;   //Load Power (mW)
uint16_t L_Voltage; //Load Voltage (mV)
uint16_t L_Current; //Load Current (mA)

//Turbine Variables
uint16_t T_Power;   //Turbine Power (mW)
uint16_t T_Voltage; //Turbine Power (mW)
uint16_t RPM;       //Turbine RPM   (r/min)
uint8_t load_Val;   //Load resistance value (1-255)
uint8_t alpha;      //Active Rectifier phase angle  (degrees)
uint16_t theta;      //Active Pitch angle            (degrees)
uint8_t tunnel_setting;
bool E_Switch;      //Bool indicating switch open   (normally closed)

//IDK Variables
uint16_t Peak_Power;  //(mW)
uint16_t Peak_RPM;    //(r/min)
uint16_t k1, k2, k3, thresh;  //(coefficients for Normal/regulate state break)

unsigned long Timer_50;
unsigned long Timer_250;
bool PCC_Relay;

//---------------------------------------------------------------------------------------
void setup()
{
  //init K coeffs
  k1, k2, k3 = 1;
  tunnel_setting = 0;
  thresh = 100;
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
  load_Val = 255;
  set_load(load_Val);
  
  //Turbine-Load UART
  Serial1.begin(31250);

  //Set up coms with PC
  Serial.begin(9600);
  
  //set up timers
  Timer_50 = millis();
  Timer_250 = millis();

  try_SD_begin(BUILTIN_SDCARD);

  PCC_Relay = false;
}

//---------------------------------------------------------------------------------------
void loop()
{
  uart_RX();
  if(millis() - Timer_50 >= 50)
  {
    Timer_50 = millis();
    //*********Code that runs all the time independent of the State**********
    fan_ctrl();
    track_peaks();
    read_sensors();
    //***********************************************************************
  
    //*********Code that runs dependent of the current machine State*********
    manage_state();
    //***********************************************************************
  }
  if(millis() - Timer_250 >= 250)
  {
    Timer_250 = millis();
    unsigned long ts = micros();
    uart_TX();
    pc_coms();
    try_Log_Data((String)
              RPM
      + "," + E_Switch 
      + "," + alpha 
      + "," + theta 
      + "," + load_Val 
      + "," + L_Voltage 
      + "," + L_Current 
      + "," + L_Power
      + "," + T_Voltage
      + "," + T_Power
      + "," + State
      + "," + k1
      + "," + k2
      + "," + k3
      + "," + thresh
      + "," + tunnel_setting
    );
    Serial.println(micros() - ts);
  }
}

//---------------------------------------------------------------------------------------
void manage_state(){
  switch (State)
  {
    case Wait:
      //If load recieves data from turbine, enter normal operation
      if (RPM != 0)
      {
        //Do something
        State = Normal;
      }
      break;
    
    case Normal:
      //Optimize for power
      

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
      //Regulate RPM at 11m/s val (PID? keep at val)


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
      //do safety1 stuff


      //Emergency switch condition
      if(E_Switch)
      {
        //Move to Safety1
        State = Normal;
      }
      break;

    case Safety2:
      //Do safety2 stuff?


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
        load_Val = Serial.parseInt();
        set_load(load_Val);
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

      case '1':
        k1 = Serial.parseInt();
      break;

      case '2':
        k2 = Serial.parseInt();
      break;

      case '3':
        k3 = Serial.parseInt();
      break;

      case 'h':
        thresh = Serial.parseInt();
      break;

      case 'w':
        tunnel_setting = Serial.parseInt();

      default:
        Serial.println("Command not recognized");
      break;
    }
  }

//---------------------------------------------------------------------------------------
  if(Serial) // check performance cost on checking if serial is active
    {
        Serial.print("RPM: ");
        Serial.println(RPM);

        Serial.print("Tunnel Set: ");
        Serial.println(tunnel_setting);

        Serial.print("Load Voltage: ");
        Serial.print(L_Voltage);
        Serial.println(" mV");

        Serial.print("Load Current: ");
        Serial.print(L_Current);
        Serial.println(" mA");
      
        Serial.print("Load Power: ");
        Serial.print(L_Power);
        Serial.println(" mW");

        Serial.print("Calculated Power: ");
        Serial.print((float) L_Voltage * L_Current / 1000000);
        Serial.println(" W");

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
        
        Serial.print("(a) Alpha: ");
        Serial.println(alpha);

        Serial.print("(t) Theta: ");
        Serial.println(theta);

        Serial.print("(r) Load: ");
        Serial.print((float)load_Val/255*63.75);
        Serial.println(" Ohms");
        
        Serial.println("(1)-k1 / (2)-k2 / (3)-k3 / (h)-thressh: ");
        Serial.println((String) k1 + ", " + k2 + ", " + k3 + ", " + thresh);

        Serial.print("Overspeed Condition: ");
        Serial.print(RPM + k1*theta + k2*alpha + k3*load_Val);
        Serial.print(" > ");
        Serial.println(thresh);
        
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
void read_sensors()
{
  L_Voltage = ina260.readBusVoltage();
  L_Power = ina260.readPower();
  L_Current = ina260.readCurrent();
}

//---------------------------------------------------------------------------------------
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
  Serial1.write('E');             //End byte
}

//---------------------------------------------------------------------------------------
void uart_RX()
{
  // ** | Start | RPM_H | RPM_L | Power_H | Power_L | End | ** //
  // ** | Start | RPM_H | RPM_L | T_Power_H | T_Power_L | T_Voltage_H | T_Voltage_L | E_Switch | End | ** // 
  //Six byte minimum needed in RX buffer
  if (Serial1.available() >= 3)
  {
    //Check for start byte
    if (Serial1.read() == 'S')
    {
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
