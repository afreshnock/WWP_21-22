#include "src/INA260/Adafruit_INA260.h"
//Local libraries need to either be directly in the project folder, or in the src folder. 
//src linker only works in Arduino IDE 1.5+ I believe.
#include "src/WP_SD.h"

Adafruit_INA260 ina260 = Adafruit_INA260();

enum States {Wait, Normal, Regulate, Safety1, Safety2};
States State = Wait;

enum TestStates {TWait, Man, Auto, StepWS, StepAlpha, StepLoad, StepTheta};
TestStates TestState = TWait;

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

//AutoTest Variables -----------------------------------------------------------------
bool EntryScreen = true;

unsigned timeWS= 15000;
double minWindSpeed = 12.0;
double maxWindSpeed = 16.0;
double incWindSpeed = 0.5;
bool staticWS = false;
bool incrementingWS;

unsigned timeAlpha = 1000;
int minAlpha = 0;
int maxAlpha = 0;
int incAlpha = 0;
bool incrementingAlpha;
bool staticAlpha = false;

unsigned timeLoad = 4000;
float minLoad = 1;
float maxLoad = 4;
float incLoad = 1;
bool incrementingLoad;
bool staticLoad = false;

unsigned timeTheta = 4000;
uint16_t minTheta = 200;
uint16_t maxTheta = 3200;
uint16_t incTheta = 600;
bool incrementingTheta;
bool staticTheta = false;

bool PCCOMS = false;
unsigned timeT;
bool paused = false; 
//-------------------------------------------------------------------------------------
unsigned logInterval = 250;
unsigned State_Interval = 250;
//IDK Variables
uint16_t Peak_Power;  //(mW)
uint16_t Peak_RPM;    //(r/min)
uint16_t k1, k2, k3, thresh;  //(coefficients for Normal/regulate state break)

unsigned long Timer_50;
unsigned long Timer_250;
unsigned long Timer_Log;
unsigned long Comms_Timeout;
bool Turbine_Comms;

unsigned long Timer_T;

bool PCC_Relay;
bool Auto_PCC;

//---------------------------------------------------------------------------------------
void setup()
{
  //init K coeffs
  k1 = 1;
  k2 = 1;
  k3 = 1;
  //tunnel_setting = 0;
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
  set_load(minLoad);
  
  //Turbine-Load UART
  Serial1.begin(9600);

  //Set up coms with PC
  Serial.begin(9600);
  
  //set up timers
  Timer_50 = millis();
  Timer_250 = millis();
  Timer_Log = millis();

  Timer_T = millis();
  
  try_SD_begin(BUILTIN_SDCARD);

  PCC_Relay = false;
  Auto_PCC = false;
  theta = minTheta;
  analogWriteFrequency(6, 200000);
  analogWriteResolution(8);
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
    if(TestState == Man) manage_state();
    //***********************************************************************
  }
  if(millis() - Timer_250 >= State_Interval)
  {
    Timer_250 = millis();
    //unsigned long ts = micros();
    uart_TX();
    manage_sim_state();
    analogWrite(6, tunnel_setting);
    if(PCCOMS) pc_coms();
    //Serial.println(micros() - ts);
    
    if(Auto_PCC)
    {
      if(L_Voltage < 3300)
      {
        PCC_Relay = true;
      }
      else
      {
        PCC_Relay = false;
      }
    }
    digitalWrite(24, PCC_Relay);
  }
  
  if(millis() - Timer_Log >= logInterval){
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
      + "," + k1
      + "," + k2
      + "," + k3
      + "," + thresh
      + "," + tunnel_setting
    );
  }
  
}

//---------------------------------------------------------------------------------------
void manage_sim_state(){

  switch (TestState)
  {
    case TWait:
        if(Serial && EntryScreen)
        {
          PCCOMS = false;
          Serial.println("Manual (M) / Auto (A)");
          EntryScreen = false;
        }
        if(Serial.available() > 0)
        {
          uint8_t cmd = Serial.read();
          switch(cmd)
          {
            case 'M':
              TestState = Man;
              PCCOMS = true;
            break;

            case 'A':
              TestState = Auto;
              PCCOMS = true;
            break;

            default:
              Serial.println("Invalid Selection...");
              EntryScreen = true;
            break;
          }
        }
      break;

    case Man:
      break;

    case Auto:

      if(SDConnected)
      {
        Logging = false;
        toggle_Logging();
        Auto_PCC = false;
        PCC_Relay = false;

        incrementingWS = false;
        incrementingAlpha = false;
        incrementingTheta = false;

        staticAlpha = (maxAlpha - minAlpha == 0 || incAlpha == 0);
        staticLoad = (maxLoad - minLoad == 0 || incLoad == 0);
        staticTheta = (maxTheta - minTheta == 0 || incTheta == 0);
        staticWS = (maxWindSpeed - minWindSpeed == 0 || incWindSpeed ==0);
        
        set_load(minLoad);
        alpha = minAlpha;
        theta = minTheta;
        set_windspeed(minWindSpeed);

        Serial.println("Automatic Testing Initializing");

        uint16_t wsIters = ((maxWindSpeed - minWindSpeed) / incWindSpeed);
        uint16_t thetaIters = ((maxTheta - minTheta) / incTheta) * wsIters;
        uint16_t loadIters = ((maxLoad - minLoad) / incLoad) * thetaIters;
        uint16_t alphaIters = ((maxAlpha - minAlpha) / incAlpha) * loadIters;

        unsigned long wsTime = wsIters * timeWS / 1000;
        unsigned long thetaTime =  wsIters * timeTheta / 1000;
        unsigned long loadTime = loadIters * timeLoad / 1000;
        unsigned long alphaTime = alphaIters * timeAlpha / 1000;

        unsigned long totalTime = wsTime + thetaTime + loadTime + alphaTime;
        Serial.println(totalTime);
        int Seconds = totalTime%60;
        int Minutes = (totalTime/60)%60;
        int Hours = (totalTime/3600)%24;
        int Days = (totalTime/(3600*24));
        String t = (String)Days + "-" + Hours + ":" + Minutes + ":" + Seconds;
        Serial.print("Estimated Test Duration: ");
        Serial.println(t);
        Timer_T = millis();
        TestState = StepLoad;
        timeT = loadTime;
      }
      else
      {
        Serial.println("SD card malfunction.");
        PCCOMS = false;
        TestState = TWait;
        EntryScreen = true;
      }
      break;

    case StepLoad:
      if(millis() - Timer_T >= timeT)
      {
        Timer_T = millis();
        if(!staticLoad)
        {
          timeT = timeLoad;
          if(resistance + incLoad <= maxLoad)
          {
            resistance += incLoad;
            set_load(resistance);
          }
          else
          {
            resistance = minLoad;
            set_load(resistance);
            TestState = StepAlpha;
          }
        }
        else
        {
          TestState = StepAlpha;
        }
      }
      break;

    case StepAlpha:
        if(!staticAlpha)
        {
          timeT = timeAlpha;
          if(alpha + incAlpha <= maxAlpha)
          {
            alpha += incAlpha;
            TestState = StepLoad;
          }
          else
          {
            alpha = minAlpha;
            TestState = StepTheta;
          }
        }
        else
        {
          TestState = StepTheta;
        }
      break;

    case StepTheta:
        if(!staticTheta)
        {
          timeT = timeTheta;
          if(theta + incTheta <= maxTheta)
          {
            theta += incTheta;
            TestState = StepLoad;
          }
          else
          {
            theta = minTheta;
            TestState = StepWS;
          }
        }
        else
        {
          TestState = StepWS;
        }
      break;

    case StepWS:
        if(!staticWS)
        {
          timeT = timeWS;
          if(windspeed + incWindSpeed <= maxWindSpeed)
          {
            windspeed += incWindSpeed;
            set_windspeed(windspeed);
            TestState = StepLoad;
          }
          else
          {
            Logging = false;
            TestState = TWait;
            set_windspeed(0.0);
            EntryScreen = true;
            PCCOMS = false;
          }
        }
        else
        {
          Logging = false;
          TestState = TWait;
          set_windspeed(0.0);
          EntryScreen = true;
          PCCOMS = false;
        }
      break;

  }
}

//---------------------------------------------------------------------------------------
void manage_state(){
  switch (State)
  {
    
    case Wait:
      //If load recieves data from turbine, enter normal operation
      if (Turbine_Comms)
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
      
      theta = 100;
      PCC_Relay = true;

      //Emergency switch condition
      if(E_Switch)
      {
        theta = 2000;
        PCC_Relay = false;
        State = Normal;
      }
      break;

    case Safety2:
      
      theta = 100;
      PCC_Relay = true;

      //Discontinuity Condition
      if ((L_Voltage > (T_Voltage * 0.9)) && (RPM >= 100))
      {
        PCC_Relay = false;
        theta = 2000;
        State = Normal;
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
      case 'Q':
        TestState = TWait;
        Serial.println("Test cancelled");
        if(Logging) toggle_Logging();
        set_windspeed(0.0);
        EntryScreen = true;
        PCCOMS = false;
      break;

      case 'P':        
        paused = true;
        while(paused)
        {
          if(Serial.available() > 0)
          {
            if(Serial.read() == 'P')
            {
              paused = false;
            }
          }
        }
      break;

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
        if (!Auto_PCC)
        {
          PCC_Relay = !PCC_Relay;
          digitalWrite(24, PCC_Relay);
        }
      break;

      case 'm':
        Auto_PCC = !Auto_PCC;
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
        set_windspeed(Serial.parseFloat());
       
      default:
        Serial.println("Command not recognized");
      break;
    }
  }

//---------------------------------------------------------------------------------------
  if(Serial) // check performance cost on checking if serial is active
    {
        Serial.print("Turbine Connected: ");
        Serial.println(Turbine_Comms);
        
        Serial.print("PCC Relay: ");
        Serial.println(PCC_Relay);

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
        
        Serial.print("(w) Windspeed (0.0-17.0): ");
        Serial.println(windspeed);
        
        Serial.print("(a) Alpha: ");
        Serial.println(alpha);

        Serial.print("(t) Theta (100 - 3380): ");
        Serial.println(theta);

        Serial.print("(r) Load ( 0- 64 ): ");
        Serial.print((float)load_Val/255*63.75);
        Serial.println(" Ohms");
        
        Serial.println("(1)-k1 / (2)-k2 / (3)-k3 / (h)-thressh: ");
        Serial.println((String) k1 + ", " + k2 + ", " + k3 + ", " + thresh);
        
        //Serial.println("RPM + k1*theta + k2*alpha + k3*load_Val > thresh");
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
  if(millis() - Comms_Timeout >= 2*State_Interval)
  {
    Comms_Timeout = millis();
    Turbine_Comms = false;
    RPM = 0;
    T_Power = 0;
    T_Voltage = 0;
  }
} //hello 
