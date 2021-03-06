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
//       theta_pos = (int)(31.3479*theta + 208.084)
uint16_t theta_pos; 
//    theta = (0.0319*theta_pos - 6.6379)   -- might need offest
float theta;      //  Active Pitch angle            (degrees)
uint8_t tunnel_setting;
double windspeed;
bool E_Switch;      //Bool indicating switch open   (normally closed)

//AutoTest Variables -----------------------------------------------------------------
bool EntryScreen = true;

unsigned timeWS= 15000;
double minWindSpeed = 10;
double maxWindSpeed = 11;
double incWindSpeed = 0.25;
bool staticWS = false;
bool incrementingWS;

unsigned timeAlpha = 1000;
int minAlpha = 0;
int maxAlpha = 0;
int incAlpha = 0;
bool incrementingAlpha;
bool staticAlpha = true;

unsigned timeLoad = 1000;
float minLoad = 1;
float maxLoad = 5.0;
float incLoad = 0.25;
bool incrementingLoad;
bool staticLoad = false;

unsigned timeTheta = 8000;
uint16_t minTheta = 15;
uint16_t maxTheta = 40;
uint16_t incTheta = 5;
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

  Auto_PCC = false;
  set_theta(minTheta);
  analogWriteFrequency(6, 200000);
  analogWriteResolution(8);

  PCC_Relay = true;
  digitalWrite(24, PCC_Relay);
  delay(500);

  PCC_Relay = true;
  uart_TX();
  delay(5000);
  PCC_Relay = false;
  uart_TX();
  delay(500);
  digitalWrite(24, PCC_Relay);
}

//---------------------------------------------------------------------------------------
void loop()
{
  uart_RX();
  if(millis() - Timer_50 >= 50)
  {
    Timer_50 = millis();

    fan_ctrl();
    track_peaks();
    read_sensors();

  }
  if(millis() - Timer_250 >= State_Interval)
  {
    Timer_250 = millis();
    uart_TX();
    manage_sim_state();
    analogWrite(6, tunnel_setting);
    if(PCCOMS) pc_coms();
    //Serial.println(micros() - ts);
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
      + "," + theta_pos
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
        
        set_load(maxLoad);
        alpha = minAlpha;
        set_theta(maxTheta);
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
        timeT = timeWS;
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
          if(resistance - incLoad >= minLoad)
          {
            resistance -= incLoad;
            set_load(resistance);
          }
          else
          {
            resistance = maxLoad;
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
          timeT += timeAlpha;
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
          timeT += timeTheta;
          if(theta - incTheta >= minTheta)
          {
            set_theta(theta - incTheta);
            TestState = StepLoad;
          }
          else
          {
            set_theta(maxTheta);
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
          timeT += timeWS;
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
        set_theta(Serial.parseFloat());
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

        Serial.print("(t) Theta (0 - 95): ");
        Serial.println(0.0319*theta_pos - 6.6379);

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
void set_theta(float t)
{
  theta = t;
  if(t > 95.0) theta = 95.0;
  if(t < 0) theta = 0;
  theta_pos = (int)(31.3479*theta + 208.084);
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
  Serial1.write(highByte(theta_pos));           //Theta
  Serial1.write(lowByte(theta_pos));           //Theta
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
