#include "src/INA260/Adafruit_INA260.h"
//Local libraries need to either be directly in the project folder, or in the src folder.
//src linker only works in Arduino IDE 1.5+ I believe.
#include "src/WP_SD.h"

#define FILTER_LENGTH 10

Adafruit_INA260 ina260 = Adafruit_INA260();

enum States {Wait, Normal, Regulate, Safety1, Safety2_Entry, Safety2_Wait};
enum OptStates {TWait, PInit, PCheck, RCtrl, PCtrl, TSS};
OptStates OState = TWait;
OptStates NextOState = RCtrl;
States State = Wait;

//Preset Go-to resistances
float cutin_r = 64;
float med_r = 2.5;
float maxpwr_r = 1.5;

//Preset Go-to theta angles
float cutin_t = 38.0221;
float revive_t = 20.0;
float maxpwr_t = 18.8821;
float brake_t = 95;
float reg_t = 38.0221;

//Preset alpha angles
uint8_t norm_a = 0;
uint8_t pwrreg_a = 30;

uint16_t min_pitch_pwr = 800;
uint16_t L_Voltage_Target = 3333;

uint16_t k1 = 1;
uint16_t k2 = 1;
uint16_t k3 = 1;
uint16_t thresh = 3800;



//Load Variables
uint16_t L_Voltage_raw;
uint16_t L_Power;   //Load Power (mW)
uint16_t L_Power_last = 14000;
uint16_t L_Power_raw;   //Load Power (mW)
uint16_t L_Voltage; //Load Voltage (mV)
uint16_t L_Current; //Load Current (mA)

//Turbine Variables
uint16_t T_Power;   //Turbine Power (mW)
uint16_t T_Voltage; //Turbine Power (mW)
uint16_t RPM;       //Turbine RPM   (r/min)
uint16_t RPM_last;       //Turbine RPM   (r/min)
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

//IDK Variables
uint16_t Peak_Power;  //(mW)
uint16_t Peak_RPM;    //(r/min)

unsigned long Timer_Fast;
unsigned long Timer_Medium;
unsigned long Timer_Slow;
unsigned long Timer_Log;
unsigned long Timer_Wait;
unsigned long Comms_Timeout;
unsigned long Timer_Transient;


unsigned Fast_Interval = 1;
unsigned Medium_Interval = 50;
unsigned Slow_Interval = 250;
unsigned Log_Interval = 25;
unsigned Wait_Interval;
unsigned Pitch_Transient = 1500;
unsigned Resistance_Transient = 250;
unsigned Transient_Interval;

bool Turbine_Comms;
bool PCC_Relay;
bool Turbine_PCC_Relay;
bool Pitch_Enable;
bool Startup = true;

//---------------------------------------------------------------------------------------
void setup()
{
  init_pins();
  init_coms();
  init_timers();

  set_load(cutin_r);

  set_theta(cutin_t);
  alpha = norm_a;

  PCC_Relay = true;
  digitalWrite(24, PCC_Relay);
  delay(500);

  PCC_Relay = true;
  uart_TX();
  delay(7000);
  PCC_Relay = false;
  uart_TX();
  delay(500);
  digitalWrite(24, PCC_Relay);

  analogWriteFrequency(6, 200000);
  analogWriteResolution(8);
  Wait_Interval = 10000;

}

//---------------------------------------------------------------------------------------
void loop()
{
  uart_RX();

  if (millis() - Timer_Fast >= Fast_Interval)
  {
    Timer_Fast = millis();
    
    fan_ctrl();
    track_peaks();
    read_sensors();
    manage_state();
  }
  if (millis() - Timer_Medium >= Medium_Interval)
  {
    Timer_Medium = millis();
    
    uart_TX();
  }
  if (millis() - Timer_Slow >= Slow_Interval)
  {
    Timer_Slow = millis();
    
    analogWrite(6, tunnel_setting);
    pc_coms();
  }

  if (millis() - Timer_Log >= Log_Interval)
  {
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
                 + "," + Turbine_Comms
                );
  }
}

//---------------------------------------------------------------------------------------
void manage_state()
{
  switch (State)
  {
    case Wait:
      //If load recieves data from turbine, enter normal operation
      if (millis() - Timer_Wait >= Wait_Interval)
      {
        if (Turbine_Comms)
        {
          State = Normal;
          if (SDConnected && !Logging)
          {
            toggle_Logging();
          }
        }
      }
      break;

    case Normal:
      //Emergency switch condition
      if (E_Switch)
      {
        //Move to Safety1
        State = Safety1;
      }
      //Discontinuity Condition
      if ((L_Voltage < (T_Voltage * 0.5)) && (RPM >= 400))
      {
        //Move to Safety2
        State = Safety2_Entry;
      }
      if (RPM > 3000)
      {
        State = Regulate;
      }
      optimize_3_3();


      break;

    case Regulate:
      //Emergency switch condition
      if (E_Switch)
      {
        //Move to Safety1
        State = Safety1;
      }
      //Discontinuity Condition
      if ((L_Voltage < (T_Voltage * 0.5)) && (RPM >= 100))
      {
        //Move to Safety2
        State = Safety2_Entry;
      }
      //Regulate RPM at 11m/s val (PID? keep at val)

      if (RPM > 3000)
      {
        set_theta( theta = - 0.1); // to be optimized
        set_load(med_r);
        if (RPM > 4000)
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

      break;

    case Safety1:
      //do safety1 stuff
      set_theta(brake_t);
      PCC_Relay = true;

      //Emergency switch condition
      if (!E_Switch)
      {
        //Move to Safety1
        set_theta(revive_t);
        set_load(cutin_r);
        //Optimize for power
        if (RPM >= 800 && PCC_Relay)
        {
          PCC_Relay = !PCC_Relay;
          Timer_Wait = millis();
          Wait_Interval = 1000;
          State = Wait;
          set_load(40);
        }
      }
      break;

    case Safety2_Entry:
      //Do safety2 stuff?
      set_theta(brake_t);
      PCC_Relay = true;
      if (!Turbine_Comms)
      {
        State = Safety2_Wait;
      }
      break;

    case Safety2_Wait:
      if (Turbine_Comms)
      {
        set_theta(revive_t);
        set_load(cutin_r);
        if (RPM >= 800 && PCC_Relay)
        {
          PCC_Relay = !PCC_Relay;
          Timer_Wait = millis();
          Wait_Interval = 1000;
          State = Wait;
          set_load(40);
        }
      }
      break;

    default:
      State = Wait;
      break;
  }
  digitalWrite(24, PCC_Relay);
}

//---------------------------------------------------------------------------------------
void optimize_3_3()
{
  static float sign = -1;
  static float pitch_coef = 5.0;
  static int r_priority = 3;
  static int r_iter = 0;
  static float rt;
  static uint16_t SS_RPM;
  switch(OState)
  {
    case TWait:
      //Wait state to allow other states to jump into for transiets
      if (millis() - Timer_Transient >= Transient_Interval)
      {
        Timer_Transient = millis();
        OState = NextOState; //can specify next state in previous state
      }
      break;

    case RCtrl:
      if(abs(L_Voltage - L_Voltage_Target >= 10))
      {
        Resistance_Transient = 64000/(abs(L_Voltage - L_Voltage_Target)*resistance)+FILTER_LENGTH;
      }
      if(L_Voltage < 0.95*L_Voltage_Target || L_Voltage > 1.05*L_Voltage_Target)
      {
        rt = (L_Voltage > L_Voltage_Target) ? -0.25 : 0.25;
        set_load(resistance + rt); // to be optimized
        Pitch_Enable = false;
        Transient_Interval = Resistance_Transient;
        r_iter++;
        NextOState = RCtrl; //come back here after transiet
      }
      if((L_Voltage >= 0.95*L_Voltage_Target && L_Voltage <= 1.05*L_Voltage_Target) || resistance == 1)// || r_iter >= r_priority)
      {
        Pitch_Enable = true;
      }
      if(Pitch_Enable && T_Power > 600) //pitch can be changed and we have enough power
      {
        NextOState = PInit; //go to change pitch after transient
        Transient_Interval = 500;
        //Transient_Interval = 10;
      }
      OState = TWait; //always wait for transient
      break;

    case PInit:
      RPM_last = RPM;
      OState = PCtrl;
      Serial.println("PInit");
      break;

    case PCtrl:
      Serial.println("PCtrl");
      Serial.println(sign);
      set_theta(theta + sign*pitch_coef);
      Transient_Interval = Pitch_Transient;
      OState = TWait;
      NextOState = PCheck;
      break;

    case PCheck:
      Serial.println("PCheck");
      if (RPM_last > RPM)
      {
        sign = -sign;
        pitch_coef = pitch_coef / 2.0;
      }
      else
      {
        sign = sign;
      }
      if(pitch_coef < .01)
      {
        pitch_coef = 0;
        SS_RPM = RPM;
        OState = TSS;
      }
      else
      {
      OState = TWait;
      }
      NextOState = RCtrl;
      Transient_Interval = 1;
      break;

    case TSS:
      if(0.9*SS_RPM > RPM || 1.1*SS_RPM < RPM)
      {
        pitch_coef = 5;
        OState = TWait;
        Transient_Interval = 2000;
      }
      
      break;
    default:
      OState = TWait;
      NextOState = RCtrl;
      Transient_Interval = 1;
      break;

  }
}

//---------------------------------------------------------------------------------------
void pc_coms()
{
  if (Serial.available() > 0)
  {
    uint8_t cmd = Serial.read();

    switch (cmd)
    {
      case 's':
        if (SDConnected)
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
        set_theta(Serial.parseFloat());
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

  //---------------------------------------------------------------------------------------
  if (Serial) // check performance cost on checking if serial is active
  {
    Serial.print("Turbine Connected: ");
    Serial.println(Turbine_Comms);

    Serial.print("PCC Relay: ");
    Serial.println(PCC_Relay);

    Serial.print("Pitch Enable: ");
    Serial.println(Pitch_Enable);

    Serial.print("RPM: ");
    Serial.println(RPM);

    Serial.print("Load Voltage: ");
    Serial.print(L_Voltage);
    Serial.println(" mV");

    Serial.print("Load Voltage (unf): ");
    Serial.print(L_Voltage_raw);
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
    switch (State)
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

      case Safety2_Entry:
        Serial.println("Safety2 Entry");
        break;

      case Safety2_Wait:
        Serial.println("Safety2 Wait");
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
    Serial.println(0.0319 * theta_pos - 6.6379);

    Serial.print("(r) Load ( 0- 64 ): ");
    Serial.print((float)load_Val / 255 * 63.75);
    Serial.println(" Ohms");

    Serial.println("(1)-k1 / (2)-k2 / (3)-k3 / (h)-thressh: ");
    Serial.println((String) k1 + ", " + k2 + ", " + k3 + ", " + thresh);

    //Serial.println("RPM + k1*theta + k2*alpha + k3*load_Val > thresh");
    Serial.print("Overspeed Condition: ");
    Serial.print(RPM + k1 * theta + k2 * alpha + k3 * load_Val);
    Serial.print(" > ");
    Serial.println(thresh);

    Serial.print("(s) Logging: ");
    if (Logging) {
      Serial.println("True");
    }
    else {
      Serial.println("False");
    }

    Serial.println();
  }
}

//---------------------------------------------------------------------------------------
void set_windspeed(double ws)
{
  windspeed = ws;
  double temp = (0.303 + 18.7 * windspeed + -0.67 * pow(windspeed, 2) + 0.0317 * pow(windspeed, 3));
  if (temp > 255) {
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
  Timer_Medium = millis();
  Timer_Slow = millis();
  Timer_Log = millis();
  Timer_Transient = millis();
}

//---------------------------------------------------------------------------------------
void read_sensors_filtered()
{
  //L_Voltage = ina260.readBusVoltage();
  L_Voltage_raw = ina260.readBusVoltage();
  L_Power_raw = ina260.readPower();
  L_Current = ina260.readCurrent();
  filter_vars();
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
  if (r > 63.75) resistance = 63.75;
  if (r < 1) resistance = 1;
  load_Val = (int)(resistance * 4);
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
  if (t > 95.0) theta = 95.0;
  if (t < 5) theta = 5;
  theta_pos = (int)(31.3479 * theta + 208.084);
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
  if (millis() - Comms_Timeout >= 2 * Slow_Interval)
  {
    Comms_Timeout = millis();
    Turbine_Comms = false;
    RPM = 0;
    T_Power = 0;
    T_Voltage = 0;
  }
} //hello

//---------------------------------------------------------------------------------------
void filter_vars()
{
  static int index = 0;
  static long L_V_raw_Samples[FILTER_LENGTH];
  static long L_V_sum = 0;
  static long L_P_raw_Samples[FILTER_LENGTH];
  static long L_P_sum = 0;


  L_V_sum = L_V_sum + L_Voltage_raw - L_V_raw_Samples[index];
  L_V_raw_Samples[index] = L_Voltage_raw;

  L_P_sum = L_P_sum + L_Power_raw - L_P_raw_Samples[index];
  L_P_raw_Samples[index] = L_Power_raw;

  index = ( index + 1 ) % FILTER_LENGTH;

  L_Voltage = L_V_sum / FILTER_LENGTH;
  L_Power = L_P_sum / FILTER_LENGTH;
}
