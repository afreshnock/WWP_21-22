#include <SoftwareSerial.h>

#define LATCH_PIN B00100000          //D5
#define CLOCK_PIN B01000000          //D6
#define SERIAL_PIN B10000000         //D7
#define COMMON_RELAY_PIN B00000010   //D9

#define CURRENT_PIN A0
#define VOLTAGE_PIN A1

#define SOFT_RX_PIN 9
#define SOFT_TX_PIN 10

#define SAMPLE_TIME 100
#define BURST_TIME 10
#define COUNT 5

#define IN 1
#define OUT 0

uint8_t counter, load_val;

enum States {Start, Optimize, Regulate, Safety_Disconnect, Safety_Switch};
States State;

double v_sum, i_sum, L_voltage, L_current, L_power;
double T_voltage;

word RPM;

unsigned long sample_timer, burst_timer;


SoftwareSerial SoftSerial(SOFT_RX_PIN, SOFT_TX_PIN, true);

void setup()
{
  //setup Ports
  DDRD = B11110001;       //7,6,5,4,3,2,1,0 1 output, 0 input
  DDRC = B00000000;       //analog Port all input
  DDRB = B00000010;       //13,12,11,10,9,8 1 output, 0 input

  //begin serial comms
  Serial.begin(115200);
  SoftSerial.begin(9600);
  pinMode(SOFT_RX_PIN, INPUT_PULLUP);

  //initialize all variables
  L_voltage = 0;
  L_current = 0;
  L_power = 0;
  v_sum = 0;
  i_sum = 0;
  counter = 0;
  load_val = 255;
  State = Start;

  //initialize board component state
  set_Load(load_val);
  set_Common_Relay(IN);

  //initialize timers
  sample_timer = millis();
  burst_timer = millis();
}

void loop()
{
  soft_Serial_Rec();
  sample_Data();

  Serial.print("L_V: ");
  Serial.print(L_voltage, 3);
  Serial.print("\tT_V: ");
  Serial.print(T_voltage, 3);
  Serial.print("\tL_I: ");
  Serial.print(L_current, 3);
  Serial.print("\tP: ");
  Serial.print(L_power, 3);
  Serial.print("\tRPM: ");
  Serial.print(RPM);
  Serial.println("\n");

  if(Serial.available())
  {
    uint8_t val = Serial.read();

    Serial.print("\nLoad Resistance: ");
    Serial.print(val * 0.062745098, 3);
    Serial.println("\n");

    set_Load(val);
  }
  
//  sample_Data();
//  set_Load(load_val);
//
//  switch (State)
//  {
//    case Start:
//
//      load_val = 255;
//
//      /*
//       * Upon starting (in setup()) the turbine-side 
//       * board should print a couple '1' characters to
//       * indicate that is in fact up and running. This,
//       * along with power reaching a minimum threshold,
//       * will trigger the transistion out of the Start
//       * state.  
//       */
//       
//      if (SoftSerial.read() == '1' && L_power > 0.250)
//      {
//        //update state
//        State = Optimize;
//        
//        //flush software serial buffer
//        while(SoftSerial.available())
//        {
//          SoftSerial.read();
//        }
//      }
//      else
//      {
//        State = Start;
//      }
//
//      break;
//
//    case Optimize:
//
//      //functions for determining load_val
//      //set_Load(load_val);
//
//      //if turbine and load voltage differe by more than 1V
//      //but communications are still up, load has been
//      //disconnected. Optimize -> Safety
//
//      break;
//
//    case Regulate:
//      break;
//
//    case Safety_Disconnect:
//      break;
//
//    case Safety_Switch:
//      break;
//
//    default:
//      break;
//  }

}

void shift_Out(uint8_t latch_pin, uint8_t clock_pin, uint8_t serial_pin, uint8_t data)
{
  PORTD &= ~latch_pin & ~serial_pin;
  uint8_t count = 0;

  while (count < 8)
  {
    PORTD = (data & 0x01) ? PIND | serial_pin : PIND & ~serial_pin;
    PORTD |= clock_pin;
    data >>= 1;
    count++;
    PORTD &= ~clock_pin;
  }
  PORTD |= latch_pin;
}

bool set_Load(uint8_t val)
{
  if (val >= 0 && val <= 255)
  {
    shift_Out(LATCH_PIN, LATCH_PIN, SERIAL_PIN, val);
    return true;
  }
  else
  {
    return false;
  }
}

double get_Voltage()
{
  unsigned int V = analogRead(VOLTAGE_PIN);
  return (0.0198 * V + 0.045);
}

double get_Current()
{
  unsigned int I = analogRead(CURRENT_PIN);
  return (9.0393 * I - 103.831) / 1000;
}

void sample_Data()
{
  if (millis() - sample_timer >= SAMPLE_TIME)
  {
    if (millis() - burst_timer >= BURST_TIME)
    {
      burst_timer = millis();

      //alternate measurement
      if (counter % 2)
      {
        v_sum += get_Voltage();
      }
      else
      {
        i_sum += get_Current();
      }
      counter++;
    }
    if (counter >= (2 * COUNT))
    {
      //calculate averages
      L_voltage = 2 * v_sum / counter;
      L_current = 2 * i_sum / counter;
      L_power = L_voltage * L_current;

      //reset values
      v_sum = 0;
      i_sum = 0;
      counter = 0;
      sample_timer = millis();
    }
  }
}

void set_Common_Relay(uint8_t val)
{
  PORTB = val ? PINB | COMMON_RELAY_PIN : PINB & ~COMMON_RELAY_PIN;
}

void soft_Serial_Rec()
{
  if (SoftSerial.available() >= 6)
  {
    if (SoftSerial.read() == '0')
    {
      uint16_t temp_v = SoftSerial.read() | (SoftSerial.read() << 8);
      uint16_t temp_rpm = SoftSerial.read() | (SoftSerial.read() << 8);

      if (SoftSerial.read() == '6')
      {
        T_voltage = double(temp_v) / 1000;
        RPM = temp_rpm;
      }
      else
      {
        while (SoftSerial.available())
        {
          SoftSerial.read();
        }
      }
    }
    else
    {
      while (SoftSerial.available())
      {
        SoftSerial.read();
      }
    }
  }
}
