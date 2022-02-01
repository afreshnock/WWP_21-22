#include <SoftwareSerial.h>

#define SAFETY_PIN 2
#define RPM_PIN 3
#define P13A B00010000
#define P13B B00100000
#define P12A B01000000
#define P12B B10000000
#define SOFT_RX 8
#define SOFT_TX 9
#define VOLTAGE_PIN A0
#define COMMON_RELAY_PIN_A B10000000
#define COMMON_RELAY_PIN_B B01000000

#define ON true
#define OFF false
#define OUT true
#define IN false
#define TRANSMIT_TIME 500
#define SAMPLE_TIME 200
#define BURST_TIME 10
#define RPM_TIME 200
#define NOTCHES 4
#define COUNT 5
#define RPM_SAMPLES 5

SoftwareSerial SoftSerial(SOFT_RX, SOFT_TX);

bool Safety_Flag;

uint8_t counter, samples;

uint16_t T_voltage, RPM;

uint32_t transmit_timer, sample_timer, burst_timer, rpm_timer, v_sum, FP_on, FP_off, FP_sum;

void setup()
{
  //Setup all ports and INPUTS or OUTPUTS
  DDRD = B11110010; //Digital Pins 7-0  1 - OUTPUT, 0 - INPUT
  DDRC = B00111100; //Analog Pins 7-0   1 - OUTPUT, 0 - INPUT
  DDRB = B00101110; //Digital Pins 13-8 1 - OUTPUT, 0 - INPUT

  //Attach interrupt to SAFETY_PIN
  attachInterrupt(digitalPinToInterrupt(SAFETY_PIN), safety_ISR, CHANGE);

  EMF_Brake(OFF);

  SoftSerial.begin(9600);
  Serial.begin(115200);

  Safety_Flag = false;
  counter = 0;
  samples = 0;
  T_voltage = 0;
  RPM = 0;
  FP_on = 0;
  FP_off = 0;
  FP_sum = 0;
  v_sum = 0;

  sample_timer = millis();
  burst_timer = 0;
  rpm_timer = millis();
  transmit_timer = millis();
}

void loop()
{
  sample_Data();

}

void EMF_Brake(bool var)
{
  /*
     If TRUE is passed in this function,
     the EMF Brake is engaged, shorting
     the three phases of the generator.

     If FALSE is passed in this function,
     the EMF Brake is disengaged, opening
     the three phases of the generator.
  */

  if (var)
  {
    PORTD |= (P13B | P12B);
    delay(30);
    PORTD &= ~(P13B | P12B);
  }
  else
  {
    PORTD |= (P13A | P12A);
    delay(30);
    PORTD &= ~(P13A | P12A);
  }
}

void safety_ISR()
{
  /*
     This Interrupt Service Routine is executed
     when a change is detected on the SAFETY_PIN
     and will set Safety_Flag TRUE if the pin is
     HIGH and FALSE if it is LOW.
  */

  Safety_Flag = PIND & B00000100 ? true : false;
}

uint8_t transmit_Data()
{
  /*
     This function transmits a packet of data to
     the Load-Side Board via Software Serial. It
     will transmit data every TRANSMIT_TIME ms.
     The function will also return the number of
     byte sent (6 if all went well).
  */

  if (millis() - transmit_timer >= TRANSMIT_TIME)
  {
    transmit_timer = millis();
    uint8_t bytes_sent = 0;

    bytes_sent += SoftSerial.write('0');
    bytes_sent += SoftSerial.write(T_voltage);
    bytes_sent += SoftSerial.write(T_voltage >> 8);
    bytes_sent +=  SoftSerial.write(RPM);
    bytes_sent +=  SoftSerial.write(RPM >> 8);
    bytes_sent += SoftSerial.write('6');

    return bytes_sent;
  }
}

word get_Voltage()
{
  word V = analogRead(VOLTAGE_PIN);
  return (1000 * (0.0196 * V + 0.0936));
}

void sample_Data()
{
  if (millis() - sample_timer >= SAMPLE_TIME)
  {
    if (millis() - burst_timer >= BURST_TIME)
    {
      burst_timer = millis();
      v_sum += get_Voltage();
      counter++;
    }
    if (counter >= (COUNT))
    {
      //calculate averages
      T_voltage = v_sum / counter;

      //reset values
      v_sum = 0;
      counter = 0;
      sample_timer = millis();
    }
  }
}

void RPM_sample_data()
{
  if (millis() - rpm_timer <= RPM_TIME)
  {
    rpm_timer = millis();

    while (FP_on == 0 && FP_off == 0)
    {
      FP_off = pulseIn(RPM_PIN, LOW);
      FP_on = pulseIn(RPM_PIN, HIGH);
    }
    samples++;

    FP_sum += (FP_on + FP_off);
    FP_on = 0;
    FP_off = 0;

    if (samples >= RPM_SAMPLES)
    {
      RPM = 1.56 * (NOTCHES * 60000000) / (samples * FP_sum);
      FP_sum = 0;
      samples = 0;

      RPM = RPM < 10000 ? RPM : 0;
    }
  }
}

#define pulse_ip 3 // read pin
const int Notches = 4; // once mechanical tells us how many notches they have, this can be hard coded instead of a variable
const int SamplesToAverage = 1;
unsigned long TimeOn[Notches];
unsigned long TimeOff[Notches];
unsigned long AVGRPM; // exported value
unsigned long timer; //universal timer

unsigned long freq, period; //
//float RPM[Notches]; //holds the readings from each notch
int n; // dummy variable to measure [Notches] amounts of times
int j = 0; // dummy variable to send out data every second


void RPMtest(){
  if (j < SamplesToAverage) {
    if (millis() - timer > 500) {
      for (n = 0; n < Notches; n++) { //read time spend high and low
        TimeOff[n] = pulseIn(pulse_ip, LOW);
        TimeOn[n] = pulseIn(pulse_ip, HIGH);
      }
      
      period = 0;
      for (n = 0; n < Notches; n++) { //sum up all of the notch timings
        period += TimeOff[n] + TimeOn[n];
      }

      freq = 60000000 / period; //get an RPM for the 1 revolution

      //Serial.print(freq);
      //Serial.print("rpm, \n");
      AVGRPM += freq; //add it to average
      j++; //increment for the while loop
      timer = millis();
    }
  }
  else if (AVGRPM < 8000000) {
    RPM = AVGRPM / SamplesToAverage;
    //Serial.println(AVGRPM); //print average
    AVGRPM = 0;
    j = 0;
  }
  else {
    RPM = 0;
    AVGRPM = 0;
    j = 0;
  }
}
