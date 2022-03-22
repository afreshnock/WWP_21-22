#include <Wire.h>
/*
   This code is meant for arduino micro


*/

// sda pin 2

volatile unsigned long alpha = 0;
volatile unsigned long phase = 30;
unsigned long Timer;
volatile byte StageNumber;
char SerialRead1;
char SerialRead2;
//const char Stages[6] = {0xC0, 0x60, 0x30, 0x12, 0x03, 0x81};
//23-20 23-18 18-21 21-22 22-19 19-20 0b7654xx10 18 = 7 23 = 0
const char Stages[6] = {~0b10100000, ~0b10000001, ~0b00010001, ~0b00010010,~0b01000010, ~0b01100000}; //0b7654xx10
volatile unsigned long StageChange = 60;
volatile unsigned int i2cInput, i2cInput1, i2cInput10;
volatile byte n = 0;
volatile int i = 0;
//volatile byte CapLock = 0;

unsigned int avg1, avg2, avg3, avg4;
//unsigned int avg5, avg6, avg7, avg8;

void setup() {
  Serial.begin(115200);
  Wire.begin(0x35);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(alphaSend);

  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);
  pinMode(20, OUTPUT);
  pinMode(21, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  pinMode(13, INPUT);
  cli();                      //stop interrupts for till we make the settings

  TCCR1A = 0;                 // Reset entire TCCR1A to 0
  TCCR1B = 0;                 // Reset entire TCCR1B to 0
  TCCR1B = B00001001;        //Set CS12 to 1 so we get prescalar 8 - count up to OCR1 register
  TIMSK1 = B00000010;        //Set OCIE1A to 1 so we enable compare match A
  //OCR1AH = 0x00;             //Finally we set compare register A to this value
  OCR1A = 0x0FFF;

  TCCR3A = 0;// set entire TCCR1A register to 0
  TCCR3B = 0; // sets to mode 12 counting up until ICP3 RE
  TCCR3A = 0b00000000;
  TCCR3B = 0b11000100; // noise cancel off, rising edge, normal mode. 1024 prescaler
  TCCR3C = 0;      // normal mode
  TIMSK3 = 0b00100000;
  TCNT3  = 0;// initialize counter value to 0

  sei();                     //Enable back the interrupts
  PORTF &= 0x00;

}

void receiveEvent() {
  while (Wire.available()) {
    if (!n) {
      if (Wire.read() == 'a') {
        n = 1;
        //Serial.print("(a0)");
      }
    }
    else if (n == 1) {
      i2cInput10 = Wire.read();
      if (i2cInput10 == 'a')
      {
        n = 1;
        //Serial.print("(a1)");
      }
      else
      {
        i2cInput10 = 10 * (i2cInput10 - 48);
        n = 2;
        //Serial.print("t");
      }
    }
    else if (n == 2) {
      i2cInput1 = Wire.read();
      if (i2cInput1 == 'a')
      {
        n = 1;
        //Serial.print("(a2)");
      }
      else
      {
        //Serial.print("o");
        //if ones!=5 or ones!=0 --> throw it away
        i2cInput = i2cInput10 + (i2cInput1 - 48);
        Serial.println(i2cInput);
        n = 0;
        if (i2cInput <= 90) {
          alpha = i2cInput;
        }
      }
    }
  }
}

void alphaSend() {
  Serial.print("REQUEST = ");
  Serial.println(alpha);
  if (alpha < 10){
    Wire.write('a');
    Wire.write('0');
    Wire.write(alpha);
  }
  else{
    Wire.write('a');
    Wire.write(alpha);
  }
}

void loop() {

}

ISR(TIMER1_COMPA_vect) { //Output
  //TCNT1  = 0;                  //First, set the timer back to 0 so it resets for next interrupt
  phase = phase + 5;
  if (phase >= StageChange) {
    PORTF = Stages[StageNumber];
    StageNumber += 1;
    StageChange += 60;
    if (StageNumber >= 6) {
      StageNumber = 0;
      StageChange = 60;
    }
  }
  if (phase >= 360) {
    phase -= 360;
  }
}

ISR(TIMER3_CAPT_vect) // Input Capture from hall effect
{
  i = ICR3;
  avg4 = avg3;
  avg3 = avg2;
  avg2 = avg1;
  avg1 = i;
  i = (avg1 + avg2 + avg3 + avg4) / 4;


  //CapLock = 1;
  if (i >= 40) {

    //Serial.println(i);
    OCR1A = (float) i * 3.555; //14.222 is 1024 // 3.555 is 256
    phase = alpha + 30;
    StageChange = 60;
    StageNumber = 0;
    if (alpha > 60) {
      StageChange = 120;
      StageNumber = 1;
    }
  }
  TCNT3 = 0x0000;        // restart timer for next revolution
  //TCNT1 &= 0x0000;
}
