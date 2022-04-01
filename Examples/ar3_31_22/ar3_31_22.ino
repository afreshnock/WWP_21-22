#include <Wire.h>

/*

   This code is meant for arduino micro

*/

volatile byte alpha = 0;

volatile unsigned long phase = 0;

unsigned long Timer;

volatile byte StageNumber;

char SerialRead1;

char SerialRead2;

//const char Stages[6] = {0xC0, 0x60, 0x30, 0x12, 0x03, 0x81};

//23-20 23-18 18-21 21-22 22-19 19-20 0b7654xx10 18 = 7 23 = 0
//G1-G6 G6-G2 G2-G3 G3-G4 G4-G5 G5-G6

//G1-G6 G1-G2 G2-G3 G3-G4 G4-G5 G5-G6

// G2 G5 G6 G3 x x G4 G1

//const char Stages[6] = {~0b00010001, ~0b10000001, ~0b10100000, ~0b00010010, ~0b01000010, ~0b01100000}; //0b7654xx10
//const char Stages[6] = {~0b01000010, ~0b01100000, ~0b10100000, ~0b10000001, ~0b00010001, ~0b00010010};
//const char Stages[6] = {~0b00000000, ~0b00000000, ~0b10000001, ~0b10100000, ~0b00010001, ~0b00000000};
//$$$$$$$const char Stages[6] = {~0b00100001, ~0b10000001, ~0b10010000, ~0b00010010, ~0b01000010, ~0b01100000}; //good, we think
// drop G1 and G6 const char Stages[6] = {~0b00000000, ~0b10000000, ~0b10010000, ~0b00010010, ~0b01000010, ~0b01000000};
// drop G1 and G2 const char Stages[6] = {~0b00100000, ~0b00000000, ~0b00010000, ~0b00010010, ~0b01000010, ~0b01100000};
//const char Stages[6] = {~0b00000001, ~0b10000001, ~0b10010000, ~0b00010010, ~0b00000010, ~0b00000000};

const byte Stages[6] = {~0b01100000, ~0b00100001, ~0b10000001, ~0b10010000, ~0b00010010, ~0b01000010};

volatile unsigned long StageChange = 60;

volatile unsigned int i2cInput, i2cInput1, i2cInput10;

volatile byte n = 0;

volatile int i = 0;



unsigned int avg1, avg2, avg3, avg4;



void setup() {

  Serial.begin(115200);

  Wire.begin(0x08);

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

  OCR1A = 0xFFFF;



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

        //Serial.println("(a0)");

      }

    }

    else if (n == 1) {

      i2cInput = Wire.read();

      if (i2cInput == 'a') {

        n = 0;

        //Serial.print("(a1)");

      }

      else {

        n = 0;

        if (i2cInput <= 90) {

          alpha = i2cInput;

          //Serial.println(alpha);

        }

      }

    }

  }

}



void alphaSend() {

  //Serial.print("REQUEST = ");

  //Serial.println(alpha);

  Wire.write('a');

  Wire.write(alpha);

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



ISR(TIMER3_CAPT_vect) { // Input Capture from hall effect
  i = ICR3;

  avg4 = avg3;

  avg3 = avg2;

  avg2 = avg1;

  avg1 = i;

  i = (avg1 + avg2 + avg3 + avg4) / 4;



  if (i >= 40) {



    OCR1A = (float) i * 3.555; //14.222 is 1024 // 3.555 is 256

    phase = alpha + 55;

    StageChange = 60;

    StageNumber = 0;

    if (alpha > 60) {

      StageChange = 120;

      StageNumber = 1;

    }

  }

  TCNT3 = 0x0000;        // restart timer for next revolution
}