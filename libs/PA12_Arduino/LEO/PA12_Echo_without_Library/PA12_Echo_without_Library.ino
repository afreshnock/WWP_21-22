#define mySerial Serial1
 
const int EnablePin = 2;     // the number of the transmition enable pin
byte PA12_ID=0;

void setup() {
  
  // initialize both serial ports:
  Serial.begin(9600);    // Monitoring PORT
  mySerial.begin(57600);  //PA12 Network
  
  pinMode(EnablePin, OUTPUT); // Transmition enable pin is set
  
}

void loop() {
  digitalWrite(EnablePin, HIGH); // Transmition mode enable
  delay(1);     
  mySerial.write(0xff);  mySerial.write(0xff);  mySerial.write(0xff); // Packet Header
  mySerial.write(PA12_ID); //Servo ID
  mySerial.write(0x02);         //Packet Size
  mySerial.write(0xF1);         // Echo Command
  mySerial.write(0x0C);         // Checksum
  mySerial.flush();             // Waiting to transmit packet
  digitalWrite(EnablePin, LOW); // Transmition mode disable
  delay(1);
  
  while(!mySerial.available());  //Wating to receive packet
  while (mySerial.available()) {
  // read from port 1, send to port 0:
  if (mySerial.available()) {
    int inByte = mySerial.read();
    Serial.print(inByte); Serial.print(" ");
  }
  }
  mySerial.println(" ");
  delay(1000);
}
