/*
  Ardoxy example - manual communication with FireSting
  This example does not use the Ardoxy library, 
  it relays serial commands directly to a connected FireSting.
  Newer FireSting devices use a baudrate of 115200 instead of 19200
  
  The circuit:
  - Arduino UNO
  - FireStingO2 - 7 pin connector:
    *Pin 1 connected to Arduino GND
    *Pin 2 connected to Arduino 5V 
    *Pin 4 connected to Arduino SoftwareSerial RX (here: 8)
    *Pin 5 connected to Arduino SoftwareSerial TX (here: 9)

  Usage:
  Upload to Arduino UNO and send commands to FireSting via the Serial Monitor.
  Echoes from the FireSting are printed to the Monitor.
  Enter commands from the communication protocol:
  * "VER 1" reads out firmware version
  * "MSR 1" triggers DO measurement on Channel 1
  * "TMP 1" triggers temp. measurement
  * "REA 1 3 4" reads out DO as % air saturation * 1000
  * "REA 1 3 5" reads out temperature from ext. sensor as °C * 1000
  * "REA 1 3 6" reads out temperature from int. sensor as °C * 1000

  created 19 September 2024
  by Stefan Mucha
*/

#include <SoftwareSerial.h>

SoftwareSerial softSerial(8, 9);      // second serial port for communication with FireSting (RX and TX pins)
char input;                           // holds input from serial monitor (single characters)
const byte numChars = 64;             // arbitrary max. number of characters per transmission
char receivedChars[numChars];         // holds received data from FireSting
boolean newData = false;              // logical indicator for received serial data

void setup(){
     softSerial.begin(19200);         // initiate serial communication with FireSting
     Serial.begin(19200);             // initiate serial communication with PC
}

void loop(){
  while(Serial.available() > 0){
    input = Serial.read();            // store incoming string in inData
    softSerial.write(input);          // send command to FireSting
  }
  recvWithEndMarker();                // function receives serial data from FireSting
  showNewData();                      // function outputs echo to serial monitor
}

void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\r';
  char rc;
  while (softSerial.available() > 0 && newData == false) {
    rc = softSerial.read();
    
    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    }
    else {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData = true;
    }
  }
}

void showNewData() {
 if (newData == true) {
  Serial.println(receivedChars);
  newData = false;
 }
}
