/*
  Ardoxy example - manual communication with FireSting
  This example does not use the Ardoxy library, 
  it relays serial commands directly to a connected FireSting.
  
  The circuit:
  - Arduino Mega
  - FireStingO2 - 7 pin connector:
    *Pin 1 connected to Arduino GND
    *Pin 2 connected to Arduino 5V 
    *Pin 4 connected to Arduino RX (here: 19)
    *Pin 5 connected to Arduino TX (here: 18)

  Usage:
  Upload to Arduino MEGA and send commands to FireSting via the Serial Monitor.
  Echoes from the FireSting are printed to the Monitor.
  Enter commands from the communication protocol:
  * "VER 1" reads out firmware version
  * "MSR 1" triggers DO measurement on Channel 1
  * "TMP 1" triggers temp. measurement
  * "REA 1 3 4" reads out DO as % air saturation * 1000
  * "REA 1 3 5" reads out temperature from ext. sensor as °C * 1000
  * "REA 1 3 6" reads out temperature from int. sensor as °C * 1000

  created 09 February 2022
  by Stefan Mucha
*/


char input;                           // holds input from serial monitor (single characters)
const byte numChars = 64;             // arbitrary max. number of characters per transmission
char receivedChars[numChars];         // holds received data from FireSting
boolean newData = false;

void setup(){
     Serial.begin(19200); 
     Serial1.begin(19200);
}

void loop(){
  while(Serial.available() > 0){
    input = Serial.read();           // store incoming string in inData
    Serial1.write(input);              // send command to mySer
  }
  recvWithEndMarker();                // function receives serial data from FireSting
  showNewData();                      // function outputs echo to serial monitor
}


void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\r';
  char rc;
  while (Serial1.available() > 0 && newData == false) {
    rc = Serial1.read();
    
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
