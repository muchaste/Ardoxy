/*
  Ardoxy example

  Measure DO.
  Oxygen sensor is calbrated using the Pyro Oxygen Logger Software.
  Oxygen probe is connected to channel 1.

  You must define the following variables:
  - sampInt - Sample interval in ms
  - channel - channel on FireSting meter where oxygen sensor is connected
  - pin numers
  
  The circuit:
  - Arduino Uno
  - FireStingO2 - 7 pin connector:
    *Pin 1 connected to Arduino GND
    *Pin 2 connected to Arduino 5V
    *Pin 4 connected to Arduino RX (here: 8)
    *Pin 5 connected to Arduino TX (here: 9)

  The software:
  Download SerialPlot (https://hackaday.io/project/5334-serialplot-realtime-plotting-software)
  and use the configuration file (*.ini) from the Ardoxy github repository.
  Import the settings in SerialPlot using File>>Load Settings
  Or simply read the values from the serial monitor or use another serial logging software (PuTTy etc.)

  by Stefan Mucha

*/

#include <Ardoxy.h>
#include <SoftwareSerial.h>

// Set sampling interval in ms (due to the duration of the measurement and communication, use interval > 1000 msec)
unsigned long sampInt = 2000;

// Define variables
long DOInt, tempInt;                        // for measurement result
double DOFloat, tempFloat;                  // measurement result as floating point number
int check;                                  // numerical indicator of succesful measurement (1: success, 0: no connection, 9: mismatch)
char DOReadCom[11] = "REA 1 3 4\r";         // template for DO-read command that is sent to sensor
char tempReadCom[11] = "REA 1 3 5\r";       // template for temp-read command that is sent to sensor
bool startTrigger = false;                  // trigger for start of measurement
unsigned long loopStart, elapsed;           // ms timestamp of beginning and end of measurement loop


// Initiate connection via SoftwareSerial and create Ardoxy instance
SoftwareSerial mySer(8, 9);
Ardoxy ardoxy(mySer);

void setup() {
  Serial.begin(19200);
  delay(100);
  Serial.println("-------------- Ardoxy measure and plot example -------------");
  ardoxy.begin();
  Serial.println("FireSting channel: 1");
  Serial.print("Measurement interval (ms): ");
  Serial.println(sampInt);
  Serial.println("Use the SerialPlot software to plot DO and temperature");
  Serial.println("Send \"1\" to start measurement and \"0\" to end measurement.");
  Serial.println("------------------------------------------------------------");
}

void loop() {
  // wait for serial input to start measurement.
  if (Serial.available() > 0) {
    switch(Serial.read()){
      case '1':
          startTrigger = true;
          Serial.println("DO_air_sat;Temp_deg_C");
          break;
      case '0':
          startTrigger = false;
          break;
    }
  }

  if (startTrigger) {
    loopStart = millis();                     // get time at beginning of loop
    
    // measure sequence
    check = ardoxy.measureSeq(1);
    if(check == 1){
      DOInt = ardoxy.readout(DOReadCom);      // read DO value from results register
      delay(20);
      tempInt = ardoxy.readout(tempReadCom);  // read temperature value from results register
      delay(20);
      DOFloat = DOInt / 1000.00;              // convert to floating point number
      tempFloat = tempInt / 1000.00;          // convert to floating point number
      Serial.print(DOFloat);                  // print to serial
      Serial.print(";");
      Serial.println(tempFloat);
      elapsed = millis()-loopStart;
      delay(sampInt - elapsed);          // wait for next loop iteration
    }
    else {
      Serial.println("Com error.");
      Serial.println("Check connection with FireSting and send \"1\" to restart measurement.");      
      startTrigger = false;
    }
  }
}
