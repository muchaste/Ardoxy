/*
  Ardoxy example

  Regulate DO to a static setpoint with defined experimental duration using a solenoid valve.
  Oxygen sensor is calbrated using the Pyro Oxygen Logger Software.
  A flyback diode is connected to the +/- poles of the valve to protect the circuit from inductive charges

  You must define the following variables:
  - sampInt - Sample interval in ms
  - channel - channel on FireSting meter where oxygen sensor is connected
  - pin numers
  - experimental duration
  - DO setpoints for each channel
  - *optionally* adjust the PID tunings (Kp, Ki, Kd) for the PID control instance

  The circuit:
  - Arduino Uno
  - FireStingO2 - 7 pin connector X1:
    *Pin 1 connected to Arduino GND
    *Pin 2 connected to Arduino 5V 
    *Pin 4 connected to Arduino RX (here: 8)
    *Pin 5 connected to Arduino TX (here: 9)
  - Relay module
    *GND connected to Arduino GND
    *VCC connected to Arduino 5V
    *IN connected to Arduino digital pin (here: 3)

  The software:
  Download SerialPlot (https://hackaday.io/project/5334-serialplot-realtime-plotting-software)
  and use the configuration file (*.ini) from the Ardoxy github repository.
  Import the settings in SerialPlot using File>>Load Settings
  Or simply read the values from the serial monitor or use another serial logging software (PuTTy etc.)

  by Stefan Mucha

*/

#include <Ardoxy.h>
#include <SoftwareSerial.h>
#include <PID_v1.h>

//#######################################################################################
//###                              General settings                                   ###
//#######################################################################################

unsigned long sampInterval = 2000;                        // sampling interval in ms
unsigned int experimentDuration = 60;                     // total duration in min
const int channelNumber = 1;                              // number of measurement and control channels
const int channelArray[channelNumber] = {1};              // channels on the Firesting where the sensors are connected
const int relayPins[channelNumber] = {3};                 // pins for operation of the corresponding relays for each channel

// Define pins
const int RX = 8;                                         // RX pin for serial communication
const int TX = 9;                                         // TX pin for serial communication

// DO setpoints for each channel
double DOSetpoints[channelNumber] = {30};                 // DO as % air saturation

//#######################################################################################
//###                            Requisite Variables                                  ###
//#######################################################################################

// DO measurement
double DOFloat[channelNumber];                      // Floating point DO values for each channel
double tempFloat;                                   // measurement result as floating point number
const int closed = HIGH;                            // marks the output on the relay pin, which should close the valve
const int measureDur = (channelNumber + 1) * 200;   // duration of measurement in ms (-> during this time, the system is blocked)

// Measurement timing
unsigned long loopStart, elapsed;           // ms timestamp of beginning and end of measurement loop
unsigned long progStart, progEnd;           // ms timestamp of beginning and end of experiment

// Switches and logical operators
bool startTrigger = false;                  // trigger for start of measurement

// Instances
SoftwareSerial mySer(RX, TX);               // serial connection to the Firesting
Ardoxy ardoxy(mySer);                       // ardoxy instance

// PID
double output[channelNumber];               // Output values from PID

// PID control for channel 1
double Kp1 = 10;                            // Proportional coefficient for channel 1
double Ki1 = 1;                             // Integral coefficient for channel 1
double Kd1 = 0;                             // Derivative coefficient for channel 1
PID valvePID1(&DOFloat[0], &output[0], &DOSetpoints[0], Kp1, Ki1, Kd1, REVERSE);
unsigned int windowSize = round(sampInterval/200);        // PID controller will calculate an output between 0 and windowSize.
                                                          // This will be multiplied by 200 to ensure a minimum opening time of 200 msec to protect the relays. 
                                                          // E.g. output = 1 -> opening time 200 msec; output 50 -> opening time 10,000 msec

//#######################################################################################
//###                                   Setup                                         ###
//#######################################################################################

void setup() {
  Serial.begin(19200);
  delay(100);
  for (int i = 0; i < channelNumber; i++){
    // Set up relay pin
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], closed);
  }

  // Set up PID for channel 1
  Ardoxy::configurePID(valvePID1, Kp1, Ki1, Kd1, sampInterval, windowSize);
  Serial.println("------------ Auto-generated Arduino Sketch ------------");
  Serial.print("FireSting channel: ");
  Serial.println(channelNumber);
  Serial.print(" Air saturation threshold(s) (% air sat.): ");
  for (int i = 0; i < channelNumber; i++){
    Serial.print(DOSetpoints[i]);
    Serial.print("; ");
  }
  Serial.println();
  Serial.print("Measurement interval (ms): ");
  Serial.println(sampInterval);
  Serial.print("Experiment Duration (min): ");
  Serial.println(experimentDuration);
  Serial.println("Use the SerialPlot software to plot DO and temperature");
  Serial.println("Send \"1\" to start measurement and \"0\" to end measurement.");
  Serial.println("-------------------------------------------------------");
}
    
//#######################################################################################
//###                                   Loop                                          ###
//#######################################################################################

void loop() {
  // wait for serial input to start measurement.
  if (Serial.available() > 0) {
    switch(Serial.read()){
      case '1':
          if (!ardoxy.begin()) {                                    // Start serial communication with FireSting
              Serial.println("Connection failed — check wiring");
              break;
          }
          startTrigger = true;
          for (int i = 0; i < channelNumber; i++) {
            Serial.print("Air_sat_ch_");
            Serial.print(channelArray[i]);
            Serial.print(";");
            Serial.print("Open_time_ch_");
            Serial.print(channelArray[i]);
            Serial.print(";");
          }
          Serial.println("Temp_deg_C");
          // Define time points for decrease end and trial end
          progStart = millis();
          progEnd = progStart + experimentDuration * 60 * 1000UL;
          break;
      case '0':
          startTrigger = false;
          ardoxy.end();
          Serial.println("Stopped");
          Ardoxy::closeRelays(channelNumber, relayPins);
          break;
    }
  }

  if (startTrigger) {           
    loopStart = millis();                               // get the time
    // If the end of the experiment hasn't been reached...
    if (loopStart <= progEnd){
      if (ardoxy.measureAll(channelNumber, DOFloat, &tempFloat)) {
        // compute opening time of solenoid valve
        valvePID1.Compute();
        // Print to serial
        for (int i = 0; i < channelNumber; i++) {
          Serial.print(DOFloat[i]);
          Serial.print(";");
          Serial.print(output[i]*200/1000);
          Serial.print(";");
        }
        Serial.println(tempFloat);

        // schedule solenoid valves
        Ardoxy::scheduleRelays(channelNumber, output, relayPins, sampInterval - measureDur);

        // wait for next loop iteration
        elapsed = millis()-loopStart;
        if (sampInterval > elapsed) {
          delay(sampInterval - elapsed);          
        }
      } else {
        Ardoxy::closeRelays(channelNumber, relayPins);
        Serial.println("Com error. Check connections and send \"1\" to restart.");
        ardoxy.end();
        startTrigger = false;
      }
    }
    
    // at the end of the experiment, close all valves and end the experiment
    else { 
      Ardoxy::closeRelays(channelNumber, relayPins);
      Serial.println("End of experiment. Arduino stopps. Send \"1\" to re-start.");
      ardoxy.end();
      startTrigger = false;
    }
  }
}
