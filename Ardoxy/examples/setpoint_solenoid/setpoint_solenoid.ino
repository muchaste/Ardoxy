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
long DOInt, tempInt;                                // for measurement result of each channel
double tempFloat;                                   // measurement result as floating point number
int check;                                          // numerical indicator of succesful measurement (1: success, 0: no connection, 9: mismatch)
const int closed = HIGH;                            // marks the output on the relay pin, which should close the valve
const int open = !closed;
const int measureDur = (channelNumber + 1) * 200;   // duration of measurement in ms (-> during this time, the system is blocked)

// Measurement timing
unsigned long loopStart, elapsed;           // ms timestamp of beginning and end of measurement loop
unsigned long progStart, progEnd;           // ms timestamp of beginning and end of experiment

// Switches and logical operators
bool startTrigger = false;                  // trigger for start of measurement
int valveOpen[channelNumber];               // indicator if the solenoid should remain open over one loop iteration
int idcArray[channelNumber];                // array for indices of the relayPins array
int idx;                                    // index of currently operated relay

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
    valveOpen[i] = 0;
      
    // Set up relay pin for channel {channel}
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], closed);
  }

  // Set up PID for channel 1
  valvePID1.SetMode(AUTOMATIC);
  valvePID1.SetSampleTime(sampInterval);
  valvePID1.SetOutputLimits(0, windowSize);
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
          startTrigger = true;
          ardoxy.begin();                                           // Start serial communication with FireSting
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
          for (int i = 0; i < channelNumber; i++) {
            digitalWrite(relayPins[i], closed);
            valveOpen[i] = 0;
          }
          break;
    }
  }

  if (startTrigger) {           
    loopStart = millis();                               // get the time
    // If the end of the experiment hasn't been reached...
    if (loopStart <= progEnd){
      check = ardoxy.measureTemp();                     // measure temperature
      if(check == 1){
        tempInt = ardoxy.readoutTemp();                 // read temperature value from results register
        tempFloat = tempInt / 1000.00;
        for (int i = 0; i < channelNumber; i++) {
          check = ardoxy.measureDO(i+1);                // measure DO on channel i+1
          if(check == 1){
            DOInt = ardoxy.readoutDO(i+1);              // read DO value from results register
            DOFloat[i] = DOInt / 1000.00;               // convert to floating point number
          }
          else {                                        // If the DO measurement returns with an error
            for (int i = 0; i < channelNumber; i++) {
              digitalWrite(relayPins[i], closed);
            }
            Serial.println("Com error. Check connections and send \"1\" to restart.");
            ardoxy.end();
            startTrigger = false;
          }
        }
      }
      else {                                            // If the temp. measurement returns with an error
        for (int i = 0; i < channelNumber; i++) {
          digitalWrite(relayPins[i], closed);
        }
        Serial.println("Com error. Check connections and send \"1\" to restart.");
        ardoxy.end();
        startTrigger = false;
      }

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
    
      // if more than one channel: sort by opening time
      if (channelNumber > 1) {
        for (int k = 0; k < channelNumber; k++) {         // sort indices of channels in ascending order of opening time
          idcArray[k] = channelNumber-1;
          for (int m = 0; m < channelNumber; m++) {
            if (output[m] > output[k]) {
              idcArray[k] -= 1;
            }
            else if (output[m] == output[k]){
              if (m > k){
                idcArray[k] -= 1;
              }
            }
          }
        }
      }
      else {
        idcArray[0] = 0;
      }
      
      // operate solenoid
      // first loop: open all valves that have a nonzero PID output and close those with zero output
      for (int i = 0; i < channelNumber; i++){
        if (output[i] == 0){
          if (valveOpen[i]){
            digitalWrite(relayPins[i], closed);
            valveOpen[i] = 0;
          }
        }
        else {
          if (!valveOpen[i]) {
            digitalWrite(relayPins[i], open);
          }
          if (output[i] * 200 >= (sampInterval - measureDur)){
            valveOpen[i] = 1;
          }
          else {
            valveOpen[i] = 0;
          }
        }
      }
      
      // second loop: close the valves that have shorter opening times than the loop duration
      for (int i = 0; i < channelNumber; i++){
        idx = idcArray[i];
        if((!valveOpen[idx]) && (output[idx] > 0)){
          if (i == 0) {
            delay(output[idx]*200);
            digitalWrite(relayPins[idx], closed);
          }
          else {
            delay((output[idx]*200)-(output[idcArray[i-1]]*200));
            digitalWrite(relayPins[idx], closed);
          }
        }
      }
      
      // wait for next loop iteration
      elapsed = millis()-loopStart;
      if (sampInterval > elapsed) {
        delay(sampInterval - elapsed);          
      }         
    }
    
    // at the end of the experiment, close all valves and end the experiment
    else { 
      for (int i = 0; i < channelNumber; i++) {
        digitalWrite(relayPins[i], closed);
      }
      Serial.println("End of experiment. Arduino stopps. Send \"1\" to re-start.");
      ardoxy.end();
      startTrigger = false;
    }
  }
}
