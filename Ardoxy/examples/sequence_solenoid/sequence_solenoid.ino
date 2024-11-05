/*
  Ardoxy example

  Regulate DO to a sequence of setpoints with defined durations using a solenoid valve.
  Oxygen sensor is calbrated using the Pyro Oxygen Logger Software.
  A flyback diode is connected to the +/- poles of the valve to protect the circuit from inductive charges

  You must define the following variables:
  - sampInt - Sample interval in ms
  - channel - channel on FireSting meter where oxygen sensor is connected
  - pin numers
  - number of phases (i.e., steps of the sequence)
  - DO setpoints, durations, and phase-type (change or hold) for each phase of the sequence
  - *optionally* adjust the PID tunings (Kp, Ki, Kd) for the two PID control instances

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

unsigned long sampInterval = 2000;              // sampling interval in ms
const int nPhases = 4;                          // number of phases
const int channel = 1;                          // channel on the Firesting meter where the sensor is connected

// Define pins
const int RX = 8;                               // RX pin for serial communication
const int TX = 9;                               // TX pin for serial communication
const int relayPin = 3;                         // pin for relay operation

// DO setpoints for each phase
double DOSetpoints[nPhases] = {50.00, 50.00, 30.00, 30.00};     // target air saturation value
unsigned long durations[nPhases] = {15, 15, 15, 15};            // duration of each phase in minutes
const char phaseType[nPhases][2] = {"c", "h", "c", "h"};        // phases: "c" = change, "h" = hold, "p" = pause

// Define coefficients for PID control
// Coefficients for constant phases
double constantKp = 10;                                                                    // coefficient for proportional control
double constantKi = 1;                                                                     // coefficient for integrative control
double constantKd = 0;                                                                     // coefficient for differential control

// Coefficients for decrease/increase phase
double changeKp = 10;                                                                        // coefficient for proportional control
double changeKi = 1;                                                                        // coefficient for integrative control
double changeKd = 0;                                                                        // coefficient for differential control

//#######################################################################################
//###                            Requisite Variables                                  ###
//#######################################################################################

// DO measurement
double DOFloat, DOFloatPrev;                        // Floating point DO values for each channel
long DOInt, tempInt;                                // for measurement result of each channel
double tempFloat;                                   // measurement result as floating point number
const int closed = HIGH;                            // marks the output on the relay pin, which should close the valve
const int open = !closed;
const int measureDur = 400;                         // duration of measurement in ms (-> during this time, the system is blocked)
double deltaDO;                                     // measured change rate
double changeRate;                                  // setpoint for change rate
double setpoint;                                    // setpoint for PID control (first: DO decrease rate, then: air saturation)
int counter = 0;                                    // measurement interval counter for re-calcuation of change rage
const int rateReCalc = round(60000/sampInterval);   // change rate has to be re-calculated to avoid drift (here: every 60 seconds)

// Measurement timing
int phaseIdx = 0;                           // index of current phase
unsigned long phaseMarks[nPhases+1];        // array for timestamps when the controller should proceed from one phase to the next
unsigned long loopStart, elapsed;           // ms timestamp of beginning and end of measurement loop
unsigned long progStart, progEnd;           // ms timestamp of beginning and end of experiment

// Switches and logical operators
bool startTrigger = false;                  // trigger for start of measurement
bool valveOpen = false;                     // indicator if the solenoid should remain open over one loop iteration
int check;                                  // numerical indicator of succesful measurement (1: success, 0: no connection, 9: mismatch)

// Instances
SoftwareSerial mySer(RX, TX);               // serial connection to the Firesting
Ardoxy ardoxy(mySer);                       // ardoxy instance

// PID
double output;                              // outputs that were calculated by PID library
PID changePID(&deltaDO, &output, &changeRate, changeKp, changeKi, changeKd, REVERSE);
PID holdPID(&DOFloat, &output, &setpoint, constantKp, constantKi, constantKd, REVERSE);
unsigned int windowSize = round(sampInterval/200);        // PID controller will calculate an output between 0 and windowSize.
                                                          // This will be multiplied by 200 to ensure a minimum opening time of 200 msec to protect the relays. 
                                                          // E.g. output = 1 -> opening time 200 msec; output 50 -> opening time 10,000 msec

//#######################################################################################
//###                                   Setup                                         ###
//#######################################################################################

void setup() {
  Serial.begin(19200);
  delay(100);    

  // Set up relay pin
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, closed);

  //# Set up PID control #
  changePID.SetMode(AUTOMATIC);
  changePID.SetSampleTime(sampInterval);
  changePID.SetOutputLimits(0, windowSize);
  
  holdPID.SetMode(AUTOMATIC);
  holdPID.SetSampleTime(sampInterval);
  holdPID.SetOutputLimits(0, windowSize);

  //# Durations as ms
  for(int i = 0; i < nPhases; i++) {
    durations[i] = durations[i] * 60 * 1000UL;
  }

  Serial.println("------------ Sequence Control Sketch ------------");
  Serial.print("FireSting channel: ");
  Serial.println(channel);
  Serial.print(" Air saturation threshold(s) (% air sat.): ");
  for (int i = 0; i < nPhases; i++){
    Serial.print(DOSetpoints[i]);
    Serial.print("; ");
  }
  Serial.println();
  Serial.print("Measurement interval (ms): ");
  Serial.println(sampInterval);
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
          
          // Define phase timestamps
          progStart = millis();
          phaseMarks[0] = progStart;
          for(int i = 0; i < nPhases; i++){
            phaseMarks[i+1] = phaseMarks[i] + durations[i];
          }
          Serial.println("Air_sat;Open_time;Temp_deg_C;Setpoint;Phase_nr;Phase_type");
          if (phaseIdx >= nPhases){
            ardoxy.end();
            digitalWrite(relayPin, closed);
            valveOpen = false;
            Serial.println("End of experiment. Arduino stopps.");
            startTrigger = false;
          } else {
            // First measurement and settings for the first phase
            check = ardoxy.measureTemp();                     // measure temperature
            if(check == 1){
              tempInt = ardoxy.readoutTemp();                 // read temperature value from results register
              tempFloat = tempInt / 1000.00;
              check = ardoxy.measureDO(channel);                // measure DO on channel i+1
              if(check == 1){
                DOInt = ardoxy.readoutDO(channel);              // read DO value from results register
                DOFloat = DOInt / 1000.00;                      // convert to floating point number

                // Define setpoints according to phase index
                // If it's a change phase
                if (strcmp(phaseType[phaseIdx],"c")==0){
                  counter = 0;
                  changeRate = (DOSetpoints[phaseIdx] - DOFloat) / (float(durations[phaseIdx]) / 60000.00); // change rate for DO decline per minute
                  // Set Controller Direction
                  if (changeRate > 0){
                    changePID.SetControllerDirection(DIRECT);
                  } else {
                    changePID.SetControllerDirection(REVERSE);
                  }
                } else if (strcmp(phaseType[phaseIdx],"h")==0){
                  setpoint = DOSetpoints[phaseIdx];
                  // Set Controller Direction
                  if (setpoint > 100){
                    holdPID.SetControllerDirection(DIRECT);
                  } else {
                    holdPID.SetControllerDirection(REVERSE);
                  }
                } else if(strcmp(phaseType[phaseIdx],"p")==0){
                  Serial.println("Paused. Send a \"9\" to continue control.");
                  startTrigger = false;
                  digitalWrite(relayPin, closed);
                }
              }
            }
            if(check != 1){
              Serial.println("Com error. Check connections and send \"1\" to restart.");
              ardoxy.end();
              startTrigger = false;
            }
          }
      break;

      case '0':
          startTrigger = false;
          ardoxy.end();
          Serial.println("Stopped");
          digitalWrite(relayPin, closed);
          valveOpen = false;
          break;
    }
  }

  if (startTrigger) {           
    loopStart = millis();                               // get the time

    // If the loop start has passed a phase mark...
    if(loopStart > phaseMarks[phaseIdx+1]){
      phaseIdx += 1;
      // Close valve at end of the experiment and stop arduino
      if (phaseIdx >= nPhases){
        ardoxy.end();
        digitalWrite(relayPin, closed);
        valveOpen = false;
        Serial.println("End of experiment. Arduino stopps.");
        startTrigger = false;
      } else {
        // Define setpoints according to phase index
        // If it's a change phase
        if (strcmp(phaseType[phaseIdx],"c")==0){
          counter = 0;
          changeRate = (DOSetpoints[phaseIdx] - DOFloat) / (float(durations[phaseIdx]) / 60000.00); // change rate for DO decline per minute
          // Set Controller Direction
          if (changeRate > 0){
            changePID.SetControllerDirection(DIRECT);
          } else {
            changePID.SetControllerDirection(REVERSE);
          }
        } else if (strcmp(phaseType[phaseIdx],"h")==0){
          setpoint = DOSetpoints[phaseIdx];
          // Set Controller Direction
          if (setpoint > 100){
            holdPID.SetControllerDirection(DIRECT);
          } else {
            holdPID.SetControllerDirection(REVERSE);
          }
        } else if(strcmp(phaseType[phaseIdx],"p")==0){
          Serial.println("Paused. Send a \"9\" to continue control.");
          startTrigger = false;
          digitalWrite(relayPin, closed);
        }
      }
    }
  }

  if (startTrigger){
    DOFloatPrev = DOFloat;
    check = ardoxy.measureTemp();                     // measure temperature
    if (check == 1){
      tempInt = ardoxy.readoutTemp();                 // read temperature value from results register
      tempFloat = tempInt / 1000.00;
      check = ardoxy.measureDO(channel);                // measure DO on channel i+1
      if (check == 1){
        DOInt = ardoxy.readoutDO(channel);              // read DO value from results register
        DOFloat = DOInt / 1000.00;                      // convert to floating point number
        if (strcmp(phaseType[phaseIdx], "c") == 0){
          deltaDO = (DOFloat - DOFloatPrev)*60/(sampInterval/1000.00); // change rate for DO decline per minute
          counter += 1;
          if (counter >= rateReCalc){
            counter = 0;
            changeRate = (DOSetpoints[phaseIdx] - DOFloat) / (float(durations[phaseIdx]-(loopStart - phaseMarks[phaseIdx])) / 60000.00); // change rate for DO decline per minute
          }
          changePID.Compute();
        } else if (strcmp(phaseType[phaseIdx], "h") == 0){
          holdPID.Compute();
        }
        // Print to serial
        Serial.print(DOFloat);
        Serial.print(";");
        Serial.print(output*200/1000);
        Serial.print(";");
        Serial.print(tempFloat);
        Serial.print(";");
        Serial.print(DOSetpoints[phaseIdx]);
        Serial.print(";");
        Serial.print(phaseIdx+1);
        Serial.print(";");
        Serial.println(phaseType[phaseIdx]);

    
        // operate solenoid
        // first loop: open all valves that have a nonzero PID output and close those with zero output
        if (output == 0){
          if (valveOpen){
            digitalWrite(relayPin, closed);
            valveOpen = false;
          }
        } else {
          if (!valveOpen) {
            digitalWrite(relayPin, open);
          }
          if (output * 200 >= (sampInterval - measureDur)){
            valveOpen = true;
          } else {
            delay(output*200);
            digitalWrite(relayPin, closed);
            valveOpen = false;
          }
        }
        // wait for next loop iteration
        elapsed = millis()-loopStart;
        if (sampInterval > elapsed) {
          delay(sampInterval - elapsed);          
        }
      }
    }
    if(check != 1){
      Serial.println("Com error. Check connections and send \"1\" to restart.");
      ardoxy.end();
      startTrigger = false;
    }
  }
}