/*
  Ardoxy example

  Regulate DO to a static setpoint with defined experimental duration using a stepper motor connected to a needle valve.
  Oxygen sensor is calbrated using the Pyro Oxygen Logger Software.
  Oxygen probe is connected to channel 1.
  
  The circuit:
  - Arduino Uno
  - FireStingO2 - 7 pin connector X1:
    *Pin 1 connected to Arduino GND
    *Pin 2 connected to Arduino 5V 
    *Pin 4 connected to Arduino RX (here: 8)
    *Pin 5 connected to Arduino TX (here: 9)
  - Stepper motor connected to motor shield

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
#include <Stepper.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

//#######################################################################################
//###                              General settings                                   ###
//#######################################################################################

// Set experimental conditions
const int sampInterval = 2000;                        // sampling interval in ms (due to the duration of the measurement and communication, use interval > 1000 msec)
double airSatThreshold = 30.00;                       // target air saturation value
unsigned int experimentDuration = 60;                 // duration for controlled DO in min

// Define pins
const int RX = 8;                                     // RX and TX pins for serial communication with FireStingO2
const int TX = 9;                                     // RX and TX pins for serial communication with FireStingO2

// Define coefficients for PID control. TEST THE SETTINGS FOR PID CONTROL BEFORE YOU USE THIS SYSTEM!!   
double Kp = 20;                                        // coefficient for proportional control
double Ki = 1;                                        // coefficient for integrative control
double Kd = 0;                                        // coefficient for differential control

//#######################################################################################
//###                           Requisite variables                                   ###
//### In the following, necessary variables are defined. If you don't change too much ###
//### of this code, you will not need to change a lot here but you will need to read  ###
//### through it at least to understand how the functions work.                       ###
//#######################################################################################

// DO measurement
long DOInt, tempInt;                        // for measurement result
double DOFloat, tempFloat;                  // measurement result as floating point number
int check;                                  // numerical indicator of succesful measurement (1: success, 0: no connection, 9: mismatch)
const int measureDur = 500;                 // duration of measurement in ms (-> during this time, the system is blocked)

// Motor settings
const int stepsPerRevolution = 200;                       // number of steps per motor revolution
const int opened = round(stepsPerRevolution*1.5);         // maximum opening position of valve
const int motorSpeed = 10;                                // motor speed in rpm
const int closed = 0;                                     // minimum opening of valve
int stepCount = 0;                                        // counts motor steps to control motor position
const int stepsPerInterval = floor(stepsPerRevolution*(motorSpeed/3)*((sampInterval-measureDur)/(60.0*1000)));
                                                          // the max. number of steps that can be adjusted per interval

// Measurement timing
unsigned long loopStart, elapsed;           // ms timestamp of beginning and end of measurement loop
unsigned long progStart, progEnd;           // ms timestamp of beginning and end of experiment

// Switches and logical operators
bool startTrigger = false;                  // trigger for start of measurement
bool valveOpen = false;                     // indicator if the solenoid should remain open over one loop iteration

// Instances
SoftwareSerial mySer(RX, TX);               // serial connection to the Firesting
Ardoxy ardoxy(mySer);                       // ardoxy instance

// PID control
double output;                              // holds output that was calculated by PID library
int PIDsteps;                               // output of PID algorithm
PID valvePID(&DOFloat, &output, &airSatThreshold, Kp, Ki, Kd, REVERSE);

// initialize the stepper library
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_StepperMotor *myStepper = AFMS.getStepper(stepsPerRevolution, 2);

//#######################################################################################
//###                                   Setup                                         ###
//#######################################################################################

void setup() {
  Serial.begin(19200);
  AFMS.begin();
  delay(100);

  // Set the motor speed (RPMs):
  myStepper->setSpeed(10);

  // Set up PID control
  valvePID.SetMode(AUTOMATIC);
  valvePID.SetSampleTime(sampInterval);
  valvePID.SetOutputLimits(0, opened);

  // Print experimental conditions
  Serial.println("------------ Ardoxy measure and control example ------------");
  Serial.println("FireSting channel: 1");
  Serial.print("Measurement interval (ms): ");
  Serial.println(sampInterval);
  Serial.print(" Air saturation threshold (% air sat.): ");
  Serial.println(airSatThreshold);
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
          ardoxy.begin();                                 // Start serial communication with FireSting
          Serial.println("DO_air_sat;Temp_deg_C;Open_steps");
          // Define time points for decrease end and trial end
          progStart = millis();
          progEnd = progStart + experimentDuration * 60 * 1000UL;
          break;
      case '0':
          startTrigger = false;
          ardoxy.end();
          Serial.println("Stopped");
          myStepper->step(stepCount, BACKWARD, MICROSTEP);
          stepCount = 0;
          break;
    }
  }

  if (startTrigger) {
    loopStart = millis();                       // get time at beginning of loop

    // If the end of the experiment hasn't been reached...
    if (loopStart <= progEnd){
      check = ardoxy.measureSeq(1);             // measure sequence on channel 1
      if(check == 1){
        tempInt = ardoxy.readoutTemp();  // read temperature value from results register
        tempFloat = tempInt / 1000.00;

        DOInt = ardoxy.readoutDO(1);      // read DO value from results register
        DOFloat = DOInt / 1000.00;

        // compute opening state of needle valve (= steps of the motor)
        valvePID.Compute();

        // Operate stepper motor
        PIDsteps = round(output);
        if(PIDsteps > stepCount) {
          if((PIDsteps - stepCount) > stepsPerInterval) {
            PIDsteps = stepCount + stepsPerInterval;
            output = PIDsteps * 1.0;
          }
          myStepper->step(int(PIDsteps-stepCount), FORWARD, MICROSTEP);
        } else if (PIDsteps < stepCount){
          if((stepCount - PIDsteps) > stepsPerInterval) {
            PIDsteps = stepCount - stepsPerInterval;
            output = PIDsteps * 1.0;
          }
          myStepper->step(int(stepCount-PIDsteps), BACKWARD, MICROSTEP);
        }
        stepCount = PIDsteps;

        // Print to serial
        Serial.print(DOFloat);
        Serial.print(";");
        Serial.print(tempFloat);
        Serial.print(";");
        Serial.println(PIDsteps);

        // wait for next loop iteration
        elapsed = millis()-loopStart;
        if (sampInterval > elapsed) {
          delay(sampInterval - elapsed);          
        }
      }  
      else {       // If the measurement returns with an error
        myStepper->step(stepCount, BACKWARD, MICROSTEP);
        stepCount = 0;
        Serial.println("Com error. Check connections and send \"1\" to restart.");
        ardoxy.end();
        startTrigger = false;
      }
    }
    else {
      myStepper->step(stepCount, BACKWARD, MICROSTEP);
      stepCount = 0;
      Serial.println("End of experiment. Arduino stopps. Send \"1\" to re-start.");
      ardoxy.end();
      startTrigger = false;
    }
  }
}
