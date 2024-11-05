
/*
  Ardoxy example
  
  Regulate DO to a sequence of setpoints with defined durations using solenoid valves.
  DO is displayed on LCD and stored on SD card.
  Oxygen sensor is calbrated using the Pyro Oxygen Logger Software.
  A flyback diode is connected to the +/- poles of the valves to protect the circuit from inductive charges

  You must define the following variables:
  - sampleInterval - Sample interval in ms
  - channelNumber - number of channels
  - channelArray - channel on FireSting meter where oxygen sensors are connected
  - pin numers
  - number of phases (i.e., steps of the sequence)
  - DO setpoints (airSatThresholds), durations, and phase-type (change or hold) for each phase of the sequence
  - *optionally* adjust the PID tunings (Kp, Ki, Kd) for the PID controller
  The circuit:
  - Arduino Mega
  - Adafruit datalogger shield with SD card and RTC
  - FireStingO2 - 7 pin connector:
    *Pin 1 connected to Arduino GND
    *Pin 2 connected to Arduino 5V 
    *Pin 4 connected to Arduino RX1 (here: 19)
    *Pin 5 connected to Arduino TX1 (here: 18)
  - Solenoid valve on relay module, connected to digital pins on Arduino (here: 46, 48, 50, 52)

  The software:
  No software needed, values are stored on SD card, or
  simply read the values from the serial monitor or LCD display

  by Stefan Mucha

*/

#include <Ardoxy.h>
#include <PID_v1.h>
#include <SdFat.h>
#include <Wire.h>
#include "RTClib.h"
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
#define WHITE 0x7                                     // LCD color code


//#######################################################################################
//###                              General settings                                   ###
//### This program is intended to work with 4 channels for measurement and control.   ###
//### Thus most arrays have a length of 4. Adapt the following lines to your use.     ###
//#######################################################################################

//# Set experimental conditions #
const int channelNumber = 4;                                                                  // total number of measurement channels
long int samples = 1;                                                                         // number of measurements that are averaged to one air saturation value 
                                                                                              // to reduce sensor fluctuation (oversampling)
char tankID[channelNumber][6] = {"A", "B", "C", "D"};                                         // IDs assigned to the channels in the order of the channelArray
char tempID[6] = {"B"};                                                                       // tanks where the temperature sensors are placed (1 per sensor)
long sampleInterval = 30 * 1000UL;                                                            // measurement and control interval in second
double airSatThreshold[channelNumber] = {100.0, 100.0, 100.0, 15.0};                          // air saturation threshold including first decimal                            
double lowDOThreshold = 7.0;                                                                  // threshold for low oxygen that causes the program to do something
int channelArray[channelNumber] = {1, 2, 3, 4};                                               // measurement channels from firesting devices 1 and 2 in that order

//# Set the RTC? #
const int setRTC = 1;                                                                         // upload this sketch once with setRTC = 1 to set the clock to the time
                                                                                              // when this sketch was compiled. Then set setRTC = 0 and upload again.
                                                                                              

//# Define pins #
int relayPin[channelNumber] = {46, 48, 50, 52};                                               // pins for relay operation ***ADAPT THIS TO FIT YOUR WIRING***
const int chipSelect = 10;                                                                    // chip pin for SD card (UNO: 4; MEGA: 53, Adafruit shield: 10)


//#######################################################################################
//###                                PID settings                                     ###
//### The arduino operates solenoid valves through a relay module to bubble N2 into   ###
//### the tanks. It calculates a time interval using the PIDlibrary (by Brett         ###
//### Beauregard). TEST THE SETTINGS FOR PID CONTROL BEFORE YOU USE THIS SYSTEM!!     ###
//### If you're not sure, replace the PID control with a simpler solution             ###
//### (e.g. bubble N2 for 10 sec if there's a large oxygen difference etc.).          ###
//### To control for stress due to bubbling, compressed air is bubbled into control   ###
//### tanks.                                                                          ###
//#######################################################################################

double Kp[channelNumber] = {10, 10, 10, 10};                    // coefficient for proportional control
double Ki[channelNumber] = {1, 1, 1, 1};                        // coefficient for integrative control
double Kd[channelNumber] = {1, 1, 1, 1};                        // coefficient for differential control
const int windowSize = 75;                                       // The PID will calculate an output between 0 and 75.
                                                                // This will be multiplied by 200 to ensure a minimum opening time of 200 msec. 
                                                                // E.g. output = 1 -> opening time 200 msec; output 50 -> opening time 10,000 msec


//#######################################################################################
//###                           Requisite variables                                   ###
//### In the following, necessary variables are defined. If you don't change too much ###
//### of this code, you will not need to change a lot here but you will need to read  ###
//### through it at least to understand how the functions work.                       ###
//#######################################################################################

//# Switches and logical operators #
boolean lowDO = false;                        // boolean for low oxygen threshold
int activeChannel = 1;                        // measurement channel
int check;                                    // numerical indicator of succesful measurement (1: success, 0: no connection, 9: mismatch)

//# Measurement timing #
unsigned long loopStart, elapsed;             // ms timestamp of beginning and end of measurement loop
int curday, lastday;                          // int of current day and last day (date) - to detect change and create a new logfile every day

//# Logging and SD card #
RTC_PCF8523 RTC;                              // real time clock
SdFs SD;
FsFile logfile;                               // initializes the logfile
char filename[21];                            // array for filename of .csv file
byte n = 0;                                   // row index for .csv file

//# Oxygen optode #
char DOReadCom[11] = "REA 1 3 4\r";           // template for DO-read command that is sent to sensor during void toggleRead() (length = 10 because of /0 string terminator)
char tempReadCom[11] = "REA 1 3 5\r";         // temperature read command
long DOInt, tempInt;                          // for measurement result
long DOSum;                                   // summing variable for air saturations in case oversampling is used (see sample variable)
double DOFloat[channelNumber], tempFloat;     // measurement result as floating point number
double lowDOValue;                            // variable for low DO values that are below the critical threshold defined above
char lowDOTank[6];                            // array for tank name in which low DO has been measured
Ardoxy ardoxy(Serial1);                       // create ardoxy instance on hardware serial port 1

//# Relay operation #
double relayArray[3][channelNumber];          // array with relay pin and assigned output values
double Output[channelNumber];                 // holds output that was calculated by PID library

//# Hardcoded PID setup for 4 control channels #
PID relay1PID(&DOFloat[0], &Output[0], &airSatThreshold[0], Kp[0], Ki[0], Kd[0], REVERSE);    
PID relay2PID(&DOFloat[1], &Output[1], &airSatThreshold[1], Kp[1], Ki[1], Kd[1], REVERSE);
PID relay3PID(&DOFloat[2], &Output[2], &airSatThreshold[2], Kp[2], Ki[2], Kd[2], REVERSE);
PID relay4PID(&DOFloat[3], &Output[3], &airSatThreshold[3], Kp[3], Ki[3], Kd[3], REVERSE);

//# LCD Display #
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
int airSatLCD = 0;                            // integer to display rounded values (due to space constraints on the LCD)
int cursorX = 0;                              // X position of cursor


//#######################################################################################
//###                           Requisite Functions                                   ###
//###               These functions are executed in the main() loop.                  ###
//### These functions only need to be changed if you want to change the number of     ###
//###                           measurement channels.                                 ###
//#######################################################################################

//# Send air saturation readings to serial monitor and LCD #
void showNewData() {
  lcd.clear();
  lcd.setCursor(0, 0);
  DateTime now;
  now = RTC.now();  
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" - ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
  Serial.print(tempID);
  Serial.print(": ");
  Serial.print(tempFloat);
  Serial.println("°C ");

  for (int k = 0; k < (channelNumber); k++) {
    if (k == 4) {                                     
      lcd.setCursor(0, 1);                          // break line on LCD display when the 5th DO value is reached
    }
    airSatLCD = int(lround(DOFloat[k]));
    lcd.print(airSatLCD);
    lcd.print(" ");
    Serial.print(tankID[k]);
    Serial.print(": ");
    Serial.print(DOFloat[k]);
    Serial.println("% air saturation");
  }
}


//# Check if air saturations are below the lowDO threshold #
void DOCheck() {
  for (int k = 0; k < channelNumber; k++){
    if (DOFloat[k] < lowDOThreshold){
      lowDO = true;
      lowDOValue = DOFloat[k];
      strcpy(lowDOTank, tankID[k]);
    }
  }
}

//# Toggle relay based on measured airSat values #
void toggleRelay() {
  relay1PID.Compute();                                  // compute the output (output * 200 = opening time) based on the input (air saturation) and threshold
  relay2PID.Compute();
  relay3PID.Compute();
  relay4PID.Compute();
  
  for (int k = 0; k < channelNumber; k++) {
    relayArray[0][k] = relayPin[k];
    relayArray[1][k] = double(int(Output[k])*200.00);   // PID computes an output between 0 and 50, the multiplicator makes sure that the relay operation time is at least 200ms
    relayArray[2][k] = DOFloat[k];
  }
  double temp[4];                                       // temporary array to sort all channels with the smallest based on the computed output (lowest output first). 
                                                        // This is necessary as the valves are kept open using the delay()-function which halts all activity. 
                                                        // With 8 channels, opening one valve after the other is too time consuming. With the opening times sorted, all valves 
                                                        // can be opened at the same time and then closed one after another.
  for (int k = 0; k < channelNumber - 1; k++) {         // sort array with lowest difference first
    for (int m = k + 1; m < channelNumber; m++) {
      if (relayArray[1][m] < relayArray[1][k]) {
        temp[0] = relayArray[0][k];                     // store higer value + relayPin in temp array
        temp[1] = relayArray[1][k];
        temp[2] = relayArray[2][k];
        relayArray[0][k] = relayArray[0][m];            // move lower value to position i
        relayArray[1][k] = relayArray[1][m];
        relayArray[2][k] = relayArray[2][m];
        relayArray[0][m] = temp[0];                     // insert higher value at position j
        relayArray[1][m] = temp[1];                     // now the values at i and j have switched places in the relayArray
        relayArray[2][m] = temp[2];
      }
    }
  }
  for (int k = 0; k < channelNumber; k++) {
    if (relayArray[1][k] > 0){                              // skip the channels that don't have to be operated (output = 0.00)
      for (int m = k; m < channelNumber; m++) {             // open all other valves
        digitalWrite(relayArray[0][m], LOW);
      }
      if (k == 0) {                                         // close the valve with the lowest output first
        delay(relayArray[1][k]);
        digitalWrite(relayArray[0][k], HIGH);
      }
      else {
        delay(relayArray[1][k] - relayArray[1][k - 1]);     // keep the other valves open based on the difference of the output value
        digitalWrite(relayArray[0][k], HIGH);               // close the valve
      }
    }
  }
}

//# Create logfile on SD card (needs global variable "filename")
void createLogfile(){

  DateTime now;
  now = RTC.now();                                  // fetch time and date from RTC
  sprintf(filename, "%4d_%2d_%2d_%2d_%2d.csv", now.year(), now.month(), now.day(), now.hour(), now.minute()); // choose filename for logfile on SD that does not exist yet, includes a three-digit sequence number in the file name
  Serial.println(filename);
  delay(100);
      
// Create logfile and write header information
  logfile = SD.open(filename, FILE_WRITE);                
  if (logfile) {
    logfile.println(";");                                   // print a leading blank line
    logfile.print("Date:;");                                // print header information: date, time, air saturation threshold, channels, tank IDs etc
    logfile.print(now.year(), DEC);
    logfile.print("/");
    logfile.print(now.month(), DEC);
    logfile.print("/");
    logfile.print(now.day(), DEC);
    logfile.print(";");
    logfile.print("Time:;");
    logfile.print(now.hour(), DEC);
    logfile.print(":");
    logfile.print(now.minute(), DEC);
    logfile.print(":");
    logfile.println(now.second(), DEC);
    logfile.print(";Measurement interval [sec]:;");
    logfile.print(sampleInterval / 1000);
    logfile.print(";Active channels:;");
    logfile.print(channelNumber);
    logfile.print(";Temp Sensor:;");
    logfile.print(tempID);
    logfile.println();
    logfile.print("Tank ID:;");
    for (int i = 0; i < channelNumber; i++) {
      logfile.print(tankID[i]);
      logfile.print(";");
    }
    logfile.println(";");
    logfile.print("Channel:;");
    for (int i = 0; i < (channelNumber); i++) {
      logfile.print(channelArray[i]);
      logfile.print(";");
    }
    logfile.println(";");
    logfile.print("Air sat threshold [% air saturation]:;");
    for (int i = 0; i < (channelNumber); i++) {
      logfile.print(airSatThreshold[i]);
      logfile.print(";");
    }
    logfile.println(";");
    logfile.println(";");
    logfile.print("Measurement;Date;Time;Temp_");                  // header row for measurements: Measurement, Date, Time, tankID1, tankID2,...
    logfile.print(tempID);
    logfile.print(";");
    for (int i = 0; i < (channelNumber); i++) {
      logfile.print("DO_");
      logfile.print(tankID[i]);
      logfile.print(";");
    }
    logfile.println();
    logfile.flush();                                  // save data to logfile

    // Print to Serial monitor
    Serial.print("Logfile created: ");
    Serial.println(filename);

    // Print to LCD
    lcd.setCursor(0,1);
    lcd.print(filename);
  }
  else if (!logfile) {                                        // check if logfile was created
    Serial.println("error: couldn't create logfile");
    lcd.setCursor(0, 1);
    lcd.print(".csv failed");
    while (1);                                                // do nothing
  }
}

//# Log dissolved oxygen measurements to SD card #
void writeToSD() {
  if (!SD.exists(filename)) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("error!");
    lcd.setCursor(0, 1);
    lcd.print("no SD");
    Serial.println("error: can't read SD");
    while (1);
  }
  else {
    DateTime now;
    now = RTC.now();                                  // fetch time and date from RTC
    n = n + 1;                                        // increase n by one for each measurement
    logfile.print(n);                                 // print row index n
    logfile.print(";");                               // print a semicolon to separate values
    logfile.print(now.year(), DEC);                   // print date and time of measurement
    logfile.print("/");
    logfile.print(now.month(), DEC);
    logfile.print("/");
    logfile.print(now.day(), DEC);
    logfile.print(";");
    logfile.print(now.hour(), DEC);
    logfile.print(":");
    logfile.print(now.minute(), DEC);
    logfile.print(":");
    logfile.print(now.second(), DEC);
    logfile.print(";");
    logfile.print(tempFloat);
    logfile.print(";");
    for (int k = 0; k < channelNumber; k++) {         // print air saturation measurements for each channel
      logfile.print(DOFloat[k]);
      logfile.print(";");
    }
    logfile.println();
    logfile.flush();                                  // save data to logfile
  }
}


//#######################################################################################
//###                                   Setup                                         ###
//### You may want to get rid of all the status updates that are printed on the LCD   ###
//### and the serial monitor. I use them to inspect what the settings are.            ###
//#######################################################################################

void setup() {
  Serial.begin(19200);
  delay(300);
  Serial.println("-------------- Ardoxy 4 channel control example -------------");
  ardoxy.begin();
    
//# Set up one PID per channel #
  relay1PID.SetMode(AUTOMATIC);
  relay1PID.SetSampleTime(sampleInterval);
  relay1PID.SetOutputLimits(0, windowSize);
  relay2PID.SetMode(AUTOMATIC);
  relay2PID.SetSampleTime(sampleInterval);
  relay2PID.SetOutputLimits(0, windowSize);
  relay3PID.SetMode(AUTOMATIC);
  relay3PID.SetSampleTime(sampleInterval);
  relay3PID.SetOutputLimits(0, windowSize);
  relay4PID.SetMode(AUTOMATIC);
  relay4PID.SetSampleTime(sampleInterval);
  relay4PID.SetOutputLimits(0, windowSize);
  
//# Start LCD display, clear serial buffer #
  lcd.begin(16, 2);
  lcd.setBacklight(WHITE);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("DO ctrl booting.");
  delay(100);

//# Declare output pins for relay operation #
  lcd.clear();
  lcd.print("Relay pins..");
  for (int i = 0; i < channelNumber; i++) {       // declare relay pins as output pins and write HIGH to close magnetic valves
    pinMode(relayPin[i], OUTPUT);
    digitalWrite(relayPin[i], HIGH);
  }
  delay(100);

//# Initialize the real time clock #
  lcd.clear();
  lcd.print("Init RTC...");
  Wire.begin();                                   // initialize real time clock
  if (!RTC.begin()) {                             // check if RTC is initialized
    lcd.setCursor(0, 1);
    Serial.println("RTC failed");
    lcd.print("RTC failed");
    while(1);
  }
  if(setRTC){
    RTC.adjust(DateTime(F(__DATE__), F(__TIME__))); // set rtc to the time this sketch was compiled - ONLY NECESSARY ON FIRST UPLOAD, THEN COMMENT THE LINE OUT AND RE-UPLOAD
  }
  delay(100);

//# Display time on LCD display #
  lcd.clear();
  DateTime now;
  now = RTC.now();                                // fetch time from RTC to display
  lcd.print(now.year(), DEC);
  lcd.print('/');
  lcd.print(now.month(), DEC);
  lcd.print('/');
  lcd.print(now.day(), DEC);
  lcd.setCursor(0,1);
  lcd.print(now.hour(), DEC);
  lcd.print(':');
  lcd.print(now.minute(), DEC);
  lcd.print(':');
  lcd.print(now.second(), DEC);
  delay(2000);                                    // long delay to check the date on the display

//# Initialize SD card #
  lcd.clear();
  lcd.print("Check SD...");
  if (!SD.begin(chipSelect)) {                    // check if SD card is present and can be read
    Serial.println("Card failed, or not present. Please insert SD card and reboot the system.");
    lcd.setCursor(0, 1);
    lcd.print("SD failed");
    while (1);
  }
  delay(500);

//# Create a new logfile #
  lcd.clear();
  lcd.print("Create .csv...");
  createLogfile();
  delay(500);

//# Display control settings 
  Serial.print("Start of measurement cycles. Measurement interval set at ");
  Serial.print(sampleInterval / 1000);
  Serial.println(" seconds.");
  Serial.println("-------------------------------------------------------------");
  lcd.clear();
  lcd.print("Ready");
  lcd.setCursor(0, 1);
  lcd.print("Interval: ");
  lcd.print(sampleInterval / 1000);
  lcd.print("s");
  delay(2000);
  lcd.clear();
  lcd.print("---Thresholds---");
  delay(1000);
  lcd.clear();
  for (int i = 0; i < channelNumber; i++){
    if (i == 2){
      lcd.setCursor(0, 1);
    }
    else if (i == 4){
      delay(2000);
      lcd.clear();
    }
    else if (i == 6){
      lcd.setCursor(0, 1);
    }
    lcd.print(airSatThreshold[i]);
    lcd.print(" ");
  }
  delay(2000); 
  lcd.clear();
  
  // Set lastday for saving every day
  lastday = now.day();
}

//#######################################################################################
//###                                 Main loop                                       ###
//###               Congratulations, you made it to the main loop.                    ###
//### I inserted delays between every subfunctions to ensure their completion before  ###
//### the program moves to the next step. Some of these are crucial as the sensor     ###
//### needs time to complete measurements. Also with serial communication, we want to ###
//### make sure that everything has been completely sent and received.                ###
//#######################################################################################

void loop() {
  loopStart = millis();                                       // start timer of loop
  DateTime now;
  now = RTC.now();  
  curday = now.day();
  if (curday != lastday){                                     // create a new logfile for every day
    logfile.close();
    delay(100);
    createLogfile();
    lastday = curday;
  }
  lcd.clear();
  cursorX = 0;
  lcd.setCursor(0, 0);
  lcd.print("Measurement...");
  for (int i = 0; i < channelNumber; i++) {                   // loop that iterates through every channel
    DOSum = 0;                                                // reset summing variable for measurements
    DOInt = 0;                                                // reset value variable
    lcd.setCursor(cursorX, 1);
    lcd.print(".");
    cursorX += 1;
    activeChannel = channelArray[i];                          // declare channel to be measured for serial commands
    sprintf(DOReadCom, "REA %d 3 4\r", activeChannel);        // insert channel in readout command
    for (int j = 0; j < samples; j++) {                       // oversampling loop 
      check = 0;                                              // reset check variable
      while(!check){                                          // as long as check doesn't come out positive, try to measure and reconnect
        check = ardoxy.measureSeq(i+1);
        if(!check){
          Serial.println("Com error. Restarting serial communication.");
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Com error!");
          ardoxy.end();
          delay(1000);
          ardoxy.begin();
          delay(2000);
        }
      }

      DOInt = ardoxy.readout(DOReadCom);              // read out DO value
      if (i == 0){                                    // with the first measurement, also read out temperature
        tempInt = ardoxy.readout(tempReadCom);
        tempFloat = tempInt / 1000.00;
      }
      DOSum = DOSum + DOInt;                          // sum up air saturation readings for consecutive samples
    }
    DOFloat[i] = DOSum / (samples * 1000.00);         // create floating point number for logging, display, etc. Results in 0 if there's a communication error
  }
  showNewData();                                      // display measurement on LCD
  delay(100);
  writeToSD();                                        // log to SD card
  delay(100);
  DOCheck();                                          // check if low DO threshold is crossed
  if (lowDO){
    Serial.print("low DO! Measured value: ");         // halt program for 20 min to let DO value recover... more code can be inserted here to open an air valve or to light an alarm LED
    Serial.println(lowDOValue);
    lcd.clear();
    lcd.print("low DO at ");
    lcd.print(lowDOTank);
    lcd.setCursor(0,1);
    lcd.print("measured: ");
    lcd.print(lowDOValue);
    delay(1200*1000UL);
    lowDO = false;
  } else { 
    toggleRelay();                                    // operate relays to open solenoid valves
    elapsed = millis() - loopStart;                   // calculate duration of loop
    if (elapsed > sampleInterval) {                   // adjust measurement interval if loop takes longer than measurement interval
      sampleInterval = elapsed;
      lcd.clear();
      lcd.print("Short int.");
      lcd.setCursor(0, 1);
      lcd.print("New: ");
      lcd.print(sampleInterval/1000);
      lcd.print("sec");
    } else {
      delay(sampleInterval - elapsed);                // adjust delay so that loop duration equals measurement interval
    }
  }
}
