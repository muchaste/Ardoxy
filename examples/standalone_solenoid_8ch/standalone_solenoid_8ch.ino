/*
  Ardoxy example — standalone_solenoid_8ch

  Regulate DO on up to 8 channels across TWO FireStingO2 sensors using solenoid valves.
  DO is displayed on LCD and stored on SD card.
  Oxygen sensors are calibrated using the Pyro Oxygen Logger Software.
  A flyback diode should be connected to the +/- poles of each valve.

  You must define:
  - s1ChannelNumber / s2ChannelNumber — channels used on each FireSting sensor
  - channelArray — per-sensor channel port number for each logical channel (in order)
  - sampleInterval, tankID, tempID, relayPin, airSatThreshold, lowDOThreshold
  - acclStartDates, decreaseDays, acclDur, acclThreshold, airSatProgression
  - PID tunings (Kp, Ki, Kd)
  - setRTC (set to 1 once to synchronise RTC, then reupload with 0)

  The circuit:
  - Arduino Mega (Serial1 pins 18/19, Serial2 pins 16/17)
  - Adafruit datalogger shield (SD card + RTC PCF8523)
  - Two FireStingO2 sensors:
      FireSting 1 — 7-pin connector: Pin1=GND, Pin2=5V, Pin4=RX1(19), Pin5=TX1(18)
      FireSting 2 — 7-pin connector: Pin1=GND, Pin2=5V, Pin4=RX2(17), Pin5=TX2(16)
  - Solenoid valves on relay module (one per channel)

  by Stefan Mucha
*/

#include <Ardoxy.h>
#include <PID_v1.h>
#include <SdFat.h>
#include <Wire.h>
#include "RTClib.h"
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
#define WHITE 0x7


//#######################################################################################
//###                              General settings                                   ###
//#######################################################################################

//# Sensor channel layout #
const int sensorNumber    = 2;          // number of FireSting sensors
const int s1ChannelNumber = 2;          // channels used on FireSting 1
const int s2ChannelNumber = 3;          // channels used on FireSting 2
const int channelNumber   = s1ChannelNumber + s2ChannelNumber;  // total logical channels

// Per-sensor channel port numbers in logical channel order
// e.g. logical channels 0,1 come from FireSting1 ports 2,4 and channels 2,3,4 from FireSting2 ports 1,3,4
int channelArray[channelNumber] = {1, 2, 1, 2, 3};            // ***ADAPT TO YOUR WIRING***

//# Experimental conditions #
long int samples = 1;                                          // oversampling count (average N readings)
char tankID[8][8] = {"A", "B", "C", "D", "E", "", "", ""};    // IDs for each logical channel (max 8 chars)
const int tempID[sensorNumber] = {0, s1ChannelNumber};        // logical channel index where each sensor's temperature is read

long sampleInterval = 60 * 1000UL;                            // measurement and control interval (ms)
double airSatThreshold[8] = {100.0, 100.0, 100.0, 100.0, 15.0, 0, 0, 0};  // DO setpoint per channel (% air sat)
double lowDOThreshold = 7.0;                                   // threshold that triggers the error flag

//# Set the RTC? #
const int setRTC = 1;                                          // upload once with 1 to set RTC, then reupload with 0

//# Pin definitions #
int relayPin[8] = {46, 48, 50, 52, 22, 24, 26, 28};           // relay pins per logical channel ***ADAPT***
const int chipSelect = 10;                                     // SD card CS pin (Adafruit shield: 10)


//#######################################################################################
//###                           Acclimation scheduling                                ###
//#######################################################################################

// Phase 1: pre-decrease | Phase 2: DO decrease | Phase 3: acclimation | Phase 4: post
const int decreaseDays = 7;
const int acclDur      = 75;
const double acclThreshold = 15.0;
const double airSatProgression[7] = {70.0, 60.0, 50.0, 40.0, 30.0, 20.0, 15.0};

const int acclStartDates[3][8] = {       // per-channel start date: row0=day, row1=month, row2=year
  {1,    1,    1,    1,    1,    1,    1,    1   },
  {1,    1,    1,    1,    1,    1,    1,    1   },
  {2026, 2026, 2026, 2026, 2026, 2026, 2026, 2026}};

int phaseIdx[8] = {1, 1, 1, 1, 1, 1, 1, 1};  // current phase per channel
int acclDays[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // elapsed hypoxia days per channel


//#######################################################################################
//###                                PID settings                                     ###
//#######################################################################################

double Kp[8] = {10, 10, 10, 10, 10, 10, 10, 10};
double Ki[8] = {1,  1,  1,  1,  1,  1,  1,  1 };
double Kd[8] = {1,  1,  1,  1,  1,  1,  1,  1 };
long int windowSize = 75;               // recomputed in setup() as round(sampleInterval / (200*2))


//#######################################################################################
//###                           Requisite variables                                   ###
//#######################################################################################

//# State / error tracking #
boolean errorDO = false;
int activeChannel = 1;
int check;
int errorCount = 0;
void(* resetFunc)(void) = 0;

//# Timing #
unsigned long loopStart, elapsed;
int curday, lastday;

//# SD / RTC #
RTC_PCF8523 RTC;
SdFs SD;
FsFile logfile;
char filename[21];
uint32_t n = 0;

//# Oxygen measurement #
char DOReadCom[11]  = "REA 1 3 4\r";
char tempReadCom[11]= "REA 1 3 5\r";
long DOInt, tempInt;
long DOSum;
double DOFloat[8], tempFloat[2];
double lowDOValue;
char   lowDOTank[8];

Ardoxy FireSting1(Serial1);
Ardoxy FireSting2(Serial2);

//# Relay operation #
double relayArray[3][8];
double Output[8];

//# Hardcoded PID instances (one per logical channel, max 8) #
PID relay1PID(&DOFloat[0], &Output[0], &airSatThreshold[0], Kp[0], Ki[0], Kd[0], REVERSE);
PID relay2PID(&DOFloat[1], &Output[1], &airSatThreshold[1], Kp[1], Ki[1], Kd[1], REVERSE);
PID relay3PID(&DOFloat[2], &Output[2], &airSatThreshold[2], Kp[2], Ki[2], Kd[2], REVERSE);
PID relay4PID(&DOFloat[3], &Output[3], &airSatThreshold[3], Kp[3], Ki[3], Kd[3], REVERSE);
PID relay5PID(&DOFloat[4], &Output[4], &airSatThreshold[4], Kp[4], Ki[4], Kd[4], REVERSE);
PID relay6PID(&DOFloat[5], &Output[5], &airSatThreshold[5], Kp[5], Ki[5], Kd[5], REVERSE);
PID relay7PID(&DOFloat[6], &Output[6], &airSatThreshold[6], Kp[6], Ki[6], Kd[6], REVERSE);
PID relay8PID(&DOFloat[7], &Output[7], &airSatThreshold[7], Kp[7], Ki[7], Kd[7], REVERSE);

PID* allPIDs[8] = {&relay1PID, &relay2PID, &relay3PID, &relay4PID,
                   &relay5PID, &relay6PID, &relay7PID, &relay8PID};

//# LCD Display #
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
int airSatLCD = 0;
int cursorX   = 0;


//#######################################################################################
//###                           Requisite Functions                                   ###
//#######################################################################################

void showNewData() {
  lcd.clear();
  lcd.setCursor(0, 0);
  DateTime now;
  now = RTC.now();
  Serial.print(now.year(), DEC); Serial.print('/');
  Serial.print(now.month(), DEC); Serial.print('/');
  Serial.print(now.day(), DEC); Serial.print(" - ");
  Serial.print(now.hour(), DEC); Serial.print(':');
  Serial.print(now.minute(), DEC); Serial.print(':');
  Serial.println(now.second(), DEC);

  for (int s = 0; s < sensorNumber; s++) {
    Serial.print(tankID[tempID[s]]);
    Serial.print(": ");
    Serial.print(tempFloat[s]);
    Serial.println(" degC");
  }

  for (int k = 0; k < channelNumber; k++) {
    if (k == 4) { lcd.setCursor(0, 1); }
    airSatLCD = int(lround(DOFloat[k]));
    lcd.print(airSatLCD);
    lcd.print(" ");
    Serial.print(tankID[k]);
    Serial.print(": ");
    Serial.print(DOFloat[k]);
    Serial.println("% air sat");
  }
}

void DOCheck() {
  for (int k = 0; k < channelNumber; k++) {
    if (DOFloat[k] < lowDOThreshold) {
      errorDO = true;
      lowDOValue = DOFloat[k];
      strcpy(lowDOTank, tankID[k]);
    }
  }
}

void toggleRelay() {
  for (int k = 0; k < channelNumber; k++) {
    allPIDs[k]->Compute();
  }

  for (int k = 0; k < channelNumber; k++) {
    relayArray[0][k] = relayPin[k];
    relayArray[1][k] = double(int(Output[k]) * 200.00);
    relayArray[2][k] = DOFloat[k];
  }

  double temp[3];
  for (int k = 0; k < channelNumber - 1; k++) {
    for (int m = k + 1; m < channelNumber; m++) {
      if (relayArray[1][m] < relayArray[1][k]) {
        temp[0] = relayArray[0][k]; temp[1] = relayArray[1][k]; temp[2] = relayArray[2][k];
        relayArray[0][k] = relayArray[0][m]; relayArray[1][k] = relayArray[1][m]; relayArray[2][k] = relayArray[2][m];
        relayArray[0][m] = temp[0];          relayArray[1][m] = temp[1];          relayArray[2][m] = temp[2];
      }
    }
  }

  for (int k = 0; k < channelNumber; k++) {
    if (relayArray[1][k] > 0) {
      for (int m = k; m < channelNumber; m++) {
        digitalWrite(relayArray[0][m], LOW);
      }
      if (k == 0) {
        delay(relayArray[1][k]);
        digitalWrite(relayArray[0][k], HIGH);
      } else {
        delay(relayArray[1][k] - relayArray[1][k - 1]);
        digitalWrite(relayArray[0][k], HIGH);
      }
    }
  }
}

void createLogfile() {
  DateTime now;
  now = RTC.now();
  sprintf(filename, "%4d_%2d_%2d_%2d_%2d.csv", now.year(), now.month(), now.day(), now.hour(), now.minute());
  Serial.println(filename);
  delay(100);

  logfile = SD.open(filename, FILE_WRITE);
  if (logfile) {
    logfile.println(";");
    logfile.print("Date:;"); logfile.print(now.year(), DEC); logfile.print("/");
    logfile.print(now.month(), DEC); logfile.print("/"); logfile.print(now.day(), DEC); logfile.print(";");
    logfile.print("Time:;"); logfile.print(now.hour(), DEC); logfile.print(":");
    logfile.print(now.minute(), DEC); logfile.print(":"); logfile.println(now.second(), DEC);
    logfile.print(";Measurement interval [sec]:;"); logfile.print(sampleInterval / 1000);
    logfile.print(";Active channels:;"); logfile.print(channelNumber);
    for (int s = 0; s < sensorNumber; s++) {
      logfile.print(";Temp Sensor "); logfile.print(s); logfile.print(":;");
      logfile.print(tankID[tempID[s]]);
    }
    logfile.println();
    logfile.print("Tank ID:;");
    for (int i = 0; i < channelNumber; i++) { logfile.print(tankID[i]); logfile.print(";"); }
    logfile.println(";");
    logfile.print("Channel:;");
    for (int i = 0; i < channelNumber; i++) { logfile.print(channelArray[i]); logfile.print(";"); }
    logfile.println(";");
    logfile.print("Air sat threshold [% air sat]:;");
    for (int i = 0; i < channelNumber; i++) { logfile.print(airSatThreshold[i]); logfile.print(";"); }
    logfile.println(";");
    logfile.print("Start acclimation:;");
    for (int i = 0; i < channelNumber; i++) {
      logfile.print(acclStartDates[2][i]); logfile.print("/");
      logfile.print(acclStartDates[1][i]); logfile.print("/");
      logfile.print(acclStartDates[0][i]); logfile.print(";");
    }
    logfile.println(";");
    logfile.println(";");
    logfile.print("Measurement;Date;Time;");
    for (int s = 0; s < sensorNumber; s++) {
      logfile.print("Temp_"); logfile.print(tankID[tempID[s]]); logfile.print(";");
    }
    for (int i = 0; i < channelNumber; i++) {
      logfile.print("DO_"); logfile.print(tankID[i]); logfile.print(";");
      logfile.print("days_hyp_"); logfile.print(tankID[i]); logfile.print(";");
    }
    logfile.println();
    logfile.flush();

    Serial.print("Logfile created: "); Serial.println(filename);
    lcd.setCursor(0, 1); lcd.print(filename);
  } else {
    Serial.println("error: couldn't create logfile");
    lcd.setCursor(0, 1); lcd.print(".csv failed");
    while (1);
  }
}

void writeToSD() {
  if (!SD.exists(filename)) {
    lcd.clear(); lcd.setCursor(0, 0); lcd.print("error!");
    lcd.setCursor(0, 1); lcd.print("no SD");
    Serial.println("error: can't read SD");
    while (1);
  }
  DateTime now; now = RTC.now();
  n++;
  logfile.print(n); logfile.print(";");
  logfile.print(now.year(), DEC); logfile.print("/");
  logfile.print(now.month(), DEC); logfile.print("/"); logfile.print(now.day(), DEC); logfile.print(";");
  logfile.print(now.hour(), DEC); logfile.print(":"); logfile.print(now.minute(), DEC); logfile.print(":");
  logfile.print(now.second(), DEC); logfile.print(";");
  for (int s = 0; s < sensorNumber; s++) { logfile.print(tempFloat[s]); logfile.print(";"); }
  for (int k = 0; k < channelNumber; k++) {
    logfile.print(DOFloat[k]); logfile.print(";");
    logfile.print(acclDays[k]); logfile.print(";");
  }
  logfile.println();
  logfile.flush();
}

void writeState() {
  FsFile stateFile;
  DateTime now; now = RTC.now();
  SD.remove("STATE.TXT");
  stateFile = SD.open("STATE.TXT", FILE_WRITE);
  if (stateFile) {
    stateFile.print(filename);       stateFile.print(",");
    stateFile.print(n);              stateFile.print(",");
    stateFile.print(sampleInterval); stateFile.print(",");
    stateFile.print(now.year());     stateFile.print(",");
    stateFile.print(now.month());    stateFile.print(",");
    stateFile.print(now.day());      stateFile.print(",");
    stateFile.print(now.hour());     stateFile.print(",");
    stateFile.print(now.minute());   stateFile.print(",");
    stateFile.print(now.second());   stateFile.print(",");
    for (int s = 0; s < sensorNumber; s++) { stateFile.print(tempFloat[s]); stateFile.print(","); }
    for (int i = 0; i < channelNumber; i++) { stateFile.print(DOFloat[i]); stateFile.print(","); }
    for (int i = 0; i < channelNumber; i++) {
      stateFile.print(acclDays[i]);
      if (i < channelNumber - 1) stateFile.print(",");
    }
    stateFile.println();
    stateFile.close();
  }
}

bool readState() {
  if (!SD.exists("STATE.TXT")) { return false; }
  FsFile stateFile;
  stateFile = SD.open("STATE.TXT", O_RDONLY);
  if (!stateFile) { return false; }
  char stateBuffer[200];
  int len = stateFile.read(stateBuffer, sizeof(stateBuffer) - 1);
  stateFile.close();
  if (len <= 0) { return false; }
  stateBuffer[len] = '\0';

  int savedYear, savedMonth, savedDay;
  char* tok;
  tok = strtok(stateBuffer, ","); if (!tok) { return false; }
  strncpy(filename, tok, sizeof(filename) - 1); filename[sizeof(filename) - 1] = '\0';
  tok = strtok(NULL, ","); if (!tok) { return false; } n             = (uint32_t)atol(tok);
  tok = strtok(NULL, ","); if (!tok) { return false; } sampleInterval = atol(tok);
  tok = strtok(NULL, ","); if (!tok) { return false; } savedYear  = atoi(tok);
  tok = strtok(NULL, ","); if (!tok) { return false; } savedMonth = atoi(tok);
  tok = strtok(NULL, ","); if (!tok) { return false; } savedDay   = atoi(tok);
  tok = strtok(NULL, ","); if (!tok) { return false; }  // hour   - skip
  tok = strtok(NULL, ","); if (!tok) { return false; }  // minute - skip
  tok = strtok(NULL, ","); if (!tok) { return false; }  // second - skip
  for (int s = 0; s < sensorNumber; s++) {
    tok = strtok(NULL, ","); if (!tok) { return false; } tempFloat[s] = atof(tok);
  }
  for (int i = 0; i < channelNumber; i++) {
    tok = strtok(NULL, ","); if (!tok) { return false; } DOFloat[i] = atof(tok);
  }
  for (int i = 0; i < channelNumber; i++) {
    tok = strtok(NULL, ",\r\n"); if (!tok) { return false; } acclDays[i] = atoi(tok);
  }

  DateTime now = RTC.now();
  return (savedYear == (int)now.year() && savedMonth == (int)now.month() && savedDay == (int)now.day());
}


//#######################################################################################
//###                                   Setup                                         ###
//#######################################################################################

void setup() {
  Serial.begin(19200);
  delay(300);
  Serial.println("------ Ardoxy 8-channel standalone control ------");
  FireSting1.begin();
  delay(500);
  FireSting2.begin();

  windowSize = (long)round((double)sampleInterval / (200.0 * 2));

//# Set up PIDs for each active channel #
  for (int i = 0; i < channelNumber; i++) {
    allPIDs[i]->SetMode(AUTOMATIC);
    allPIDs[i]->SetSampleTime(sampleInterval);
    allPIDs[i]->SetOutputLimits(0, windowSize);
  }

//# LCD #
  lcd.begin(16, 2);
  lcd.setBacklight(WHITE);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("DO ctrl booting.");
  delay(100);

//# Relay pins #
  lcd.clear(); lcd.print("Relay pins..");
  for (int i = 0; i < channelNumber; i++) {
    pinMode(relayPin[i], OUTPUT);
    digitalWrite(relayPin[i], HIGH);
  }
  delay(100);

//# RTC #
  lcd.clear(); lcd.print("Init RTC...");
  Wire.begin();
  if (!RTC.begin()) {
    lcd.setCursor(0, 1); lcd.print("RTC failed");
    Serial.println("RTC failed");
    while (1);
  }
  if (setRTC) {
    RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  delay(100);

  lcd.clear();
  DateTime now; now = RTC.now();
  lcd.print(now.year(), DEC); lcd.print('/'); lcd.print(now.month(), DEC); lcd.print('/'); lcd.print(now.day(), DEC);
  lcd.setCursor(0, 1);
  lcd.print(now.hour(), DEC); lcd.print(':'); lcd.print(now.minute(), DEC); lcd.print(':'); lcd.print(now.second(), DEC);
  delay(2000);

//# SD card #
  lcd.clear(); lcd.print("Check SD...");
  if (!SD.begin(chipSelect)) {
    Serial.println("SD failed.");
    lcd.setCursor(0, 1); lcd.print("SD failed");
    while (1);
  }
  delay(500);

//# Logfile or recovery #
  lcd.clear();
  bool recovered = readState();
  if (recovered) {
    logfile = SD.open(filename, FILE_WRITE);
    if (!logfile) {
      lcd.print("Recovery failed");
      delay(1000);
      lcd.clear(); lcd.print("Create .csv...");
      createLogfile();
    } else {
      DateTime restartTime = RTC.now();
      logfile.print("RESTART;");
      logfile.print(restartTime.year()); logfile.print("/");
      logfile.print(restartTime.month()); logfile.print("/"); logfile.print(restartTime.day()); logfile.print(";");
      logfile.print(restartTime.hour()); logfile.print(":"); logfile.print(restartTime.minute()); logfile.print(":");
      logfile.print(restartTime.second()); logfile.print(";");
      for (int s = 0; s < sensorNumber; s++) { logfile.print(tempFloat[s]); logfile.print(";"); }
      for (int i = 0; i < channelNumber; i++) { logfile.print(DOFloat[i]); logfile.print(";"); }
      logfile.println();
      logfile.flush();
      lcd.print("Recovery!");
      lcd.setCursor(0, 1); lcd.print(filename);
      Serial.println("State restored. Resuming logfile.");
      Serial.print("Restored n="); Serial.println(n);
    }
  } else {
    lcd.print("Create .csv...");
    createLogfile();
  }
  delay(500);

  Serial.print("Interval: "); Serial.print(sampleInterval / 1000); Serial.println(" s");
  lcd.clear(); lcd.print("Ready"); lcd.setCursor(0, 1);
  lcd.print("Int: "); lcd.print(sampleInterval / 1000); lcd.print("s");
  delay(2000);
  lcd.clear(); lcd.print("---Thresholds---"); delay(1000); lcd.clear();
  for (int i = 0; i < channelNumber; i++) {
    if (i == 2) { lcd.setCursor(0, 1); }
    else if (i == 4) { delay(2000); lcd.clear(); }
    else if (i == 6) { lcd.setCursor(0, 1); }
    lcd.print(airSatThreshold[i]); lcd.print(" ");
  }
  delay(2000); lcd.clear();

  lastday = now.day();

  // Initialise per-channel acclimation phase from RTC date
  for (int i = 0; i < channelNumber; i++) {
    acclDays[i] = Ardoxy::calcDays(acclStartDates[0][i], acclStartDates[1][i], acclStartDates[2][i], now.day(), now.month(), now.year());
    if (acclDays[i] == 0) {
      phaseIdx[i] = 1;
    } else if (acclDays[i] <= decreaseDays) {
      phaseIdx[i] = 2;
      airSatThreshold[i] = airSatProgression[acclDays[i] - 1];
    } else if (acclDays[i] <= decreaseDays + acclDur) {
      phaseIdx[i] = 3;
      airSatThreshold[i] = acclThreshold;
    } else {
      phaseIdx[i] = 4;
    }
  }
}


//#######################################################################################
//###                                 Main loop                                       ###
//#######################################################################################

void loop() {
  loopStart = millis();
  DateTime now; now = RTC.now();
  curday = now.day();
  errorDO = false;

  // Update per-channel acclimation phase and threshold (before lastday is updated)
  for (int i = 0; i < channelNumber; i++) {
    if (phaseIdx[i] == 1) {
      acclDays[i] = Ardoxy::calcDays(acclStartDates[0][i], acclStartDates[1][i], acclStartDates[2][i], curday, now.month(), now.year());
    } else if (phaseIdx[i] >= 2 && curday != lastday) {
      acclDays[i]++;
    }
    if (acclDays[i] == 0) {
      phaseIdx[i] = 1;
    } else if (acclDays[i] <= decreaseDays) {
      phaseIdx[i] = 2;
      airSatThreshold[i] = airSatProgression[acclDays[i] - 1];
    } else if (acclDays[i] <= decreaseDays + acclDur) {
      phaseIdx[i] = 3;
      airSatThreshold[i] = acclThreshold;
    } else {
      phaseIdx[i] = 4;
      airSatThreshold[i] = 100.0;
    }
  }

  if (curday != lastday) {
    logfile.close(); delay(100);
    createLogfile();
    lastday = curday;
  }

  lcd.clear(); cursorX = 0; lcd.setCursor(0, 0); lcd.print("Measurement...");

  for (int i = 0; i < channelNumber; i++) {
    DOSum = 0; DOInt = 0;
    lcd.setCursor(cursorX, 1); lcd.print("."); cursorX++;

    activeChannel = channelArray[i];
    sprintf(DOReadCom, "REA %d 3 4\r", activeChannel);

    for (int j = 0; j < samples; j++) {
      check = 0;
      while (!check) {
        // Route measurement to correct FireSting
        if (i < s1ChannelNumber) {
          check = FireSting1.measureSeq(activeChannel);
        } else {
          check = FireSting2.measureSeq(activeChannel);
        }
        if (!check) {
          Serial.println("Com error. Restarting serial communication.");
          lcd.clear(); lcd.setCursor(0, 0); lcd.print("Com error!");
          errorCount++;
          if (errorCount >= 50) { resetFunc(); }
          if (i < s1ChannelNumber) { FireSting1.end(); delay(1000); FireSting1.begin(); }
          else                     { FireSting2.end(); delay(1000); FireSting2.begin(); }
          delay(2000);
        }
      }

      // Read DO and route temperature read to the correct sensor
      if (i < s1ChannelNumber) {
        DOInt = FireSting1.readout(DOReadCom);
        if (i == 0) {                              // temperature on first channel of FireSting1
          tempInt = FireSting1.readout(tempReadCom);
          tempFloat[0] = tempInt / 1000.0;
        }
      } else {
        DOInt = FireSting2.readout(DOReadCom);
        if (i == s1ChannelNumber) {                // temperature on first channel of FireSting2
          tempInt = FireSting2.readout(tempReadCom);
          tempFloat[1] = tempInt / 1000.0;
        }
      }
      DOSum += DOInt;
    }

    DOFloat[i] = DOSum / (samples * 1000.0);
    if (DOFloat[i] == 34276.94) {                  // known bad-read sentinel from PyroScience firmware
      DOFloat[i] = airSatThreshold[i];
      errorDO = true;
    }
  }

  showNewData();
  delay(100);
  writeToSD();
  delay(100);
  writeState();
  DOCheck();

  if (errorDO) {
    Serial.print("Error/low DO! Measured value: ");
    Serial.println(lowDOValue);
    lcd.clear(); lcd.print("err DO at "); lcd.print(lowDOTank);
    lcd.setCursor(0, 1); lcd.print("val: "); lcd.print(lowDOValue);
    errorCount++;
    if (errorCount >= 50) { resetFunc(); }
  } else {
    toggleRelay();
    elapsed = millis() - loopStart;
    if (elapsed > sampleInterval) {
      sampleInterval = elapsed;
      lcd.clear(); lcd.print("Short int.");
      lcd.setCursor(0, 1); lcd.print("New: "); lcd.print(sampleInterval / 1000); lcd.print("s");
    } else {
      delay(sampleInterval - elapsed);
    }
  }
}
