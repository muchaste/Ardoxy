/*
  ardoxy_standalone.ino  — v2
  Author: Stefan Mucha with Claude Sonnet 4.6
  Cite: Mucha S (2025) J Exp Biol 228(1):jeb249207. https://doi.org/10.1242/jeb.249207
  Ardoxy Standalone Sketch — upload once, configure via ardoxy_gui.py (Standalone Mode),
  then run autonomously with SD card logging and LCD display.
  Power-outage recovery is automatic via STATE.TXT on the SD card.
  Each channel has its own independent mode and optional start time.

  Hardware:
    - Arduino MEGA2560
    - FireSting 1 on Serial1 (MEGA pins 18 RX1 / 19 TX1)
    - FireSting 2 on Serial2 (MEGA pins 17 RX2 / 16 TX2)  — optional, 2-sensor mode
    - Adafruit Datalogger Shield  (SD on CS=10, RTC PCF8523 via I2C)
    - Adafruit RGB LCD Shield     (16×2, I2C via MCP23017)
    - Relay module on digital output pins (default: 46, 48, 50, 52, 22, 24, 26, 28)

  Protocol (USB serial, 19200 baud, newline-terminated):
    PC -> Arduino  — shared hardware config —
      CFG:NCHANNELS:<1-8>
      CFG:SENSORS:<1|2>              number of FireSting sensors (default 1)
      CFG:S1CHANNELS:<n>             channels on sensor 1; required when SENSORS=2
      CFG:RELAY:<ch>:<pin>           ch = 0-based
      CFG:INTERVAL:<ms>
      CFG:TANKID:<ch>:<id>           up to 6 chars
      CFG:KP:<ch>:<float>            per-channel valve PID gains (ch = 0-based)
      CFG:KI:<ch>:<float>
      CFG:KD:<ch>:<float>

    PC -> Arduino  — per-channel mode config —
      CFG:CH:<ch>:MODE:<MEASURE|SETPOINT|SEQUENCE>
      CFG:CH:<ch>:START:<Y>:<M>:<D>:<h>:<m>:<s>   (0:0:0:0:0:0 = start immediately)
      CFG:CH:<ch>:SETPOINT:<float>   SETPOINT mode target DO
      CFG:CH:<ch>:DURATION:<minutes> SETPOINT duration in minutes
      CFG:CH:<ch>:NPHASES:<n>        number of phases (1..MAX_PHASES)
      CFG:CH:<ch>:PHASE:<idx>:<sp>:<dur_d>:<dur_h>:<dur_m>:<type>[:<params>]
        type: h=hold  c=change  p=pause  d=daily-cycle
        startDO (>0) required when type == 'c'; min_sp/max_sp/peak_h required when type == 'd'

    PC -> Arduino  — commands —
      CMD:START
      CMD:STOP
      CMD:PAUSE
      CMD:RESUME
      CMD:STATUS
      CMD:SAVECONFIG                 write current config to CONFIG.TXT on SD
      CMD:TESTPIN:<pin>:<0|1>        open (1) or close (0) relay by pin nr; blocked while RUNNING
      CMD:READCONFIG                 emit config (loads SD first if IDLE; uses in-memory if CONFIGURED)
      CMD:RECOVER                    resume experiment from STATE.TXT; blocked if IDLE or RUNNING
      CMD:LISTFILES                  list all .csv log files on SD with file sizes
      CMD:SENDFILE:<filename>        stream a log file over serial as FLINE: rows
      CMD:SETRTC:<Y>:<M>:<D>:<h>:<m>:<s>

    Arduino -> PC:
      ACK:OK
      ACK:ERR:<msg>
      STATUS:<IDLE|CONFIGURED|RUNNING|PAUSED>
      STATUS:CH:<ch>:<statusStr>:<phaseIdx>:<do>:<sp>
        statusStr: MEASURE | SETPOINT | SEQUENCE | WAITING-MEASURING | DONE
      DATA:<elapsed_s>,<do_ch0[,do_ch1...]>,<temp1>[,<temp2>],<out_ms_ch0[,...]>
      MSG:<text>
      DONE

  SD card files:
    CONFIG.TXT  — saved config (key=value); auto-loaded on boot if no serial input
    STATE.TXT   — last-known experiment state; auto-restored on boot
    YYYY_MM_DD_HH_MM.csv  — semicolon-delimited measurement log (new file per day)

  Boot behaviour:
    1. Wait 10 s for any serial byte (GUI config session).
    2. If no serial input: load CONFIG.TXT → restore STATE.TXT (power-outage recovery)
       or start fresh. If CONFIG.TXT is absent, wait for serial config.

  Note on N2-only control: DO can only be decreased by bubbling N2; passive increase
  occurs through mixing. REVERSE PID ensures output=0 when DO < setpoint (no N2).
*/

#include <Ardoxy.h>
#include <PID_v1.h>
#include <SdFat.h>
#include <Wire.h>
#include "RTClib.h"
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
#include <avr/wdt.h>

#define WHITE        0x7
#define MAX_CHANNELS 8
#define MAX_PHASES   6          // phases per channel (was 10 global in v1)
#define RECV_BUF     96
#define CHIP_SELECT  10         // Adafruit datalogger shield
#define LCD_PAGE_INTERVAL 2000UL  // ms between automatic LCD page changes
#define LCD_REFRESH_MS      2000UL   // ms between LCD content redraws

// ─── hardware instances ───────────────────────────────────────────────────────
Ardoxy              ardoxy(Serial1);          // FireSting 1 on Serial1 (MEGA pins 18/19)
Ardoxy              ardoxy2(Serial2);         // FireSting 2 on Serial2 (MEGA pins 16/17)
RTC_PCF8523         RTC;
SdFs                SD;
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

// ─── state machine ────────────────────────────────────────────────────────────
typedef enum { IDLE, CONFIGURED, RUNNING, PAUSED } State;
State state = IDLE;

// ─── per-channel mode ─────────────────────────────────────────────────────────
enum ChMode : uint8_t { CH_MEASURE = 0, CH_SETPOINT = 1, CH_SEQUENCE = 2 };

// ─── shared hardware config ───────────────────────────────────────────────────
int    nChannels                 = 1;
int    nSensors                  = 1;
int    s1Channels                = 1;
int    relayPins[MAX_CHANNELS]   = {23, 25, 27, 29, 31, 33, 35, 37};
long   sampInterval              = 30000UL;
float  Kp[MAX_CHANNELS]          = {10, 10, 10, 10, 10, 10, 10, 10};
float  Ki[MAX_CHANNELS]          = { 1,  1,  1,  1,  1,  1,  1,  1};
float  Kd[MAX_CHANNELS]          = { 0,  0,  0,  0,  0,  0,  0,  0};
char   tankID[MAX_CHANNELS][7]   = {"CH1","CH2","CH3","CH4","CH5","CH6","CH7","CH8"};

// ─── per-channel config ───────────────────────────────────────────────────────
ChMode   chMode[MAX_CHANNELS];                       // mode per channel
uint32_t chStart[MAX_CHANNELS];                      // RTC unixtime to begin (0 = immediate)
float    chSetpoint[MAX_CHANNELS];                   // SETPOINT mode target DO
long     chDurationMin[MAX_CHANNELS];                // SETPOINT duration in minutes
uint32_t chSetpointEndUnix[MAX_CHANNELS];            // computed at CMD:START
byte     chNPhases[MAX_CHANNELS];                    // number of phases per channel
byte     chPhaseIdx[MAX_CHANNELS];                   // current phase index per channel
uint32_t chCurrentPhaseEndUnix[MAX_CHANNELS];        // rolling end of current phase
bool     chDone[MAX_CHANNELS];                       // channel finished → passive MEASURE
bool     chActive[MAX_CHANNELS];                     // false until chStart time arrives (WAITING)
float    chPhaseStartDO[MAX_CHANNELS];               // DO at start of 'c' (change) phase
byte     chFault[MAX_CHANNELS];                      // 0=OK  1=CAL error  2=PRB disconnected

// ─── per-channel phase arrays (2-D: [channel][phase]) ─────────────────────────
float    chPhaseSP[MAX_CHANNELS][MAX_PHASES];        // hold / change target DO
uint32_t chPhaseDurSec[MAX_CHANNELS][MAX_PHASES];    // phase duration in seconds
char     chPhaseType[MAX_CHANNELS][MAX_PHASES];      // 'h','c','p','d'
float    chPhaseMinSP[MAX_CHANNELS][MAX_PHASES];     // 'd': min DO
float    chPhaseMaxSP[MAX_CHANNELS][MAX_PHASES];     // 'd': max DO; 'c': start DO (required, >0)
float    chPhasePeakHour[MAX_CHANNELS][MAX_PHASES];  // 'd': hour of maximum (0–24)

// ─── PIDs ─────────────────────────────────────────────────────────────────────
double doInput[MAX_CHANNELS]  = {0};
double doOutput[MAX_CHANNELS] = {0};
double holdSP[MAX_CHANNELS]   = {30,30,30,30,30,30,30,30};

PID valvePID0(&doInput[0], &doOutput[0], &holdSP[0], 10,1,0, REVERSE);
PID valvePID1(&doInput[1], &doOutput[1], &holdSP[1], 10,1,0, REVERSE);
PID valvePID2(&doInput[2], &doOutput[2], &holdSP[2], 10,1,0, REVERSE);
PID valvePID3(&doInput[3], &doOutput[3], &holdSP[3], 10,1,0, REVERSE);
PID valvePID4(&doInput[4], &doOutput[4], &holdSP[4], 10,1,0, REVERSE);
PID valvePID5(&doInput[5], &doOutput[5], &holdSP[5], 10,1,0, REVERSE);
PID valvePID6(&doInput[6], &doOutput[6], &holdSP[6], 10,1,0, REVERSE);
PID valvePID7(&doInput[7], &doOutput[7], &holdSP[7], 10,1,0, REVERSE);
PID* valvePIDs[MAX_CHANNELS] = {&valvePID0,&valvePID1,&valvePID2,&valvePID3,
                                 &valvePID4,&valvePID5,&valvePID6,&valvePID7};


// ─── serial receive buffer ────────────────────────────────────────────────────
char recvBuf[RECV_BUF];
int  recvIdx = 0;

// ─── runtime state ────────────────────────────────────────────────────────────
uint32_t expStartUnix    = 0;    // RTC unixtime of CMD:START (for elapsed-time log column)
uint32_t pauseStartUnix  = 0;
int      windowSize      = 0;
unsigned long loopStart  = 0;

// ─── SD / logging state ───────────────────────────────────────────────────────
char     filename[22];   // "YYYY_MM_DD_HH_MM.csv\0"
uint32_t rowN      = 0;
int      lastLogDay = 0;
bool     sdReady   = false;
bool     sdError   = false;

// ─── LCD state ────────────────────────────────────────────────────────────────
int           lcdPage           = 0;
int           lcdNumPages       = 1;
unsigned long lcdLastPageChange = 0;
unsigned long lcdLastRefresh    = 0;
double lastDO[MAX_CHANNELS] = {0};

// ─── error handling ───────────────────────────────────────────────────────────
int  errorCount = 0;


// ═══════════════════════════════════════════════════════════════════════════════
//  Helper: daily-cycle setpoint (takes values directly, not an array index)
// ═══════════════════════════════════════════════════════════════════════════════
float dailyCycleSP(float minSP, float maxSP, float peakHour, float hourDecimal) {
    float mean = (minSP + maxSP) / 2.0f;
    float ampl = (maxSP - minSP) / 2.0f;
    return mean + ampl * cos(2.0f * PI * (hourDecimal - peakHour) / 24.0f);
}


// ═══════════════════════════════════════════════════════════════════════════════
//  Measurement: route channels to the correct FireSting sensor
//  Sensor 1 (Serial1): logical channels 0 .. s1Channels-1  (ports 1, 2, ... s1Channels)
//  Sensor 2 (Serial2): logical channels s1Channels .. nChannels-1 (ports 1, 2, ...)
//  tempVals[0] = sensor 1 temperature ; tempVals[1] = sensor 2 temperature
// ═══════════════════════════════════════════════════════════════════════════════
bool measureAllChannels(double* doVals, double* tempVals) {
    if (nSensors == 1) {
        return ardoxy.measureAll(nChannels, doVals, &tempVals[0]);
    } else {
        int s2Channels = nChannels - s1Channels;
        bool ok1 = ardoxy.measureAll(s1Channels, doVals, &tempVals[0]);
        bool ok2 = ardoxy2.measureAll(s2Channels, doVals + s1Channels, &tempVals[1]);
        return ok1 && ok2;
    }
}


// ═══════════════════════════════════════════════════════════════════════════════
//  Serial data emission: DATA line + per-channel STATUS lines
// ═══════════════════════════════════════════════════════════════════════════════
void emitData(uint32_t elapsedSec, double* doVals, double* tempVals) {
    // DATA:<elapsed_s>,<do0,...>,<temp1>[,<temp2>],<out_ms0,...>
    Serial.print(F("DATA:"));
    Serial.print(elapsedSec);
    for (int i = 0; i < nChannels; i++) { Serial.print(','); Serial.print(doVals[i], 2); }
    Serial.print(','); Serial.print(tempVals[0], 2);
    if (nSensors == 2) { Serial.print(','); Serial.print(tempVals[1], 2); }
    for (int i = 0; i < nChannels; i++) { Serial.print(','); Serial.print((long)doOutput[i]); }
    Serial.println();

    // STATUS:CH:<ch>:<statusStr>:<phaseIdx>:<do>:<sp>
    for (int i = 0; i < nChannels; i++) {
        Serial.print(F("STATUS:CH:")); Serial.print(i); Serial.print(':');
        if      (chFault[i] == 2)              Serial.print(F("PRB"));
        else if (chFault[i] == 1)              Serial.print(F("CAL"));
        else if (chDone[i])                    Serial.print(F("DONE"));
        else if (chMode[i] == CH_MEASURE)      Serial.print(F("MEASURE"));
        else if (!chActive[i])                 Serial.print(F("WAITING-MEASURING"));
        else if (chMode[i] == CH_SETPOINT)     Serial.print(F("SETPOINT"));
        else                                   Serial.print(F("SEQUENCE"));
        Serial.print(':'); Serial.print(chPhaseIdx[i]);
        Serial.print(':'); Serial.print(doVals[i], 2);
        Serial.print(':');
        float sp = 0.0f;
        if (chActive[i] && !chDone[i] && chMode[i] != CH_MEASURE) {
            if (chMode[i] == CH_SETPOINT) {
                sp = chSetpoint[i];
            } else if (chNPhases[i] > 0) {
                byte pi = chPhaseIdx[i];
                char pt = chPhaseType[i][pi];
                sp = (pt == 'p') ? 0.0f : (float)holdSP[i];
            }
        }
        Serial.println(sp, 2);
    }
}


// ═══════════════════════════════════════════════════════════════════════════════
//  LCD update: page 0 = current datetime; pages 1..n = per-channel detail
//  Pages cycle automatically every 10 s
// ═══════════════════════════════════════════════════════════════════════════════
void lcdUpdate() {
    unsigned long now_ms = millis();
    bool pageChanged = false;
    if (now_ms - lcdLastPageChange >= LCD_PAGE_INTERVAL) {
        lcdPage = (lcdPage + 1) % lcdNumPages;
        lcdLastPageChange = now_ms;
        pageChanged = true;
    }
    if (!pageChanged && (now_ms - lcdLastRefresh < LCD_REFRESH_MS)) return;
    lcdLastRefresh = now_ms;

    lcd.clear();

    if (errorCount > 0) {
        lcd.setCursor(0, 0); lcd.print(F("Sensor error!"));
        lcd.setCursor(0, 1); lcd.print(F("Err ")); lcd.print(errorCount); lcd.print(F("/5"));
        return;
    }

    if (lcdPage == 0) {
        // Page 0: current date (line 0) + current time (line 1)
        DateTime now = RTC.now();
        lcd.setCursor(0, 0);
        lcd.print(now.year());
        lcd.print('/');
        if (now.month()  < 10) lcd.print('0'); lcd.print(now.month());
        lcd.print('/');
        if (now.day()    < 10) lcd.print('0'); lcd.print(now.day());
        lcd.setCursor(0, 1);
        if (sdError) {
            lcd.print(F("SD card fail!   "));
        } else {
            if (now.hour()   < 10) lcd.print('0'); lcd.print(now.hour());
            lcd.print(':');
            if (now.minute() < 10) lcd.print('0'); lcd.print(now.minute());
            lcd.print(':');
            if (now.second() < 10) lcd.print('0'); lcd.print(now.second());
        }

    } else {
        int i = lcdPage - 1;
        // Line 0: tankID  DO%  status
        lcd.setCursor(0, 0);
        lcd.print(tankID[i]); lcd.print(' ');
        if      (chFault[i] == 2) { lcd.print(F("PRB ")); }
        else if (chFault[i] == 1) { lcd.print(F("CAL ")); }
        else { lcd.print(lastDO[i], 2); lcd.print('%'); lcd.print(' '); }
        if      (chFault[i] > 0)               lcd.print(F("ERR"));
        else if (chDone[i])                    lcd.print(F("DONE"));
        else if (chMode[i] == CH_MEASURE)      lcd.print('M');
        else if (!chActive[i])                 lcd.print(F("WAIT"));
        else if (chMode[i] == CH_SETPOINT)     lcd.print(F("SP"));
        else { lcd.print('P'); lcd.print(chPhaseIdx[i] + 1); }  // SEQUENCE

        // Line 1: setpoint info
        lcd.setCursor(0, 1);
        if (chFault[i] > 0) {
            if (chFault[i] == 2) lcd.print(F("Probe error!    "));
            else                  lcd.print(F("Cal error!      "));
        } else if (chActive[i] && !chDone[i] && chMode[i] != CH_MEASURE) {
            if (chMode[i] == CH_SETPOINT) {
                lcd.print(F("SP:")); lcd.print(chSetpoint[i], 2);
            } else if (chNPhases[i] > 0) {
                byte pi = chPhaseIdx[i];
                char pt = chPhaseType[i][pi];
                lcd.print(pt); lcd.print(' ');
                if      (pt == 'h' || pt == 'd') { lcd.print(F("SP:")); lcd.print(holdSP[i], 2); }
                else if (pt == 'c')              { lcd.print(F(">")); lcd.print(chPhaseSP[i][pi], 2); }
                else                               lcd.print(F("pause"));
            }
        }
    }
}


// ═══════════════════════════════════════════════════════════════════════════════
//  SD: open or create today's log file; write CSV header if new
// ═══════════════════════════════════════════════════════════════════════════════
void createLogfile() {
    DateTime now = RTC.now();
    sprintf(filename, "%04d_%02d_%02d_%02d_%02d.csv",
            now.year(), now.month(), now.day(), now.hour(), now.minute());
    lastLogDay = now.day();
    if (!SD.exists(filename)) {
        FsFile f = SD.open(filename, O_WRITE | O_CREAT | O_AT_END);
        if (f) {
            f.print(F("ROW;ELAPSED_S;DATE;TIME"));
            for (int i = 0; i < nChannels; i++) { f.print(';'); f.print(F("DO_")); f.print(tankID[i]); }
            f.print(F(";TEMP1"));
            if (nSensors == 2) f.print(F(";TEMP2"));
            for (int i = 0; i < nChannels; i++) { f.print(';'); f.print(F("OUT_MS_")); f.print(tankID[i]); }
            f.println();
            f.close();
        }
    }
    Serial.print(F("MSG:Logfile ")); Serial.println(filename);
}


// ═══════════════════════════════════════════════════════════════════════════════
//  SD: append one measurement row; rotate file at midnight
// ═══════════════════════════════════════════════════════════════════════════════
void writeToSD(double* doVals, double* tempVals) {
    if (!sdReady) { sdError = true; return; }
    DateTime now = RTC.now();
    if (now.day() != lastLogDay) createLogfile();  // daily rotation
    FsFile f = SD.open(filename, O_WRITE | O_AT_END);
    if (!f) { sdError = true; return; }
    sdError = false;
    uint32_t elapsedSec = now.unixtime() - expStartUnix;
    f.print(rowN);       f.print(';');
    f.print(elapsedSec); f.print(';');
    f.print(now.year()); f.print('/');
    f.print(now.month()); f.print('/');
    f.print(now.day());  f.print(';');
    f.print(now.hour()); f.print(':');
    f.print(now.minute()); f.print(':');
    f.print(now.second());
    for (int i = 0; i < nChannels; i++) { f.print(';'); f.print(doVals[i], 3); }
    f.print(';'); f.print(tempVals[0], 2);
    if (nSensors == 2) { f.print(';'); f.print(tempVals[1], 2); }
    for (int i = 0; i < nChannels; i++) { f.print(';'); f.print((long)doOutput[i]); }
    f.println();
    f.close();
    rowN++;
}


// ═══════════════════════════════════════════════════════════════════════════════
//  SD: persist experiment state to STATE.TXT (called every measurement cycle)
// ═══════════════════════════════════════════════════════════════════════════════
void writeState() {
    if (!sdReady) { sdError = true; return; }
    FsFile stFile = SD.open("STATE.TXT", O_WRITE | O_CREAT | O_TRUNC);
    if (!stFile) { sdError = true; return; }
    sdError = false;
    stFile.print(F("EXP_START=")); stFile.println(expStartUnix);
    stFile.print(F("LOGFILE="));   stFile.println(filename);
    stFile.print(F("ROWN="));      stFile.println(rowN);
    for (int i = 0; i < nChannels; i++) {
        stFile.print(F("CH_")); stFile.print(i); stFile.print(F("_PIDX="));  stFile.println(chPhaseIdx[i]);
        stFile.print(F("CH_")); stFile.print(i); stFile.print(F("_DONE="));  stFile.println(chDone[i] ? 1 : 0);
        stFile.print(F("CH_")); stFile.print(i); stFile.print(F("_SPEND=")); stFile.println(chSetpointEndUnix[i]);
        stFile.print(F("CH_")); stFile.print(i); stFile.print(F("_PEND="));  stFile.println(chCurrentPhaseEndUnix[i]);
    }
    stFile.close();
}


// ═══════════════════════════════════════════════════════════════════════════════
//  SD: load experiment state from STATE.TXT on boot
// ═══════════════════════════════════════════════════════════════════════════════

static void parseStateLine(char* key, char* val) {
    if (strcmp(key, "EXP_START") == 0) { expStartUnix = strtoul(val, NULL, 10); return; }
    if (strcmp(key, "LOGFILE")   == 0) { strncpy(filename, val, 21); filename[21] = '\0'; return; }
    if (strcmp(key, "ROWN")      == 0) { rowN = strtoul(val, NULL, 10); return; }
    if (strncmp(key, "CH_", 3)  != 0) return;
    char* p = key + 3;
    int ch = atoi(p);
    if (ch < 0 || ch >= MAX_CHANNELS) return;
    while (*p && *p != '_') p++;  // skip digit(s)
    if (*p == '_') p++;           // skip '_'
    if      (strcmp(p, "PIDX" ) == 0) chPhaseIdx[ch]            = (byte)atoi(val);
    else if (strcmp(p, "DONE" ) == 0) chDone[ch]                = (atoi(val) != 0);
    else if (strcmp(p, "SPEND") == 0) chSetpointEndUnix[ch]     = strtoul(val, NULL, 10);
    else if (strcmp(p, "PEND" ) == 0) chCurrentPhaseEndUnix[ch] = strtoul(val, NULL, 10);
}

bool readState() {
    if (!sdReady) return false;
    FsFile stFile = SD.open("STATE.TXT", O_READ);
    if (!stFile) return false;
    char line[42];
    int  lineLen = 0;
    bool gotStart = false;
    while (stFile.available()) {
        char c = (char)stFile.read();
        if (c == '\n' || c == '\r') {
            if (lineLen > 0) {
                line[lineLen] = '\0';
                char* eq = strchr(line, '=');
                if (eq) {
                    *eq = '\0';
                    if (strcmp(line, "EXP_START") == 0) gotStart = true;
                    parseStateLine(line, eq + 1);
                }
                lineLen = 0;
            }
        } else if (lineLen < 41) {
            line[lineLen++] = c;
        }
    }
    stFile.close();
    return gotStart;
}


// ═══════════════════════════════════════════════════════════════════════════════
//  SD: save config to CONFIG.TXT
// ═══════════════════════════════════════════════════════════════════════════════
void saveConfig() {
    if (!sdReady) { sdError = true; Serial.println(F("ACK:ERR:SD not ready")); return; }
    SD.remove("CONFIG.TXT");
    SD.remove("STATE.TXT"); // remove old state - new config means that the old state is invalid
    FsFile cfgFile = SD.open("CONFIG.TXT", O_WRITE | O_CREAT | O_TRUNC);
    if (!cfgFile) { sdError = true; Serial.println(F("ACK:ERR:SD open fail")); return; }
    sdError = false;

    // ── shared hardware config ────────────────────────────────────────────────
    cfgFile.print(F("NCHANNELS="));  cfgFile.println(nChannels);
    cfgFile.print(F("NSENSORS="));   cfgFile.println(nSensors);
    cfgFile.print(F("S1CHANNELS=")); cfgFile.println(s1Channels);
    for (int i = 0; i < MAX_CHANNELS; i++) {
        cfgFile.print(F("RELAY_")); cfgFile.print(i);
        cfgFile.print('='); cfgFile.println(relayPins[i]);
    }
    cfgFile.print(F("INTERVAL=")); cfgFile.println(sampInterval);
    for (int i = 0; i < MAX_CHANNELS; i++) {
        cfgFile.print(F("TANKID_")); cfgFile.print(i);
        cfgFile.print('='); cfgFile.println(tankID[i]);
    }
    for (int i = 0; i < MAX_CHANNELS; i++) {
        cfgFile.print(F("KP_")); cfgFile.print(i); cfgFile.print('='); cfgFile.println(Kp[i], 4);
        cfgFile.print(F("KI_")); cfgFile.print(i); cfgFile.print('='); cfgFile.println(Ki[i], 4);
        cfgFile.print(F("KD_")); cfgFile.print(i); cfgFile.print('='); cfgFile.println(Kd[i], 4);
    }

    // ── per-channel config ────────────────────────────────────────────────────
    for (int i = 0; i < MAX_CHANNELS; i++) {
        cfgFile.print(F("CH_")); cfgFile.print(i); cfgFile.print(F("_MODE="));     cfgFile.println((int)chMode[i]);
        cfgFile.print(F("CH_")); cfgFile.print(i); cfgFile.print(F("_START="));    cfgFile.println(chStart[i]);
        cfgFile.print(F("CH_")); cfgFile.print(i); cfgFile.print(F("_SETPOINT=")); cfgFile.println(chSetpoint[i], 4);
        cfgFile.print(F("CH_")); cfgFile.print(i); cfgFile.print(F("_DUR_MIN="));  cfgFile.println(chDurationMin[i]);
        cfgFile.print(F("CH_")); cfgFile.print(i); cfgFile.print(F("_NPHASES="));  cfgFile.println(chNPhases[i]);
        for (int j = 0; j < chNPhases[i]; j++) {
            cfgFile.print(F("CH_")); cfgFile.print(i);
            cfgFile.print(F("_PHASE_")); cfgFile.print(j); cfgFile.print('=');
            cfgFile.print(chPhaseSP[i][j], 4); cfgFile.print(',');
            cfgFile.print(chPhaseDurSec[i][j]); cfgFile.print(',');
            cfgFile.print(chPhaseType[i][j]);
            if (chPhaseType[i][j] == 'd') {
                cfgFile.print(','); cfgFile.print(chPhaseMinSP[i][j],   4);
                cfgFile.print(','); cfgFile.print(chPhaseMaxSP[i][j],   4);
                cfgFile.print(','); cfgFile.print(chPhasePeakHour[i][j], 4);
            } else if (chPhaseType[i][j] == 'c' && chPhaseMaxSP[i][j] > 0.0f) {
                cfgFile.print(','); cfgFile.print(chPhaseMaxSP[i][j], 4);  // start DO
            }
            cfgFile.println();
        }
    }
    cfgFile.close();
    Serial.println(F("ACK:OK"));
    Serial.println(F("MSG:Config saved"));
}


// ═══════════════════════════════════════════════════════════════════════════════
//  SD: load config from CONFIG.TXT  (line-by-line, 72-byte stack buffer)
// ═══════════════════════════════════════════════════════════════════════════════

// Helper — called once per key=value line. Modifies key and val in place.
static void parseConfigLine(char* key, char* val) {
    // ── simple keys ──────────────────────────────────────────────────────────
    if (strcmp(key, "NCHANNELS")  == 0) { nChannels   = constrain(atoi(val), 1, MAX_CHANNELS);   return; }
    if (strcmp(key, "NSENSORS")   == 0) { nSensors    = constrain(atoi(val), 1, 2);              return; }
    if (strcmp(key, "S1CHANNELS") == 0) { s1Channels  = constrain(atoi(val), 1, MAX_CHANNELS-1); return; }
    if (strcmp(key, "INTERVAL")   == 0) { sampInterval = atol(val);                               return; }

    // ── indexed keys: RELAY_i, TANKID_i, KP_i, KI_i, KD_i ──────────────────
    if (strncmp(key, "RELAY_",  6) == 0) { int i=atoi(key+6);  if (i>=0&&i<MAX_CHANNELS) relayPins[i]=atoi(val);         return; }
    if (strncmp(key, "TANKID_", 7) == 0) { int i=atoi(key+7);  if (i>=0&&i<MAX_CHANNELS) { strncpy(tankID[i],val,6); tankID[i][6]='\0'; } return; }
    if (strncmp(key, "KP_",     3) == 0) { int i=atoi(key+3);  if (i>=0&&i<MAX_CHANNELS) Kp[i]=atof(val);                return; }
    if (strncmp(key, "KI_",     3) == 0) { int i=atoi(key+3);  if (i>=0&&i<MAX_CHANNELS) Ki[i]=atof(val);                return; }
    if (strncmp(key, "KD_",     3) == 0) { int i=atoi(key+3);  if (i>=0&&i<MAX_CHANNELS) Kd[i]=atof(val);                return; }

    // ── per-channel keys: CH_i_* ──────────────────────────────────────────────
    if (strncmp(key, "CH_", 3) != 0) return;
    char* p = key + 3;
    int ch = atoi(p);
    if (ch < 0 || ch >= MAX_CHANNELS) return;
    while (*p && *p != '_') p++;   // skip digit(s)
    if (*p == '_') p++;            // skip '_'

    if (strcmp(p, "MODE")     == 0) { int m=atoi(val); if(m>=0&&m<=2) chMode[ch]=(ChMode)m;            return; }
    if (strcmp(p, "START")    == 0) { chStart[ch]       = strtoul(val, NULL, 10);                        return; }
    if (strcmp(p, "SETPOINT") == 0) { chSetpoint[ch]    = atof(val);                                     return; }
    if (strcmp(p, "DUR_MIN")  == 0) { chDurationMin[ch] = atol(val);                                     return; }
    if (strcmp(p, "NPHASES")  == 0) { chNPhases[ch]     = (byte)constrain(atoi(val), 0, MAX_PHASES);     return; }

    if (strncmp(p, "PHASE_", 6) == 0) {
        int j = atoi(p + 6);
        if (j < 0 || j >= MAX_PHASES) return;
        // val format: "sp,durSec,type[,minSP,maxSP,peakHour]"
        char vbuf[56];
        strncpy(vbuf, val, 55); vbuf[55] = '\0';
        char* tok = strtok(vbuf, ","); if (!tok) return;
        chPhaseSP[ch][j] = atof(tok);
        tok = strtok(NULL, ","); if (!tok) return;
        chPhaseDurSec[ch][j] = strtoul(tok, NULL, 10);
        tok = strtok(NULL, ","); if (!tok) return;
        chPhaseType[ch][j] = tok[0];
        if (tok[0] == 'd') {
            tok = strtok(NULL, ","); if (!tok) return; chPhaseMinSP[ch][j]    = atof(tok);
            tok = strtok(NULL, ","); if (!tok) return; chPhaseMaxSP[ch][j]    = atof(tok);
            tok = strtok(NULL, ","); if (!tok) return; chPhasePeakHour[ch][j] = atof(tok);
        } else if (tok[0] == 'c') {
            tok = strtok(NULL, ",");
            chPhaseMaxSP[ch][j] = (tok && atof(tok) > 0.0f) ? atof(tok) : 0.0f;  // start DO (0=auto)
        }
    }
}

bool loadConfig() {
    if (!sdReady) return false;
    FsFile cfgFile = SD.open("CONFIG.TXT", O_READ);
    if (!cfgFile) return false;

    char line[72];
    int  lineLen = 0;

    while (cfgFile.available()) {
        char c = (char)cfgFile.read();
        if (c == '\n' || c == '\r') {
            if (lineLen > 0) {
                line[lineLen] = '\0';
                char* eq = strchr(line, '=');
                if (eq) { *eq = '\0'; parseConfigLine(line, eq + 1); }
                lineLen = 0;
            }
        } else if (lineLen < 71) {
            line[lineLen++] = c;
        }
    }
    cfgFile.close();
    return true;
}


// ═══════════════════════════════════════════════════════════════════════════════
//  Hardware init shared by startExperiment() and recoverExperiment()
// ═══════════════════════════════════════════════════════════════════════════════
void initHardware() {
    for (int i = 0; i < nChannels; i++) {
        pinMode(relayPins[i], OUTPUT);
        digitalWrite(relayPins[i], HIGH);   // HIGH = relay closed (valve shut)
    }
    windowSize = sampInterval / 200;
    for (int i = 0; i < nChannels; i++) {
        Ardoxy::configurePID(*valvePIDs[i], Kp[i], Ki[i], Kd[i], sampInterval, windowSize);
    }
    lcdNumPages       = nChannels + 1;
    lcdPage           = 0;
    lcdLastPageChange = 0;
    lcdLastRefresh    = 0;
    bool ok1 = ardoxy.begin();
    bool ok2 = (nSensors == 2) ? ardoxy2.begin() : true;
    if (!ok1 || !ok2) {
        Serial.println(F("MSG:Sensor connection failed — resetting in 5s"));
        lcd.clear(); lcd.print(F("Sensor FAILED")); lcd.setCursor(0, 1); lcd.print(F("Resetting..."));
        delay(5000);
        wdt_enable(WDTO_15MS); while(1);
    }
}


// ═══════════════════════════════════════════════════════════════════════════════
//  Start experiment fresh
// ═══════════════════════════════════════════════════════════════════════════════
void startExperiment() {
    initHardware();
    expStartUnix = RTC.now().unixtime();
    rowN = 0;

    for (int i = 0; i < nChannels; i++) {
        chPhaseIdx[i]    = 0;
        chDone[i]        = false;
        chPhaseStartDO[i] = 0.0f;

        // Normalise chStart: 0 means "start immediately at CMD:START"
        if (chStart[i] == 0) chStart[i] = expStartUnix;

        bool immediate = (chStart[i] <= expStartUnix);
        chActive[i] = immediate;   // WAITING channels are activated in runAllChannels

        if (chMode[i] == CH_SETPOINT) {
            chSetpointEndUnix[i] = chStart[i] + (uint32_t)chDurationMin[i] * 60UL;
            if (immediate) {
                holdSP[i] = chSetpoint[i];
                valvePIDs[i]->SetMode(AUTOMATIC);
            }

        } else if (chMode[i] == CH_SEQUENCE && chNPhases[i] > 0) {
            chCurrentPhaseEndUnix[i] = chStart[i] + chPhaseDurSec[i][0];
            if (immediate) {
                char t = chPhaseType[i][0];
                if (t == 'h') {
                    holdSP[i] = chPhaseSP[i][0];
                    valvePIDs[i]->SetMode(AUTOMATIC);
                } else if (t == 'd') {
                    DateTime _dt = RTC.now();
                    float _hr = _dt.hour() + _dt.minute() / 60.0f + _dt.second() / 3600.0f;
                    holdSP[i] = dailyCycleSP(chPhaseMinSP[i][0], chPhaseMaxSP[i][0], chPhasePeakHour[i][0], _hr);
                    valvePIDs[i]->SetMode(AUTOMATIC);
                } else if (t == 'c') {
                    chPhaseStartDO[i] = chPhaseMaxSP[i][0] > 0.0f ? chPhaseMaxSP[i][0] : 0.0f;
                    valvePIDs[i]->SetMode(AUTOMATIC);
                }
                // 'p': all PIDs remain MANUAL (relays closed)
            }
        }
        // CH_MEASURE: no PIDs, no end times needed
    }

    createLogfile();
    state = RUNNING;
    Serial.println(F("ACK:OK"));
    Serial.println(F("MSG:Running"));
}


// ═══════════════════════════════════════════════════════════════════════════════
//  Recover experiment from STATE.TXT after power outage
//  Precondition: loadConfig() + readState() already called by setup()
// ═══════════════════════════════════════════════════════════════════════════════
void recoverExperiment() {
    initHardware();
    uint32_t nowUnix = RTC.now().unixtime();

    for (int i = 0; i < nChannels; i++) {
        chPhaseStartDO[i] = 0.0f;
        chActive[i]       = (chStart[i] <= nowUnix);

        if (chDone[i] || chMode[i] == CH_MEASURE || !chActive[i]) continue;

        if (chMode[i] == CH_SETPOINT) {
            if (nowUnix >= chSetpointEndUnix[i]) {
                chDone[i] = true;  // finished during outage
            } else {
                holdSP[i] = chSetpoint[i];
                valvePIDs[i]->SetMode(AUTOMATIC);
            }

        } else if (chMode[i] == CH_SEQUENCE) {
            // Fast-forward through phases that expired during outage
            while (chPhaseIdx[i] < chNPhases[i] && nowUnix >= chCurrentPhaseEndUnix[i]) {
                chPhaseIdx[i]++;
                if (chPhaseIdx[i] < chNPhases[i])
                    chCurrentPhaseEndUnix[i] += chPhaseDurSec[i][chPhaseIdx[i]];
            }
            if (chPhaseIdx[i] >= chNPhases[i]) {
                chDone[i] = true;
            } else {
                char t = chPhaseType[i][chPhaseIdx[i]];
                if (t == 'h') {
                    holdSP[i] = chPhaseSP[i][chPhaseIdx[i]]; valvePIDs[i]->SetMode(AUTOMATIC);
                } else if (t == 'd') {
                    DateTime _dt = RTC.now();
                    float _hr = _dt.hour() + _dt.minute() / 60.0f + _dt.second() / 3600.0f;
                    holdSP[i] = dailyCycleSP(chPhaseMinSP[i][chPhaseIdx[i]], chPhaseMaxSP[i][chPhaseIdx[i]], chPhasePeakHour[i][chPhaseIdx[i]], _hr);
                    valvePIDs[i]->SetMode(AUTOMATIC);
                } else if (t == 'c') { chPhaseStartDO[i] = chPhaseMaxSP[i][chPhaseIdx[i]] > 0.0f ? chPhaseMaxSP[i][chPhaseIdx[i]] : 0.0f; valvePIDs[i]->SetMode(AUTOMATIC); }
                // 'p': both PIDs remain MANUAL
            }
        }
        if (chDone[i]) {
            Serial.print(F("MSG:CH")); Serial.print(i); Serial.println(F(" finished during outage"));
        }
    }

    // Append restart marker to existing logfile then create today's log
    if (SD.exists(filename)) {
        FsFile old = SD.open(filename, O_WRITE | O_AT_END);
        if (old) {
            DateTime now = RTC.now();
            old.print(F("RESTART;")); old.print(now.year()); old.print('/');
            old.print(now.month()); old.print('/'); old.print(now.day());
            old.print(';'); old.print(now.hour()); old.print(':');
            old.print(now.minute()); old.print(':'); old.println(now.second());
            old.close();
        }
    }
    createLogfile();  // D7 will open/create the daily CSV

    state = RUNNING;
    Serial.println(F("MSG:Recovered"));
    lcd.clear(); lcd.print(F("Recovered!")); lcd.setCursor(0, 1); lcd.print(filename);
    delay(1500);
}


// ═══════════════════════════════════════════════════════════════════════════════
//  Main run loop  — core structure complete; phase logic STUB (Phases D4 + D5)
// ═══════════════════════════════════════════════════════════════════════════════
void runAllChannels() {
    loopStart = millis();
    double doVals[MAX_CHANNELS];
    double tempVals[2] = {0, 0};

    // 1. Measure ALL channels unconditionally every cycle
    if (!measureAllChannels(doVals, tempVals)) {
        Serial.println(F("MSG:Sensor error"));
        if (++errorCount >= 5) {
            Serial.println(F("MSG:Too many sensor errors — resetting in 3s"));
            lcd.clear(); lcd.print(F("Sensor errors")); lcd.setCursor(0, 1); lcd.print(F("Resetting..."));
            Ardoxy::closeRelays(nChannels, relayPins);
            delay(3000);
            wdt_enable(WDTO_15MS); while(1);
        }
        Ardoxy::closeRelays(nChannels, relayPins);
        lcdLastRefresh = 0;   // force immediate LCD redraw on first poll
        wdt_enable(WDTO_8S);
        long rem = sampInterval - (long)(millis() - loopStart);
        while (rem > 0) {
            wdt_reset();
            delay(rem > 500L ? 500L : rem);
            lcdUpdate();
            rem = sampInterval - (long)(millis() - loopStart);
        }
        wdt_disable();
        return;
    }
    errorCount = 0;
    for (int i = 0; i < nChannels; i++) lastDO[i] = doVals[i];

    // 2a. Probe/calibration fault detection (auto-clears each cycle)
    for (int i = 0; i < nChannels; i++) {
        byte prevFault = chFault[i];
        if      (doVals[i] < -20.0 || doVals[i] > 300.0) chFault[i] = 2;
        else if (doVals[i] <  -5.0 || doVals[i] > 150.0) chFault[i] = 1;
        else                                               chFault[i] = 0;
        if (chFault[i] != prevFault && chFault[i] > 0) {
            Serial.print(F("MSG:CH")); Serial.print(i);
            if (chFault[i] == 2) Serial.println(F(":PRB"));
            else                 Serial.println(F(":CAL"));
        }
    }

    // 2. Per-channel output computation
    DateTime nowDT = RTC.now();
    uint32_t nowUnix = nowDT.unixtime();

    for (int i = 0; i < nChannels; i++) {
        doInput[i]  = doVals[i];
        doOutput[i] = 0;   // default: valve closed

        // Probe/cal fault: suppress control, keep measuring
        if (chFault[i] > 0) continue;

        // Done or pure-measure: no control
        if (chDone[i] || chMode[i] == CH_MEASURE) continue;

        // WAITING: scheduled start not yet reached
        if (chStart[i] > nowUnix) continue;

        // First activation: WAITING channel just became active this cycle
        if (!chActive[i]) {
            chActive[i]    = true;
            if (chMode[i] == CH_SETPOINT) {
                holdSP[i] = chSetpoint[i];
                valvePIDs[i]->SetMode(AUTOMATIC);
            } else if (chMode[i] == CH_SEQUENCE && chNPhases[i] > 0) {
                char t = chPhaseType[i][chPhaseIdx[i]];
                if (t == 'h') {
                    holdSP[i] = chPhaseSP[i][chPhaseIdx[i]]; valvePIDs[i]->SetMode(AUTOMATIC);
                } else if (t == 'd') {
                    float _hr = nowDT.hour() + nowDT.minute() / 60.0f + nowDT.second() / 3600.0f;
                    holdSP[i] = dailyCycleSP(chPhaseMinSP[i][chPhaseIdx[i]], chPhaseMaxSP[i][chPhaseIdx[i]], chPhasePeakHour[i][chPhaseIdx[i]], _hr);
                    valvePIDs[i]->SetMode(AUTOMATIC);
                } else if (t == 'c') { chPhaseStartDO[i] = chPhaseMaxSP[i][chPhaseIdx[i]] > 0.0f ? chPhaseMaxSP[i][chPhaseIdx[i]] : (float)doVals[i]; valvePIDs[i]->SetMode(AUTOMATIC); }
                // 'p': all PIDs remain MANUAL
            }
            Serial.print(F("MSG:CH")); Serial.print(i); Serial.println(F(" started"));
        }

        // ── SETPOINT ─────────────────────────────────────────────────────────
        if (chMode[i] == CH_SETPOINT) {
            if (nowUnix >= chSetpointEndUnix[i]) {
                valvePIDs[i]->SetMode(MANUAL);
                chDone[i] = true;
                Serial.print(F("MSG:CH")); Serial.print(i); Serial.println(F(":DONE"));
                continue;
            }
            holdSP[i] = chSetpoint[i];
            valvePIDs[i]->Compute();

        // ── SEQUENCE ─────────────────────────────────────────────────────────
        } else if (chMode[i] == CH_SEQUENCE) {
            if (chNPhases[i] == 0) { chDone[i] = true; continue; }

            // Phase advance
            while (!chDone[i] && nowUnix >= chCurrentPhaseEndUnix[i]) {
                chPhaseIdx[i]++;
                if (chPhaseIdx[i] >= chNPhases[i]) {
                    valvePIDs[i]->SetMode(MANUAL);
                    chDone[i] = true;
                    Serial.print(F("MSG:CH")); Serial.print(i); Serial.println(F(":DONE"));
                    break;
                }
                chCurrentPhaseEndUnix[i] += chPhaseDurSec[i][chPhaseIdx[i]];
                char nt = chPhaseType[i][chPhaseIdx[i]];
                if (nt == 'h') {
                    holdSP[i] = chPhaseSP[i][chPhaseIdx[i]]; valvePIDs[i]->SetMode(AUTOMATIC);
                } else if (nt == 'd') {
                    float _hr = nowDT.hour() + nowDT.minute() / 60.0f + nowDT.second() / 3600.0f;
                    holdSP[i] = dailyCycleSP(chPhaseMinSP[i][chPhaseIdx[i]], chPhaseMaxSP[i][chPhaseIdx[i]], chPhasePeakHour[i][chPhaseIdx[i]], _hr);
                    valvePIDs[i]->SetMode(AUTOMATIC);
                } else if (nt == 'c') { chPhaseStartDO[i] = chPhaseMaxSP[i][chPhaseIdx[i]] > 0.0f ? chPhaseMaxSP[i][chPhaseIdx[i]] : (float)doVals[i]; valvePIDs[i]->SetMode(AUTOMATIC); }
                else                  { valvePIDs[i]->SetMode(MANUAL); }  // 'p'
                Serial.print(F("MSG:CH")); Serial.print(i);
                Serial.print(F(":phase ")); Serial.println(chPhaseIdx[i]);
            }
            if (chDone[i]) continue;

            // Phase logic
            byte pi = chPhaseIdx[i];
            char pt = chPhaseType[i][pi];

            if (pt == 'p') {
                doOutput[i] = 0;

            } else if (pt == 'h') {
                holdSP[i] = chPhaseSP[i][pi];
                valvePIDs[i]->Compute();

            } else if (pt == 'd') {
                float hr = nowDT.hour() + nowDT.minute() / 60.0f + nowDT.second() / 3600.0f;
                holdSP[i] = dailyCycleSP(chPhaseMinSP[i][pi], chPhaseMaxSP[i][pi],
                                         chPhasePeakHour[i][pi], hr);
                valvePIDs[i]->Compute();

            } else if (pt == 'c') {
                // Linear ramp from startDO to targetSP; startDO required (set via chPhaseMaxSP).
                // If unconfigured (0), falls back to target SP — behaves as hold, no ramp.
                if (chPhaseStartDO[i] <= 0.0f) {
                    chPhaseStartDO[i] = chPhaseMaxSP[i][pi] > 0.0f ? chPhaseMaxSP[i][pi] : chPhaseSP[i][pi];
                }
                uint32_t phaseStartUnix = chCurrentPhaseEndUnix[i] - chPhaseDurSec[i][pi];
                float elapsed = (nowUnix > phaseStartUnix) ? (float)(nowUnix - phaseStartUnix) : 0.0f;
                float total   = (float)chPhaseDurSec[i][pi];
                float frac    = (total > 0.0f) ? min(elapsed / total, 1.0f) : 1.0f;
                holdSP[i]     = chPhaseStartDO[i] + (chPhaseSP[i][pi] - chPhaseStartDO[i]) * frac;
                valvePIDs[i]->Compute();
            }
        }
    }

    // 3. Schedule relays (parallel-open, sequential-close by duration)
    {
        long _window = sampInterval - ((long)nChannels * 40 + 500);
        Ardoxy::scheduleRelays(nChannels, doOutput, relayPins,
                               _window > 0L ? (unsigned long)_window : 0UL);
    }

    // 4. Emit, log, persist, display
    wdt_enable(WDTO_8S);
    uint32_t elapsedSec = nowDT.unixtime() - expStartUnix;
    emitData(elapsedSec, doVals, tempVals);
    writeToSD(doVals, tempVals);
    writeState();
    lcdUpdate();

    long rem = sampInterval - (long)(millis() - loopStart);
    while (rem > 0) {
        wdt_reset();
        delay(rem > 500L ? 500L : rem);
        lcdUpdate();
        rem = sampInterval - (long)(millis() - loopStart);
    }
    wdt_disable();
}


// ═══════════════════════════════════════════════════════════════════════════════
//  Serial command parser
// ═══════════════════════════════════════════════════════════════════════════════
void processCommand(char* buf) {
    char* cat = strtok(buf, ":");
    if (cat == NULL) return;

    // ── CMD ──────────────────────────────────────────────────────────────────
    if (strcmp_P(cat, PSTR("CMD")) == 0) {
        char* key = strtok(NULL, ":");
        if (key == NULL) return;

        if (strcmp_P(key, PSTR("STATUS")) == 0) {
            Serial.print(F("STATUS:"));
            if      (state == IDLE)       Serial.println(F("IDLE"));
            else if (state == CONFIGURED) Serial.println(F("CONFIGURED"));
            else if (state == PAUSED)     Serial.println(F("PAUSED"));
            else                          Serial.println(F("RUNNING"));
            return;
        }

        if (strcmp_P(key, PSTR("STOP")) == 0) {
            if (state == RUNNING || state == PAUSED) {
                Ardoxy::closeRelays(nChannels, relayPins);
                ardoxy.end();
                if (nSensors == 2) ardoxy2.end();
                state = CONFIGURED;
            }
            Serial.println(F("ACK:OK"));
            return;
        }

        if (strcmp_P(key, PSTR("PAUSE")) == 0) {
            if (state == RUNNING) {
                Ardoxy::closeRelays(nChannels, relayPins);
                pauseStartUnix = RTC.now().unixtime();
                state = PAUSED;
                Serial.println(F("ACK:OK"));
                Serial.println(F("MSG:Paused"));
            } else {
                Serial.println(F("ACK:ERR:Not running"));
            }
            return;
        }

        if (strcmp_P(key, PSTR("RESUME")) == 0) {
            if (state == PAUSED) {
                uint32_t pausedFor = RTC.now().unixtime() - pauseStartUnix;
                expStartUnix += pausedFor;
                for (int i = 0; i < nChannels; i++) {
                    if (chDone[i]) continue;
                    if (chStart[i] > pauseStartUnix) {
                        // WAITING channel: shift scheduled start forward
                        chStart[i] += pausedFor;
                    } else {
                        // Active channel: shift end times forward
                        chSetpointEndUnix[i]    += pausedFor;
                        chCurrentPhaseEndUnix[i] += pausedFor;
                    }
                }
                state = RUNNING;
                Serial.println(F("ACK:OK"));
                Serial.println(F("MSG:Running"));
            } else {
                Serial.println(F("ACK:ERR:Not paused"));
            }
            return;
        }

        if (strcmp_P(key, PSTR("SAVECONFIG")) == 0) {
            saveConfig();
            return;
        }

        if (strcmp_P(key, PSTR("READCONFIG")) == 0) {
            if (state == RUNNING) { Serial.println(F("ACK:ERR:Running")); return; }
            Serial.println(F("CONFIG_START"));
            Serial.print(F("NCHANNELS=")); Serial.println(nChannels);
            Serial.print(F("NSENSORS="));  Serial.println(nSensors);
            Serial.print(F("S1CHANNELS=")); Serial.println(s1Channels);
            for (int i = 0; i < MAX_CHANNELS; i++) {
                Serial.print(F("RELAY_")); Serial.print(i); Serial.print('='); Serial.println(relayPins[i]);
            }
            Serial.print(F("INTERVAL=")); Serial.println(sampInterval);
            for (int i = 0; i < MAX_CHANNELS; i++) {
                Serial.print(F("TANKID_")); Serial.print(i); Serial.print('='); Serial.println(tankID[i]);
            }
            for (int i = 0; i < MAX_CHANNELS; i++) {
                Serial.print(F("KP_")); Serial.print(i); Serial.print('='); Serial.println(Kp[i], 4);
                Serial.print(F("KI_")); Serial.print(i); Serial.print('='); Serial.println(Ki[i], 4);
                Serial.print(F("KD_")); Serial.print(i); Serial.print('='); Serial.println(Kd[i], 4);
            }
            for (int i = 0; i < MAX_CHANNELS; i++) {
                Serial.print(F("CH_")); Serial.print(i); Serial.print(F("_MODE="));     Serial.println((int)chMode[i]);
                Serial.print(F("CH_")); Serial.print(i); Serial.print(F("_START="));    Serial.println(chStart[i]);
                Serial.print(F("CH_")); Serial.print(i); Serial.print(F("_SETPOINT=")); Serial.println(chSetpoint[i], 4);
                Serial.print(F("CH_")); Serial.print(i); Serial.print(F("_DUR_MIN="));  Serial.println(chDurationMin[i]);
                Serial.print(F("CH_")); Serial.print(i); Serial.print(F("_NPHASES="));  Serial.println(chNPhases[i]);
                for (int j = 0; j < chNPhases[i]; j++) {
                    Serial.print(F("CH_")); Serial.print(i); Serial.print(F("_PHASE_")); Serial.print(j); Serial.print('=');
                    Serial.print(chPhaseSP[i][j], 4);   Serial.print(',');
                    Serial.print(chPhaseDurSec[i][j]);   Serial.print(',');
                    Serial.print(chPhaseType[i][j]);
                    if (chPhaseType[i][j] == 'd') {
                        Serial.print(','); Serial.print(chPhaseMinSP[i][j],   4);
                        Serial.print(','); Serial.print(chPhaseMaxSP[i][j],   4);
                        Serial.print(','); Serial.print(chPhasePeakHour[i][j], 4);
                    } else if (chPhaseType[i][j] == 'c' && chPhaseMaxSP[i][j] > 0.0f) {
                        Serial.print(','); Serial.print(chPhaseMaxSP[i][j], 4);
                    }
                    Serial.println();
                }
            }
            Serial.println(F("CONFIG_END"));
            return;
        }

        if (strcmp_P(key, PSTR("LISTFILES")) == 0) {
            if (state == RUNNING) { Serial.println(F("ACK:ERR:Running")); return; }
            if (!sdReady) { Serial.println(F("ACK:ERR:SD not ready")); return; }
            FsFile root, f;
            root.open("/");
            char fname[25];
            while (f.openNext(&root, O_RDONLY)) {
                f.getName(fname, sizeof(fname));
                int len = strlen(fname);
                if (!f.isDir() && len > 4 && strcmp(fname + len - 4, ".csv") == 0) {
                    Serial.print(F("FILE:")); Serial.print(fname);
                    Serial.print(':');         Serial.println((uint32_t)f.fileSize());
                }
                f.close();
            }
            root.close();
            Serial.println(F("FILES_DONE"));
            return;
        }

        if (strcmp_P(key, PSTR("SENDFILE")) == 0) {
            if (state == RUNNING) { Serial.println(F("ACK:ERR:Running")); return; }
            if (!sdReady) { Serial.println(F("ACK:ERR:SD not ready")); return; }
            char* fname = strtok(NULL, ":");
            if (!fname) { Serial.println(F("ACK:ERR:SENDFILE fmt")); return; }
            if (strchr(fname, '/') || strstr(fname, "..")) {
                Serial.println(F("ACK:ERR:SENDFILE path")); return;
            }
            FsFile f = SD.open(fname, O_READ);
            if (!f) { Serial.println(F("ACK:ERR:SENDFILE notfound")); return; }
            Serial.print(F("FILESTART:")); Serial.print(fname);
            Serial.print(':');             Serial.println((uint32_t)f.fileSize());
            uint32_t lineCount = 0;
            bool lineStart = true;
            while (f.available()) {
                char c = (char)f.read();
                if (c == '\r') continue;
                if (c == '\n') {
                    if (!lineStart) { Serial.println(); lineCount++; lineStart = true; }
                } else {
                    if (lineStart) { Serial.print(F("FLINE:")); lineStart = false; }
                    Serial.write(c);
                }
            }
            if (!lineStart) { Serial.println(); lineCount++; }
            f.close();
            Serial.print(F("FILEEND:")); Serial.println(lineCount);
            return;
        }

        if (strcmp_P(key, PSTR("TESTPIN")) == 0) {
            if (state == RUNNING) { Serial.println(F("ACK:ERR:Running")); return; }
            char* pinStr = strtok(NULL, ":");
            char* valStr = strtok(NULL, ":");
            if (!pinStr || !valStr) { Serial.println(F("ACK:ERR:TESTPIN fmt")); return; }
            int pin = atoi(pinStr);
            if (pin < 2 || pin > 53)  { Serial.println(F("ACK:ERR:TESTPIN pin")); return; }
            pinMode(pin, OUTPUT);
            digitalWrite(pin, atoi(valStr) ? LOW : HIGH);  // LOW=open (valve on), HIGH=closed
            Serial.println(F("ACK:OK"));
            return;
        }

        if (strcmp_P(key, PSTR("SETRTC")) == 0) {
            char* y  = strtok(NULL, ":");
            char* mo = strtok(NULL, ":");
            char* d  = strtok(NULL, ":");
            char* h  = strtok(NULL, ":");
            char* mi = strtok(NULL, ":");
            char* s  = strtok(NULL, ":");
            if (y && mo && d && h && mi && s) {
                RTC.adjust(DateTime(atoi(y), atoi(mo), atoi(d), atoi(h), atoi(mi), atoi(s)));
                Serial.println(F("ACK:OK"));
                Serial.println(F("MSG:RTC updated"));
            } else {
                Serial.println(F("ACK:ERR:SETRTC fmt"));
            }
            return;
        }

        if (strcmp_P(key, PSTR("START")) == 0) {
            if (state == IDLE) { Serial.println(F("ACK:ERR:Not configured")); return; }
            startExperiment();
            return;
        }

        if (strcmp_P(key, PSTR("RECOVER")) == 0) {
            if (state == IDLE)    { Serial.println(F("ACK:ERR:Not configured")); return; }
            if (state == RUNNING) { Serial.println(F("ACK:ERR:Running")); return; }
            if (!sdReady || !SD.exists("STATE.TXT")) { Serial.println(F("ACK:ERR:No recovery data")); return; }
            recoverExperiment();
            return;
        }
        return;
    }

    // ── CFG ──────────────────────────────────────────────────────────────────
    if (strcmp_P(cat, PSTR("CFG")) == 0) {
        if (state == RUNNING || state == PAUSED) {
            Serial.println(F("ACK:ERR:Running")); return;
        }
        char* key = strtok(NULL, ":");
        if (key == NULL) return;

        // ── per-channel config: CFG:CH:<ch>:<sub>[:<val>...] ─────────────────
        if (strcmp_P(key, PSTR("CH")) == 0) {
            char* chStr = strtok(NULL, ":");
            char* sub   = strtok(NULL, ":");
            if (!chStr || !sub) { Serial.println(F("ACK:ERR:CH fmt")); return; }
            int ch = atoi(chStr);
            if (ch < 0 || ch >= MAX_CHANNELS) { Serial.println(F("ACK:ERR:CH range")); return; }

            if (strcmp_P(sub, PSTR("MODE")) == 0) {
                char* val = strtok(NULL, ":");
                if (!val) { Serial.println(F("ACK:ERR:CH:MODE val")); return; }
                if      (strcmp_P(val, PSTR("MEASURE")) == 0)  chMode[ch] = CH_MEASURE;
                else if (strcmp_P(val, PSTR("SETPOINT")) == 0) chMode[ch] = CH_SETPOINT;
                else if (strcmp_P(val, PSTR("SEQUENCE")) == 0) chMode[ch] = CH_SEQUENCE;
                else { Serial.println(F("ACK:ERR:CH:MODE unknown")); return; }

            } else if (strcmp_P(sub, PSTR("START")) == 0) {
                // CFG:CH:<ch>:START:<Y>:<M>:<D>:<h>:<m>:<s>  (0:0:0:0:0:0 = immediate)
                char* y  = strtok(NULL, ":"); char* mo = strtok(NULL, ":");
                char* d  = strtok(NULL, ":"); char* h  = strtok(NULL, ":");
                char* mi = strtok(NULL, ":"); char* s  = strtok(NULL, ":");
                if (!y || !mo || !d || !h || !mi || !s) {
                    Serial.println(F("ACK:ERR:CH:START fmt")); return;
                }
                int iy = atoi(y);
                chStart[ch] = (iy == 0) ? 0
                    : DateTime(iy, atoi(mo), atoi(d), atoi(h), atoi(mi), atoi(s)).unixtime();

            } else if (strcmp_P(sub, PSTR("SETPOINT")) == 0) {
                char* val = strtok(NULL, ":");
                if (val) chSetpoint[ch] = atof(val);

            } else if (strcmp_P(sub, PSTR("DURATION")) == 0) {
                char* val = strtok(NULL, ":");
                if (val) chDurationMin[ch] = atol(val);

            } else if (strcmp_P(sub, PSTR("NPHASES")) == 0) {
                char* val = strtok(NULL, ":");
                if (val) chNPhases[ch] = (byte)constrain(atoi(val), 0, MAX_PHASES);

            } else if (strcmp_P(sub, PSTR("PHASE")) == 0) {
                // CFG:CH:<ch>:PHASE:<idx>:<sp>:<d>:<h>:<m>:<type>[:<minSP>:<maxSP>:<peakH>]
                char* idxStr  = strtok(NULL, ":"); char* spStr = strtok(NULL, ":");
                char* dStr    = strtok(NULL, ":"); char* hStr  = strtok(NULL, ":");
                char* mStr    = strtok(NULL, ":"); char* tStr  = strtok(NULL, ":");
                if (!idxStr || !spStr || !dStr || !hStr || !mStr || !tStr) {
                    Serial.println(F("ACK:ERR:CH:PHASE fmt")); return;
                }
                int idx = atoi(idxStr);
                if (idx < 0 || idx >= MAX_PHASES) {
                    Serial.println(F("ACK:ERR:CH:PHASE idx")); return;
                }
                chPhaseSP[ch][idx]     = atof(spStr);
                chPhaseDurSec[ch][idx] = (uint32_t)(atol(dStr) * 86400L
                                                   + atol(hStr) * 3600L
                                                   + atol(mStr) * 60L);
                chPhaseType[ch][idx]   = tStr[0];
                if (tStr[0] == 'd') {
                    char* minStr  = strtok(NULL, ":");
                    char* maxStr  = strtok(NULL, ":");
                    char* peakStr = strtok(NULL, ":");
                    if (!minStr || !maxStr || !peakStr) {
                        Serial.println(F("ACK:ERR:CH:PHASE daily params")); return;
                    }
                    chPhaseMinSP[ch][idx]    = atof(minStr);
                    chPhaseMaxSP[ch][idx]    = atof(maxStr);
                    chPhasePeakHour[ch][idx] = atof(peakStr);
                    if (chPhaseMinSP[ch][idx] >= chPhaseMaxSP[ch][idx]) {
                        Serial.println(F("ACK:ERR:CH:PHASE daily minSP>=maxSP")); return;
                    }
                } else if (tStr[0] == 'c') {
                    char* startStr = strtok(NULL, ":");
                    if (!startStr || atof(startStr) <= 0.0f) {
                        Serial.println(F("ACK:ERR:CH:PHASE c requires startDO>0")); return;
                    }
                    chPhaseMaxSP[ch][idx] = atof(startStr);
                }

            } else {
                Serial.println(F("ACK:ERR:CH:sub unknown")); return;
            }
            state = CONFIGURED;
            Serial.println(F("ACK:OK"));
            return;
        }

        // ── shared hardware config (unchanged from v1) ────────────────────────
        char* val = strtok(NULL, ":");

        if (strcmp_P(key, PSTR("NCHANNELS")) == 0) {
            nChannels = constrain(atoi(val), 1, MAX_CHANNELS);

        } else if (strcmp_P(key, PSTR("SENSORS")) == 0) {
            nSensors = constrain(atoi(val), 1, 2);
            if (nSensors == 1) s1Channels = nChannels;

        } else if (strcmp_P(key, PSTR("S1CHANNELS")) == 0) {
            s1Channels = constrain(atoi(val), 1, MAX_CHANNELS - 1);

        } else if (strcmp_P(key, PSTR("RELAY")) == 0) {
            int ch = atoi(val);
            char* pinStr = strtok(NULL, ":");
            if (ch >= 0 && ch < MAX_CHANNELS && pinStr) relayPins[ch] = atoi(pinStr);

        } else if (strcmp_P(key, PSTR("INTERVAL")) == 0) {
            sampInterval = atol(val);

        } else if (strcmp_P(key, PSTR("TANKID")) == 0) {
            int ch = atoi(val);
            char* idStr = strtok(NULL, ":");
            if (ch >= 0 && ch < MAX_CHANNELS && idStr) { strncpy(tankID[ch], idStr, 6); tankID[ch][6] = '\0'; }

        } else if (strcmp_P(key, PSTR("KP")) == 0) {
            int ch = atoi(val);
            char* fStr = strtok(NULL, ":");
            if (ch >= 0 && ch < MAX_CHANNELS && fStr) Kp[ch] = atof(fStr);

        } else if (strcmp_P(key, PSTR("KI")) == 0) {
            int ch = atoi(val);
            char* fStr = strtok(NULL, ":");
            if (ch >= 0 && ch < MAX_CHANNELS && fStr) Ki[ch] = atof(fStr);

        } else if (strcmp_P(key, PSTR("KD")) == 0) {
            int ch = atoi(val);
            char* fStr = strtok(NULL, ":");
            if (ch >= 0 && ch < MAX_CHANNELS && fStr) Kd[ch] = atof(fStr);

        } else {
            // Removed in v2: CFG:MODE, CFG:SETPOINT, CFG:DURATION, CFG:NPHASES, CFG:PHASE
            Serial.println(F("ACK:ERR:Unknown CFG key")); return;
        }

        state = CONFIGURED;
        Serial.println(F("ACK:OK"));
        return;
    }
}


// ═══════════════════════════════════════════════════════════════════════════════
//  Serial receive
// ═══════════════════════════════════════════════════════════════════════════════
void readSerial() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (recvIdx > 0) {
                recvBuf[recvIdx] = '\0';
                processCommand(recvBuf);
                recvIdx = 0;
            }
        } else if (recvIdx < RECV_BUF - 1) {
            recvBuf[recvIdx++] = c;
        }
    }
}


// ═══════════════════════════════════════════════════════════════════════════════
//  setup
// ═══════════════════════════════════════════════════════════════════════════════
void setup() {
    wdt_disable();                      // clear any WDT the bootloader left armed
    Serial.begin(19200);
    Serial.println(F("MSG:ArdoxyStandalone v2 ready"));

    lcd.begin(16, 2);
    lcd.setBacklight(WHITE);
    lcd.clear();
    lcd.print(F("Ardoxy v2"));

    Wire.begin();
    Wire.setWireTimeout(50000, true);   // 50ms I2C timeout; auto-resets bus on lockup

    // RTC
    lcd.setCursor(0, 1); lcd.print(F("Init RTC..."));
    if (!RTC.begin()) {
        Serial.println(F("MSG:RTC init failed"));
        lcd.clear(); lcd.print(F("RTC FAILED")); while (1);
    }
    if (!RTC.initialized()) {
        Serial.println(F("MSG:RTC not set — send CMD:SETRTC"));
        lcd.setCursor(0, 1); lcd.print(F("RTC not set!"));
        delay(2000);
    } else {
        DateTime now = RTC.now();
        // Sanity check: warn if year looks wrong
        if (now.year() < 2024) {
            Serial.println(F("MSG:RTC year looks wrong — check with CMD:SETRTC"));
        }
    }

    // SD
    lcd.clear(); lcd.print(F("Init SD..."));
    if (!SD.begin(CHIP_SELECT)) {
        Serial.println(F("MSG:SD init failed"));
        lcd.setCursor(0, 1); lcd.print(F("SD FAILED")); while (1);
    }
    sdReady = true;

    // Wait up to 10 s for serial config input
    lcd.clear();
    lcd.print(F("Waiting 10s..."));
    lcd.setCursor(0, 1); lcd.print(F("for serial cfg"));
    bool gotSerial = false;
    unsigned long waitStart = millis();
    while (millis() - waitStart < 10000UL) {
        if (Serial.available()) { gotSerial = true; break; }
        delay(100);
    }

    if (gotSerial) {
        // If a config exists on SD, load it so the user can start/recover without re-uploading
        lcd.clear(); lcd.print(F("Serial mode..."));
        if (sdReady && loadConfig()) {
            state = CONFIGURED;
            Serial.println(F("MSG:Config loaded from SD — ready"));
            if (SD.exists("STATE.TXT")) {
                Serial.println(F("MSG:Recovery data found — CMD:RECOVER to resume"));
            }
        } else {
            Serial.println(F("MSG:No SD config — send CFG: commands"));
        }
    } else {
        // Auto-start from SD
        lcd.clear(); lcd.print(F("Loading SD cfg.."));
        if (!loadConfig()) {
            Serial.println(F("MSG:No CONFIG.TXT — waiting for serial"));
            lcd.clear(); lcd.print(F("No CONFIG.TXT"));
            lcd.setCursor(0, 1); lcd.print(F("Connect GUI"));
            // Fall through: loop() will handle serial input
        } else {
            state = CONFIGURED;
            if (readState()) {
                lcd.clear(); lcd.print(F("Recovering..."));
                recoverExperiment();
            } else {
                startExperiment();
            }
        }
    }
}


// ═══════════════════════════════════════════════════════════════════════════════
//  loop
// ═══════════════════════════════════════════════════════════════════════════════
void loop() {
    readSerial();

    if (state == PAUSED || state != RUNNING) return;

    runAllChannels();
}
