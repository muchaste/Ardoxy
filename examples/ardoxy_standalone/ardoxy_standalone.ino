/*
  ardoxy_standalone.ino
  Ardoxy Standalone Sketch — upload once, configure via ardoxy_gui.py (Standalone Mode),
  then run autonomously with SD card logging and LCD display.
  Power-outage recovery is automatic via STATE.TXT on the SD card.

  Hardware:
    - Arduino MEGA2560
    - FireSting 1 on Serial1 (MEGA pins 18 RX1 / 19 TX1)
    - FireSting 2 on Serial2 (MEGA pins 17 RX2 / 16 TX2)  — optional, 2-sensor mode
    - Adafruit Datalogger Shield  (SD on CS=10, RTC PCF8523 via I2C)
    - Adafruit RGB LCD Shield     (16×2, I2C via MCP23017)
    - Relay module on digital output pins (default: 46, 48, 50, 52, 22, 24, 26, 28)

  Protocol (USB serial, 19200 baud, newline-terminated):
    PC -> Arduino:
      CFG:MODE:<MEASURE|SETPOINT|SEQUENCE>
      CFG:NCHANNELS:<1-8>
      CFG:SENSORS:<1|2>              number of FireSting sensors (default 1)
      CFG:S1CHANNELS:<n>             channels on sensor 1; required when SENSORS=2
      CFG:RELAY:<ch>:<pin>           ch = 0-based
      CFG:INTERVAL:<ms>
      CFG:DURATION:<minutes>         SETPOINT total experiment duration
      CFG:TANKID:<ch>:<id>           up to 6 chars; used in SD log header + LCD
      CFG:SETPOINT:<float>           SETPOINT mode setpoint
      CFG:KP:<ch>:<float>            per-channel PID gains (ch = 0-based)
      CFG:KI:<ch>:<float>
      CFG:KD:<ch>:<float>
      CFG:NPHASES:<n>                SEQUENCE mode
      CFG:PHASE:<idx>:<sp>:<dur_d>:<dur_h>:<dur_m>:<type>[:<min_sp>:<max_sp>:<peak_h>]
        type: h=hold  c=change  p=pause  d=daily-cycle
        min_sp/max_sp/peak_h only required when type == 'd'
      CMD:START
      CMD:STOP
      CMD:PAUSE
      CMD:RESUME
      CMD:STATUS
      CMD:SAVECONFIG                 write current config to CONFIG.TXT on SD
      CMD:SETRTC:<Y>:<M>:<D>:<h>:<m>:<s>   set RTC from PC time
    Arduino -> PC:
      ACK:OK
      ACK:ERR:<msg>
      STATUS:<IDLE|CONFIGURED|RUNNING|PAUSED>
      DATA:<elapsed_ms>,<do_ch1[,do_ch2...]>,<temp>,<output[,output...]>,<sp>,<phase>,<ptype>
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
*/

#include <Ardoxy.h>
#include <PID_v1.h>
#include <SdFat.h>
#include <Wire.h>
#include "RTClib.h"
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>

#define WHITE        0x7
#define MAX_CHANNELS 8
#define MAX_PHASES   10
#define RECV_BUF     96
#define CHIP_SELECT  10    // Adafruit datalogger shield

// ─── hardware instances ───────────────────────────────────────────────────────
Ardoxy              ardoxy(Serial1);          // FireSting 1 on Serial1 (MEGA pins 18/19)
Ardoxy              ardoxy2(Serial2);         // FireSting 2 on Serial2 (MEGA pins 16/17) — 2-sensor mode
RTC_PCF8523         RTC;
SdFs                SD;
FsFile              logFile;
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

// ─── state machine ────────────────────────────────────────────────────────────
typedef enum { IDLE, CONFIGURED, RUNNING, PAUSED } State;
typedef enum { MEASURE, SETPOINT, SEQUENCE } Mode;
State state = IDLE;
Mode  mode  = MEASURE;

// ─── config ───────────────────────────────────────────────────────────────────
int    nChannels          = 1;
int    nSensors           = 1;           // 1 or 2 FireSting sensors
int    s1Channels         = 1;           // channels on sensor 1 (= nChannels when nSensors == 1)
int    relayPins[MAX_CHANNELS]   = {46, 48, 50, 52, 22, 24, 26, 28};
long   sampInterval       = 30000UL;          // ms
long   experimentDuration = 1440;             // minutes (SETPOINT mode)
float  DOSetpoint         = 30.0;
float  Kp[MAX_CHANNELS]   = {10, 10, 10, 10, 10, 10, 10, 10};
float  Ki[MAX_CHANNELS]   = { 1,  1,  1,  1,  1,  1,  1,  1};
float  Kd[MAX_CHANNELS]   = { 0,  0,  0,  0,  0,  0,  0,  0};
char   tankID[MAX_CHANNELS][7] = {"CH1","CH2","CH3","CH4","CH5","CH6","CH7","CH8"};

// sequence config
int      nPhases = 0;
float    phaseSetpoints[MAX_PHASES];
uint32_t phaseDurSec[MAX_PHASES];     // phase duration in seconds
char     phaseTypes[MAX_PHASES];      // 'h','c','p','d'
float    phaseMinSP[MAX_PHASES];      // 'd' type: minimum setpoint
float    phaseMaxSP[MAX_PHASES];      // 'd' type: maximum setpoint
float    phasePeakHour[MAX_PHASES];   // 'd' type: hour of maximum (0–24)

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

double seqRateInput[MAX_CHANNELS] = {0};
double seqRateSP[MAX_CHANNELS]    = {0};
PID seqRatePID0(&seqRateInput[0],&doOutput[0],&seqRateSP[0],0,1,0,REVERSE);
PID seqRatePID1(&seqRateInput[1],&doOutput[1],&seqRateSP[1],0,1,0,REVERSE);
PID seqRatePID2(&seqRateInput[2],&doOutput[2],&seqRateSP[2],0,1,0,REVERSE);
PID seqRatePID3(&seqRateInput[3],&doOutput[3],&seqRateSP[3],0,1,0,REVERSE);
PID seqRatePID4(&seqRateInput[4],&doOutput[4],&seqRateSP[4],0,1,0,REVERSE);
PID seqRatePID5(&seqRateInput[5],&doOutput[5],&seqRateSP[5],0,1,0,REVERSE);
PID seqRatePID6(&seqRateInput[6],&doOutput[6],&seqRateSP[6],0,1,0,REVERSE);
PID seqRatePID7(&seqRateInput[7],&doOutput[7],&seqRateSP[7],0,1,0,REVERSE);
PID* seqRatePIDs[MAX_CHANNELS] = {&seqRatePID0,&seqRatePID1,&seqRatePID2,&seqRatePID3,
                                   &seqRatePID4,&seqRatePID5,&seqRatePID6,&seqRatePID7};

// ─── serial receive buffer ────────────────────────────────────────────────────
char recvBuf[RECV_BUF];
int  recvIdx = 0;

// ─── runtime state ────────────────────────────────────────────────────────────
DateTime expStart;
uint32_t phaseEndUnix[MAX_PHASES + 1]; // RTC unixtime of each phase boundary
uint32_t setpointEndUnix = 0;
uint32_t pauseStartUnix  = 0;
int      phaseIdx         = 0;
int      windowSize       = 0;
double   doFloatPrev[MAX_CHANNELS] = {0};
int      rateReCalc       = 0;
int      samplesSinceCalc = 0;
unsigned long loopStart   = 0;

// ─── SD / logging state ───────────────────────────────────────────────────────
char     filename[22];   // "YYYY_MM_DD_HH_MM.csv\0"
uint32_t rowN      = 0;
int      lastLogDay = 0;
bool     sdReady   = false;

// ─── LCD state ────────────────────────────────────────────────────────────────
int    lcdPage     = 0;
int    lcdNumPages = 1;
double lastTemp    = 0.0;   // sensor 1 temperature
double lastTemp2   = 0.0;   // sensor 2 temperature (2-sensor mode only)
double lastDO[MAX_CHANNELS] = {0};

// ─── error handling ───────────────────────────────────────────────────────────
int  errorCount = 0;
void (*resetFunc)(void) = 0;   // software reboot to address 0


// ═══════════════════════════════════════════════════════════════════════════════
//  Helper: daily-cycle setpoint from RTC time
// ═══════════════════════════════════════════════════════════════════════════════
float dailyCycleSP(int idx, float hourDecimal) {
    float mean = (phaseMinSP[idx] + phaseMaxSP[idx]) / 2.0;
    float ampl = (phaseMaxSP[idx] - phaseMinSP[idx]) / 2.0;
    return mean + ampl * cos(2.0 * PI * (hourDecimal - phasePeakHour[idx]) / 24.0);
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
//  Serial data emission (for live monitoring; SD is the primary log)
// ═══════════════════════════════════════════════════════════════════════════════
void emitData(uint32_t elapsedMs, double* doVals, double* tempVals,
              float sp, int pidx, char ptype) {
    Serial.print(F("DATA:"));
    Serial.print(elapsedMs);
    for (int i = 0; i < nChannels; i++) { Serial.print(','); Serial.print(doVals[i], 3); }
    Serial.print(','); Serial.print(tempVals[0], 3);
    if (nSensors == 2) { Serial.print(','); Serial.print(tempVals[1], 3); }
    for (int i = 0; i < nChannels; i++) {
        Serial.print(',');
        Serial.print((long)(doOutput[i]) * 200);
    }
    Serial.print(','); Serial.print(sp, 2);
    Serial.print(','); Serial.print(pidx);
    Serial.print(','); Serial.println(ptype);
}


// ═══════════════════════════════════════════════════════════════════════════════
//  LCD: non-blocking cycle through pages once per measurement cycle
// ═══════════════════════════════════════════════════════════════════════════════
void lcdUpdate(float sp, char ptype) {
    lcd.clear();

    if (lcdPage == 0) {
        // ── page 0: phase / setpoint / time remaining ──
        lcd.setCursor(0, 0);
        if (mode == SEQUENCE) {
            lcd.print("Ph"); lcd.print(phaseIdx + 1);
            lcd.print('/'); lcd.print(nPhases);
            lcd.print(' '); lcd.print(ptype);
            lcd.print(" SP:");
            if (sp < 100) lcd.print(sp, 1); else lcd.print((int)sp);
        } else if (mode == SETPOINT) {
            lcd.print("SP:");
            lcd.print(sp, 1);
            lcd.print(" ");
        } else {
            lcd.print("MEASURE");
        }

        lcd.setCursor(0, 1);
        if (mode == SEQUENCE && phaseIdx < nPhases) {
            uint32_t nowUnix = RTC.now().unixtime();
            long remSec = (long)(phaseEndUnix[phaseIdx + 1] - nowUnix);
            if (remSec < 0) remSec = 0;
            int remD = remSec / 86400;
            int remH = (remSec % 86400) / 3600;
            int remM = (remSec % 3600) / 60;
            lcd.print("Rem:");
            lcd.print(remD); lcd.print('d');
            lcd.print(remH); lcd.print('h');
            lcd.print(remM); lcd.print('m');
        } else if (mode == SETPOINT) {
            uint32_t nowUnix = RTC.now().unixtime();
            long remSec = (long)(setpointEndUnix - nowUnix);
            if (remSec < 0) remSec = 0;
            int remH = remSec / 3600;
            int remM = (remSec % 3600) / 60;
            lcd.print("Rem:"); lcd.print(remH); lcd.print('h'); lcd.print(remM); lcd.print('m');
        }
        lcd.print(" T:"); lcd.print((int)round(lastTemp));

    } else {
        // ── pages 1..nChannels: per-channel DO ──
        int ch = lcdPage - 1;
        if (ch < nChannels) {
            lcd.setCursor(0, 0);
            lcd.print(tankID[ch]); lcd.print(':');
            lcd.print(lastDO[ch], 1); lcd.print("% air");
            lcd.setCursor(0, 1);
            lcd.print("Out:"); lcd.print((long)(doOutput[ch]) * 200); lcd.print("ms");
        }
    }

    lcdPage = (lcdPage + 1) % lcdNumPages;
}


// ═══════════════════════════════════════════════════════════════════════════════
//  SD: create daily log file with metadata header
// ═══════════════════════════════════════════════════════════════════════════════
void createLogfile() {
    DateTime now = RTC.now();
    sprintf(filename, "%04d_%02d_%02d_%02d_%02d.csv",
            now.year(), now.month(), now.day(), now.hour(), now.minute());
    logFile = SD.open(filename, FILE_WRITE);
    if (!logFile) {
        Serial.println(F("MSG:SD logfile create failed"));
        lcd.clear(); lcd.print("SD error!"); return;
    }
    lastLogDay = now.day();

    // Metadata header
    logFile.print(F("Date:;")); logFile.print(now.year()); logFile.print('/');
    logFile.print(now.month()); logFile.print('/'); logFile.println(now.day());
    logFile.print(F("Mode:;"));
    if      (mode == MEASURE)  logFile.println(F("MEASURE"));
    else if (mode == SETPOINT) logFile.println(F("SETPOINT"));
    else                       logFile.println(F("SEQUENCE"));
    logFile.print(F("Interval_ms:;")); logFile.println(sampInterval);
    logFile.print(F("Channels:;"));    logFile.println(nChannels);
    logFile.print(F("Sensors:;"));     logFile.println(nSensors);
    if (nSensors == 2) { logFile.print(F("S1Channels:;")); logFile.println(s1Channels); }
    logFile.print(F("TankIDs:;"));
    for (int i = 0; i < nChannels; i++) { logFile.print(tankID[i]); logFile.print(';'); }
    logFile.println();
    logFile.print(F("RelayPins:;"));
    for (int i = 0; i < nChannels; i++) { logFile.print(relayPins[i]); logFile.print(';'); }
    logFile.println();
    // Column headers
    logFile.print(F("n;Date;Time;Temp1_C;"));
    if (nSensors == 2) logFile.print(F("Temp2_C;"));
    for (int i = 0; i < nChannels; i++) {
        logFile.print(F("DO_")); logFile.print(tankID[i]); logFile.print(';');
    }
    for (int i = 0; i < nChannels; i++) {
        logFile.print(F("Out_ms_")); logFile.print(tankID[i]); logFile.print(';');
    }
    logFile.println(F("Setpoint;Phase;PhaseType"));
    logFile.flush();
    Serial.print(F("MSG:Logfile ")); Serial.println(filename);
}


// ═══════════════════════════════════════════════════════════════════════════════
//  SD: append one measurement row
// ═══════════════════════════════════════════════════════════════════════════════
void writeToSD(double* doVals, double* tempVals, float sp, int pidx, char ptype) {
    if (!logFile) return;
    DateTime now = RTC.now();

    // Daily log rotation
    if (now.day() != lastLogDay) {
        logFile.close();
        createLogfile();
    }

    rowN++;
    logFile.print(rowN);           logFile.print(';');
    logFile.print(now.year());     logFile.print('/');
    logFile.print(now.month());    logFile.print('/');
    logFile.print(now.day());      logFile.print(';');
    logFile.print(now.hour());     logFile.print(':');
    logFile.print(now.minute());   logFile.print(':');
    logFile.print(now.second());   logFile.print(';');
    logFile.print(tempVals[0], 3); logFile.print(';');
    if (nSensors == 2) { logFile.print(tempVals[1], 3); logFile.print(';'); }
    for (int i = 0; i < nChannels; i++) { logFile.print(doVals[i], 3); logFile.print(';'); }
    for (int i = 0; i < nChannels; i++) {
        logFile.print((long)(doOutput[i]) * 200); logFile.print(';');
    }
    logFile.print(sp, 2);      logFile.print(';');
    logFile.print(pidx);       logFile.print(';');
    logFile.println(ptype);
    logFile.flush();
}


// ═══════════════════════════════════════════════════════════════════════════════
//  SD: persist experiment state after every cycle (power-outage recovery)
// ═══════════════════════════════════════════════════════════════════════════════
void writeState() {
    if (!sdReady) return;
    FsFile sf = SD.open("STATE.TXT", O_WRITE | O_CREAT | O_TRUNC);
    if (!sf) return;
    sf.print(filename);            sf.print(';');
    sf.print(rowN);                sf.print(';');
    sf.print(expStart.year());     sf.print(';');
    sf.print(expStart.month());    sf.print(';');
    sf.print(expStart.day());      sf.print(';');
    sf.print(expStart.hour());     sf.print(';');
    sf.print(expStart.minute());   sf.print(';');
    sf.print(expStart.second());   sf.print(';');
    sf.print(phaseIdx);            sf.print(';');
    for (int i = 0; i < MAX_CHANNELS; i++) { sf.print(lastDO[i], 3); sf.print(';'); }
    sf.print(lastTemp, 3);   sf.print(';');
    sf.println(lastTemp2, 3);
    sf.close();
}


// ═══════════════════════════════════════════════════════════════════════════════
//  SD: read experiment state on boot
// ═══════════════════════════════════════════════════════════════════════════════
bool readState() {
    if (!sdReady || !SD.exists("STATE.TXT")) return false;
    FsFile sf = SD.open("STATE.TXT", O_RDONLY);
    if (!sf) return false;
    char buf[260];
    int len = sf.read(buf, sizeof(buf) - 1);
    sf.close();
    if (len <= 0) return false;
    buf[len] = '\0';

    char* tok;
    tok = strtok(buf, ";"); if (!tok) return false;
    strncpy(filename, tok, sizeof(filename) - 1); filename[sizeof(filename)-1] = '\0';

    tok = strtok(NULL, ";"); if (!tok) return false; rowN = (uint32_t)atol(tok);

    int ey, em, ed, eh, emi, es;
    tok = strtok(NULL, ";"); if (!tok) return false; ey  = atoi(tok);
    tok = strtok(NULL, ";"); if (!tok) return false; em  = atoi(tok);
    tok = strtok(NULL, ";"); if (!tok) return false; ed  = atoi(tok);
    tok = strtok(NULL, ";"); if (!tok) return false; eh  = atoi(tok);
    tok = strtok(NULL, ";"); if (!tok) return false; emi = atoi(tok);
    tok = strtok(NULL, ";"); if (!tok) return false; es  = atoi(tok);
    expStart = DateTime(ey, em, ed, eh, emi, es);

    tok = strtok(NULL, ";"); if (!tok) return false; phaseIdx = atoi(tok);

    for (int i = 0; i < MAX_CHANNELS; i++) {
        tok = strtok(NULL, ";");
        if (tok) lastDO[i] = atof(tok);
    }
    tok = strtok(NULL, ";");
    if (tok) lastTemp = atof(tok);
    tok = strtok(NULL, ";\r\n");
    if (tok) lastTemp2 = atof(tok);
    return true;
}


// ═══════════════════════════════════════════════════════════════════════════════
//  SD: save current config to CONFIG.TXT
// ═══════════════════════════════════════════════════════════════════════════════
void saveConfig() {
    if (!sdReady) { Serial.println(F("ACK:ERR:SD not ready")); return; }
    FsFile cf = SD.open("CONFIG.TXT", O_WRITE | O_CREAT | O_TRUNC);
    if (!cf) { Serial.println(F("ACK:ERR:SD write")); return; }

    cf.print(F("MODE="));
    if      (mode == MEASURE)  cf.println(F("MEASURE"));
    else if (mode == SETPOINT) cf.println(F("SETPOINT"));
    else                       cf.println(F("SEQUENCE"));
    cf.print(F("NCHANNELS=")); cf.println(nChannels);
    cf.print(F("SENSORS="));   cf.println(nSensors);
    cf.print(F("S1CHANNELS=")); cf.println(s1Channels);
    for (int i = 0; i < MAX_CHANNELS; i++) {
        cf.print(F("RELAY_")); cf.print(i); cf.print('='); cf.println(relayPins[i]);
    }
    cf.print(F("INTERVAL=")); cf.println(sampInterval);
    cf.print(F("DURATION=")); cf.println(experimentDuration);
    for (int i = 0; i < MAX_CHANNELS; i++) {
        cf.print(F("TANKID_")); cf.print(i); cf.print('='); cf.println(tankID[i]);
    }
    cf.print(F("SETPOINT=")); cf.println(DOSetpoint);
    for (int i = 0; i < MAX_CHANNELS; i++) {
        cf.print(F("KP_")); cf.print(i); cf.print('='); cf.println(Kp[i]);
        cf.print(F("KI_")); cf.print(i); cf.print('='); cf.println(Ki[i]);
        cf.print(F("KD_")); cf.print(i); cf.print('='); cf.println(Kd[i]);
    }
    cf.print(F("NPHASES=")); cf.println(nPhases);
    for (int i = 0; i < nPhases; i++) {
        cf.print(F("PHASE_")); cf.print(i); cf.print('=');
        cf.print(phaseSetpoints[i]);              cf.print(',');
        cf.print(phaseDurSec[i] / 86400UL);       cf.print(',');  // days
        cf.print((phaseDurSec[i] % 86400UL) / 3600UL); cf.print(','); // hours
        cf.print((phaseDurSec[i] % 3600UL)  / 60UL);   cf.print(','); // minutes
        cf.print(phaseTypes[i]);                  cf.print(',');
        cf.print(phaseMinSP[i]);                  cf.print(',');
        cf.print(phaseMaxSP[i]);                  cf.print(',');
        cf.println(phasePeakHour[i]);
    }
    cf.close();
    Serial.println(F("ACK:OK"));
    Serial.println(F("MSG:Config saved to SD"));
}


// ═══════════════════════════════════════════════════════════════════════════════
//  SD: load config from CONFIG.TXT
// ═══════════════════════════════════════════════════════════════════════════════
bool loadConfig() {
    if (!sdReady || !SD.exists("CONFIG.TXT")) return false;
    FsFile cf = SD.open("CONFIG.TXT", O_RDONLY);
    if (!cf) return false;
    static char cfgBuf[1024];
    int len = cf.read(cfgBuf, sizeof(cfgBuf) - 1);
    cf.close();
    if (len <= 0) return false;
    cfgBuf[len] = '\0';

    char* line = strtok(cfgBuf, "\n\r");
    while (line != NULL) {
        char* eq = strchr(line, '=');
        if (!eq) { line = strtok(NULL, "\n\r"); continue; }
        *eq = '\0';
        char* key = line;
        char* val = eq + 1;

        if (strcmp(key, "MODE") == 0) {
            if      (strcmp(val, "MEASURE") == 0)  mode = MEASURE;
            else if (strcmp(val, "SETPOINT") == 0) mode = SETPOINT;
            else if (strcmp(val, "SEQUENCE") == 0) mode = SEQUENCE;
        } else if (strcmp(key, "NCHANNELS") == 0) {
            nChannels = constrain(atoi(val), 1, MAX_CHANNELS);
        } else if (strcmp(key, "SENSORS") == 0) {
            nSensors = constrain(atoi(val), 1, 2);
        } else if (strcmp(key, "S1CHANNELS") == 0) {
            s1Channels = constrain(atoi(val), 1, MAX_CHANNELS - 1);
        } else if (strncmp(key, "RELAY_", 6) == 0) {
            int ch = atoi(key + 6);
            if (ch >= 0 && ch < MAX_CHANNELS) relayPins[ch] = atoi(val);
        } else if (strcmp(key, "INTERVAL") == 0) {
            sampInterval = atol(val);
        } else if (strcmp(key, "DURATION") == 0) {
            experimentDuration = atol(val);
        } else if (strncmp(key, "TANKID_", 7) == 0) {
            int ch = atoi(key + 7);
            if (ch >= 0 && ch < MAX_CHANNELS) strncpy(tankID[ch], val, 6);
        } else if (strcmp(key, "SETPOINT") == 0) {
            DOSetpoint = atof(val);
        } else if (strncmp(key, "KP_", 3) == 0) {
            int ch = atoi(key + 3);
            if (ch >= 0 && ch < MAX_CHANNELS) Kp[ch] = atof(val);
        } else if (strncmp(key, "KI_", 3) == 0) {
            int ch = atoi(key + 3);
            if (ch >= 0 && ch < MAX_CHANNELS) Ki[ch] = atof(val);
        } else if (strncmp(key, "KD_", 3) == 0) {
            int ch = atoi(key + 3);
            if (ch >= 0 && ch < MAX_CHANNELS) Kd[ch] = atof(val);
        } else if (strcmp(key, "NPHASES") == 0) {
            nPhases = constrain(atoi(val), 0, MAX_PHASES);
        } else if (strncmp(key, "PHASE_", 6) == 0) {
            int idx = atoi(key + 6);
            if (idx >= 0 && idx < MAX_PHASES) {
                // val = "sp,days,hours,minutes,type,minSP,maxSP,peakH"
                char* vt = strtok(val, ",");
                if (vt) phaseSetpoints[idx] = atof(vt);
                vt = strtok(NULL, ","); long dd = vt ? atol(vt) : 0;
                vt = strtok(NULL, ","); long hh = vt ? atol(vt) : 0;
                vt = strtok(NULL, ","); long mm = vt ? atol(vt) : 0;
                phaseDurSec[idx] = (uint32_t)(dd * 86400L + hh * 3600L + mm * 60L);
                vt = strtok(NULL, ","); if (vt) phaseTypes[idx] = vt[0];
                vt = strtok(NULL, ","); if (vt) phaseMinSP[idx]    = atof(vt);
                vt = strtok(NULL, ","); if (vt) phaseMaxSP[idx]    = atof(vt);
                vt = strtok(NULL, ",\r\n"); if (vt) phasePeakHour[idx] = atof(vt);
            }
        }
        line = strtok(NULL, "\n\r");
    }
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
    for (int i = 0; i < nChannels; i++) {
        Ardoxy::configurePID(*seqRatePIDs[i], 0, Ki[i], 0, sampInterval, windowSize);
        seqRatePIDs[i]->SetMode(MANUAL);
    }
    lcdNumPages = nChannels + 1;
    lcdPage = 0;
    ardoxy.begin();
    if (nSensors == 2) ardoxy2.begin();
}


// ═══════════════════════════════════════════════════════════════════════════════
//  Start experiment fresh
// ═══════════════════════════════════════════════════════════════════════════════
void startExperiment() {
    initHardware();
    expStart = RTC.now();
    rowN = 0;

    if (mode == SETPOINT) {
        setpointEndUnix = expStart.unixtime() + (uint32_t)experimentDuration * 60UL;
        for (int i = 0; i < nChannels; i++) holdSP[i] = DOSetpoint;
    } else if (mode == SEQUENCE) {
        phaseIdx = 0;
        phaseEndUnix[0] = expStart.unixtime();
        for (int i = 0; i < nPhases; i++)
            phaseEndUnix[i + 1] = phaseEndUnix[i] + phaseDurSec[i];
        for (int i = 0; i < nChannels; i++) {
            holdSP[i]       = phaseSetpoints[0];
            seqRateSP[i]    = 0.0;
            doFloatPrev[i]  = 0.0;
            seqRatePIDs[i]->SetMode(AUTOMATIC);
        }
        samplesSinceCalc = 0;
    }

    createLogfile();
    state = RUNNING;
    Serial.println(F("ACK:OK"));
    Serial.println(F("MSG:Running"));
}


// ═══════════════════════════════════════════════════════════════════════════════
//  Recover experiment from STATE.TXT after power outage
//  Assumes config already loaded via loadConfig() and state read via readState()
// ═══════════════════════════════════════════════════════════════════════════════
void recoverExperiment() {
    initHardware();

    // Recompute phase boundaries from recovered expStart
    if (mode == SEQUENCE) {
        phaseEndUnix[0] = expStart.unixtime();
        for (int i = 0; i < nPhases; i++)
            phaseEndUnix[i + 1] = phaseEndUnix[i] + phaseDurSec[i];

        // Find current phase from RTC
        uint32_t nowUnix = RTC.now().unixtime();
        while (phaseIdx < nPhases && nowUnix >= phaseEndUnix[phaseIdx + 1])
            phaseIdx++;

        if (phaseIdx >= nPhases) {
            // Experiment already finished during downtime
            ardoxy.end();
            if (nSensors == 2) ardoxy2.end();
            Ardoxy::closeRelays(nChannels, relayPins);
            Serial.println(F("MSG:Sequence finished during downtime"));
            lcd.clear(); lcd.print("Seq complete!"); lcd.setCursor(0,1); lcd.print("Connect to reset");
            state = CONFIGURED;
            return;
        }
        // Restore doFloatPrev for 'c' phase continuity
        for (int i = 0; i < nChannels; i++) doFloatPrev[i] = lastDO[i];
        if (phaseTypes[phaseIdx] == 'c') {
            samplesSinceCalc = 0;
            for (int i = 0; i < nChannels; i++) seqRatePIDs[i]->SetMode(AUTOMATIC);
        }
        if (phaseTypes[phaseIdx] == 'h') {
            for (int i = 0; i < nChannels; i++) holdSP[i] = phaseSetpoints[phaseIdx];
        }

    } else if (mode == SETPOINT) {
        setpointEndUnix = expStart.unixtime() + (uint32_t)experimentDuration * 60UL;
        if (RTC.now().unixtime() >= setpointEndUnix) {
            ardoxy.end();
            if (nSensors == 2) ardoxy2.end();
            Ardoxy::closeRelays(nChannels, relayPins);
            Serial.println(F("MSG:Setpoint experiment finished during downtime"));
            state = CONFIGURED;
            return;
        }
        for (int i = 0; i < nChannels; i++) holdSP[i] = DOSetpoint;
    }

    // Re-open or create logfile
    if (SD.exists(filename)) {
        logFile = SD.open(filename, FILE_WRITE);
        if (logFile) {
            DateTime now = RTC.now();
            lastLogDay = now.day();
            logFile.print(F("RESTART;"));
            logFile.print(now.year()); logFile.print('/');
            logFile.print(now.month()); logFile.print('/');
            logFile.print(now.day()); logFile.print(';');
            logFile.print(now.hour()); logFile.print(':');
            logFile.print(now.minute()); logFile.print(':');
            logFile.print(now.second());
            logFile.println(F(";Recovered from STATE.TXT"));
            logFile.flush();
        }
    } else {
        createLogfile();
    }

    state = RUNNING;
    Serial.println(F("MSG:Recovered from STATE.TXT"));
    lcd.clear(); lcd.print("Recovered!"); lcd.setCursor(0,1); lcd.print(filename);
    delay(1500);
}


// ═══════════════════════════════════════════════════════════════════════════════
//  Run modes
// ═══════════════════════════════════════════════════════════════════════════════

void runMeasure() {
    loopStart = millis();
    double doVals[MAX_CHANNELS];
    double tempVals[2] = {0, 0};

    if (!measureAllChannels(doVals, tempVals)) {
        Serial.println(F("MSG:Sensor error"));
        if (++errorCount >= 50) resetFunc();
        return;
    }
    errorCount = 0;
    for (int i = 0; i < nChannels; i++) lastDO[i] = doVals[i];
    lastTemp = tempVals[0]; lastTemp2 = tempVals[1];

    uint32_t elapsedMs = (RTC.now().unixtime() - expStart.unixtime()) * 1000UL;
    emitData(elapsedMs, doVals, tempVals, 0.0, 0, 'm');
    writeToSD(doVals, tempVals, 0.0, 0, 'm');
    writeState();
    lcdUpdate(0.0, 'm');

    long rem = sampInterval - (long)(millis() - loopStart);
    if (rem > 0) delay(rem);
}

// ─────────────────────────────────────────────────────────────────────────────

void runSetpoint() {
    if (RTC.now().unixtime() >= setpointEndUnix) {
        Ardoxy::closeRelays(nChannels, relayPins);
        ardoxy.end();
        if (nSensors == 2) ardoxy2.end();
        logFile.close();
        Serial.println(F("DONE"));
        lcd.clear(); lcd.print("DONE"); lcd.setCursor(0,1); lcd.print("Setpoint done");
        state = IDLE;
        return;
    }

    loopStart = millis();
    double doVals[MAX_CHANNELS];
    double tempVals[2] = {0, 0};

    if (!measureAllChannels(doVals, tempVals)) {
        Serial.println(F("MSG:Sensor error"));
        if (++errorCount >= 50) resetFunc();
        Ardoxy::closeRelays(nChannels, relayPins);
        return;
    }
    errorCount = 0;
    for (int i = 0; i < nChannels; i++) {
        lastDO[i]  = doVals[i];
        doInput[i] = doVals[i];
        valvePIDs[i]->Compute();
    }
    lastTemp = tempVals[0]; lastTemp2 = tempVals[1];
    Ardoxy::scheduleRelays(nChannels, doOutput, relayPins,
                           sampInterval - ((long)nChannels * 40 + 500));

    uint32_t elapsedMs = (RTC.now().unixtime() - expStart.unixtime()) * 1000UL;
    emitData(elapsedMs, doVals, tempVals, DOSetpoint, 0, 's');
    writeToSD(doVals, tempVals, DOSetpoint, 0, 's');
    writeState();
    lcdUpdate(DOSetpoint, 's');

    long rem = sampInterval - (long)(millis() - loopStart);
    if (rem > 0) delay(rem);
}

// ─────────────────────────────────────────────────────────────────────────────

void runSequence() {
    if (phaseIdx >= nPhases) {
        Ardoxy::closeRelays(nChannels, relayPins);
        ardoxy.end();
        if (nSensors == 2) ardoxy2.end();
        logFile.close();
        Serial.println(F("DONE"));
        lcd.clear(); lcd.print("DONE"); lcd.setCursor(0,1); lcd.print("Sequence done");
        state = IDLE;
        return;
    }

    // RTC-based phase advance
    uint32_t nowUnix = RTC.now().unixtime();
    if (nowUnix >= phaseEndUnix[phaseIdx + 1]) {
        phaseIdx++;
        if (phaseIdx >= nPhases) return;   // caught at top of next call
        if (phaseTypes[phaseIdx] == 'c') samplesSinceCalc = 0;
        Serial.print(F("MSG:Phase ")); Serial.println(phaseIdx);
        lcd.clear(); lcd.print("New phase "); lcd.print(phaseIdx + 1);
    }

    char ptype = phaseTypes[phaseIdx];

    if (ptype == 'p') {
        Ardoxy::closeRelays(nChannels, relayPins);
        long rem = sampInterval - (long)(millis() - loopStart);
        if (rem > 0) delay(rem);
        loopStart = millis();
        return;
    }

    loopStart = millis();
    double doVals[MAX_CHANNELS];
    double tempVals[2] = {0, 0};

    if (!measureAllChannels(doVals, tempVals)) {
        Serial.println(F("MSG:Sensor error"));
        if (++errorCount >= 50) resetFunc();
        Ardoxy::closeRelays(nChannels, relayPins);
        return;
    }
    errorCount = 0;
    for (int i = 0; i < nChannels; i++) lastDO[i] = doVals[i];
    lastTemp = tempVals[0]; lastTemp2 = tempVals[1];

    float currentSP = phaseSetpoints[phaseIdx];

    if (ptype == 'h') {
        for (int i = 0; i < nChannels; i++) {
            holdSP[i]  = currentSP;
            doInput[i] = doVals[i];
            valvePIDs[i]->Compute();
        }
        Ardoxy::scheduleRelays(nChannels, doOutput, relayPins,
                               sampInterval - ((long)nChannels * 40 + 500));

    } else if (ptype == 'd') {
        DateTime now = RTC.now();
        float hourDecimal = now.hour() + now.minute() / 60.0f + now.second() / 3600.0f;
        currentSP = dailyCycleSP(phaseIdx, hourDecimal);
        for (int i = 0; i < nChannels; i++) {
            holdSP[i]  = currentSP;
            doInput[i] = doVals[i];
            valvePIDs[i]->Compute();
        }
        Ardoxy::scheduleRelays(nChannels, doOutput, relayPins,
                               sampInterval - ((long)nChannels * 40 + 500));

    } else if (ptype == 'c') {
        long phaseMsRem = (long)((long)(phaseEndUnix[phaseIdx + 1] - nowUnix) * 1000L);
        float minRem = phaseMsRem / 60000.0;
        if (minRem < 0.01) minRem = 0.01;

        samplesSinceCalc++;
        rateReCalc = (int)round(60000.0 / sampInterval);
        bool doRecalc = (samplesSinceCalc >= rateReCalc || samplesSinceCalc == 1);
        for (int i = 0; i < nChannels; i++) {
            if (doRecalc)
                seqRateSP[i] = (phaseSetpoints[phaseIdx] - doVals[i]) / minRem;
            seqRateInput[i] = (doVals[i] - doFloatPrev[i]) * 60.0
                              / ((float)sampInterval / 1000.0);
            seqRatePIDs[i]->Compute();
        }
        if (doRecalc) samplesSinceCalc = 0;
        Ardoxy::scheduleRelays(nChannels, doOutput, relayPins,
                               sampInterval - ((long)nChannels * 40 + 500));
    }

    for (int i = 0; i < nChannels; i++) doFloatPrev[i] = doVals[i];

    uint32_t elapsedMs = (nowUnix - expStart.unixtime()) * 1000UL;
    emitData(elapsedMs, doVals, tempVals, currentSP, phaseIdx, ptype);
    writeToSD(doVals, tempVals, currentSP, phaseIdx, ptype);
    writeState();
    lcdUpdate(currentSP, ptype);

    long rem = sampInterval - (long)(millis() - loopStart);
    if (rem > 0) delay(rem);
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
                logFile.close();
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
                // Shift phase boundaries and expStart so pause time is excluded
                for (int i = 0; i <= nPhases; i++) phaseEndUnix[i] += pausedFor;
                setpointEndUnix += pausedFor;
                expStart = DateTime(expStart.unixtime() + pausedFor);
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

        if (strcmp_P(key, PSTR("SETRTC")) == 0) {
            // CMD:SETRTC:<Y>:<M>:<D>:<h>:<m>:<s>
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
        return;
    }

    // ── CFG ──────────────────────────────────────────────────────────────────
    if (strcmp_P(cat, PSTR("CFG")) == 0) {
        if (state == RUNNING || state == PAUSED) {
            Serial.println(F("ACK:ERR:Running")); return;
        }
        char* key = strtok(NULL, ":");
        char* val = strtok(NULL, ":");
        if (key == NULL) return;

        if (strcmp_P(key, PSTR("MODE")) == 0) {
            if (!val) return;
            if      (strcmp_P(val, PSTR("MEASURE")) == 0)  mode = MEASURE;
            else if (strcmp_P(val, PSTR("SETPOINT")) == 0) mode = SETPOINT;
            else if (strcmp_P(val, PSTR("SEQUENCE")) == 0) mode = SEQUENCE;

        } else if (strcmp_P(key, PSTR("NCHANNELS")) == 0) {
            nChannels = constrain(atoi(val), 1, MAX_CHANNELS);

        } else if (strcmp_P(key, PSTR("SENSORS")) == 0) {
            nSensors = constrain(atoi(val), 1, 2);
            if (nSensors == 1) s1Channels = nChannels;  // auto-sync

        } else if (strcmp_P(key, PSTR("S1CHANNELS")) == 0) {
            s1Channels = constrain(atoi(val), 1, MAX_CHANNELS - 1);

        } else if (strcmp_P(key, PSTR("RELAY")) == 0) {
            int ch = atoi(val);
            char* pinStr = strtok(NULL, ":");
            if (ch >= 0 && ch < MAX_CHANNELS && pinStr)
                relayPins[ch] = atoi(pinStr);

        } else if (strcmp_P(key, PSTR("INTERVAL")) == 0) {
            sampInterval = atol(val);

        } else if (strcmp_P(key, PSTR("DURATION")) == 0) {
            experimentDuration = atol(val);

        } else if (strcmp_P(key, PSTR("TANKID")) == 0) {
            // CFG:TANKID:<ch>:<id>
            int ch = atoi(val);
            char* idStr = strtok(NULL, ":");
            if (ch >= 0 && ch < MAX_CHANNELS && idStr)
                strncpy(tankID[ch], idStr, 6);

        } else if (strcmp_P(key, PSTR("SETPOINT")) == 0) {
            DOSetpoint = atof(val);

        } else if (strcmp_P(key, PSTR("KP")) == 0) {
            // CFG:KP:<ch>:<float>
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

        } else if (strcmp_P(key, PSTR("NPHASES")) == 0) {
            nPhases = constrain(atoi(val), 0, MAX_PHASES);

        } else if (strcmp_P(key, PSTR("PHASE")) == 0) {
            // CFG:PHASE:<idx>:<sp>:<dur_d>:<dur_h>:<dur_m>:<type>[:<min_sp>:<max_sp>:<peak_h>]
            int idx = atoi(val);
            if (idx < 0 || idx >= MAX_PHASES) { Serial.println(F("ACK:ERR:Phase idx")); return; }
            char* spStr   = strtok(NULL, ":");
            char* dStr    = strtok(NULL, ":");
            char* hStr    = strtok(NULL, ":");
            char* mStr    = strtok(NULL, ":");
            char* typeStr = strtok(NULL, ":");
            if (!spStr || !dStr || !hStr || !mStr || !typeStr) {
                Serial.println(F("ACK:ERR:Phase fmt")); return;
            }
            phaseSetpoints[idx] = atof(spStr);
            phaseDurSec[idx]    = (uint32_t)(atol(dStr) * 86400L
                                           + atol(hStr) * 3600L
                                           + atol(mStr) * 60L);
            phaseTypes[idx]     = typeStr[0];
            if (typeStr[0] == 'd') {
                char* minStr  = strtok(NULL, ":");
                char* maxStr  = strtok(NULL, ":");
                char* peakStr = strtok(NULL, ":");
                if (minStr && maxStr && peakStr) {
                    phaseMinSP[idx]    = atof(minStr);
                    phaseMaxSP[idx]    = atof(maxStr);
                    phasePeakHour[idx] = atof(peakStr);
                } else {
                    Serial.println(F("ACK:ERR:Daily-cycle params missing")); return;
                }
            }
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
    Serial.begin(19200);
    Serial.println(F("MSG:ArdoxyStandalone ready"));

    // LCD
    lcd.begin(16, 2);
    lcd.setBacklight(WHITE);
    lcd.clear();
    lcd.print(F("Ardoxy Standalone"));

    // I2C (RTC + LCD)
    Wire.begin();

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
        // Serial config session: GUI will send CFG:* then CMD:START
        lcd.clear(); lcd.print(F("Serial config..."));
        Serial.println(F("MSG:Serial config mode"));
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

    if      (mode == MEASURE)  runMeasure();
    else if (mode == SETPOINT) runSetpoint();
    else if (mode == SEQUENCE) runSequence();
}
