/*
  ardoxy_os.ino
  Ardoxy Operating System Sketch — upload once, configure via ardoxy_gui.py

  Protocol (USB serial, 19200 baud, newline-terminated):
    PC -> Arduino:
      CFG:MODE:<MEASURE|SETPOINT|SEQUENCE>
      CFG:NCHANNELS:<1-4>
      CFG:RELAY:<ch>:<pin>          ch = 0-based index
      CFG:INTERVAL:<ms>
      CFG:DURATION:<minutes>
      CFG:SETPOINT:<float>          SETPOINT mode
      CFG:KP:<float>
      CFG:KI:<float>
      CFG:KD:<float>
      CFG:NPHASES:<n>               SEQUENCE mode
      CFG:PHASE:<idx>:<sp>:<min>:<t>  t = c|h|p
      CMD:START
      CMD:STOP
      CMD:PAUSE
      CMD:RESUME
      CMD:STATUS
    Arduino -> PC:
      ACK:OK
      ACK:ERR:<msg>
      STATUS:<IDLE|CONFIGURED|RUNNING|PAUSED>
      DATA:<ms>,<do_ch1[,do_ch2...]>,<temp>,<output[,output...]>,<sp>,<phase>,<ptype>
      MSG:<text>
      DONE
*/

#include <SoftwareSerial.h>
#include <PID_v1.h>
#include <Ardoxy.h>

// ---- hardware ---------------------------------------------------------------
#define RX_PIN 8
#define TX_PIN 9
#define MAX_CHANNELS 4
#define MAX_PHASES   10
#define RECV_BUF     96

SoftwareSerial firestingSerial(RX_PIN, TX_PIN);
Ardoxy ardoxy(firestingSerial);

// ---- state ------------------------------------------------------------------
typedef enum { IDLE, CONFIGURED, RUNNING, PAUSED } State;
typedef enum { MEASURE, SETPOINT, SEQUENCE } Mode;

State state = IDLE;
Mode  mode  = MEASURE;

// ---- config -----------------------------------------------------------------
int   nChannels = 1;
int   relayPins[MAX_CHANNELS];
long  sampInterval    = 2000;       // ms
long  experimentDuration = 60;      // minutes
float DOSetpoint = 30.0;
float Kp = 10.0, Ki = 1.0, Kd = 0.0;

// sequence
int   nPhases = 0;
float phaseSetpoints[MAX_PHASES];
long  phaseDurations[MAX_PHASES];   // ms (converted from minutes at start)
char  phaseTypes[MAX_PHASES];       // 'c', 'h', 'p'

// ---- PID --------------------------------------------------------------------
// Four PIDs for setpoint / hold-phase (one per channel)
double doInput[MAX_CHANNELS]  = {0,0,0,0};
double doOutput[MAX_CHANNELS] = {0,0,0,0};
double holdSP[MAX_CHANNELS]   = {30,30,30,30};

PID valvePID0(&doInput[0], &doOutput[0], &holdSP[0], 10, 1, 0, REVERSE);
PID valvePID1(&doInput[1], &doOutput[1], &holdSP[1], 10, 1, 0, REVERSE);
PID valvePID2(&doInput[2], &doOutput[2], &holdSP[2], 10, 1, 0, REVERSE);
PID valvePID3(&doInput[3], &doOutput[3], &holdSP[3], 10, 1, 0, REVERSE);
PID* valvePIDs[MAX_CHANNELS] = {&valvePID0, &valvePID1, &valvePID2, &valvePID3};

// Per-channel rate PIDs for change phases in sequence mode
double seqRateInput[MAX_CHANNELS] = {0,0,0,0};
double seqRateSP[MAX_CHANNELS]    = {0,0,0,0};
PID seqRatePID0(&seqRateInput[0], &doOutput[0], &seqRateSP[0], 0, 1, 0, REVERSE);
PID seqRatePID1(&seqRateInput[1], &doOutput[1], &seqRateSP[1], 0, 1, 0, REVERSE);
PID seqRatePID2(&seqRateInput[2], &doOutput[2], &seqRateSP[2], 0, 1, 0, REVERSE);
PID seqRatePID3(&seqRateInput[3], &doOutput[3], &seqRateSP[3], 0, 1, 0, REVERSE);
PID* seqRatePIDs[MAX_CHANNELS] = {&seqRatePID0, &seqRatePID1, &seqRatePID2, &seqRatePID3};

// ---- serial parsing ---------------------------------------------------------
char   recvBuf[RECV_BUF];
int    recvIdx = 0;

// ---- runtime state ----------------------------------------------------------
unsigned long progStart, progEnd, loopStart;
unsigned long pauseStart = 0;
int     windowSize;

// sequence runtime
int           phaseIdx = 0;
unsigned long phaseMarks[MAX_PHASES + 1];
double        doFloatPrev[MAX_CHANNELS] = {0,0,0,0};
int           rateReCalc;
int           samplesSinceCalc;


// ---- measurement + output helpers -------------------------------------------

void emitData(unsigned long ms, double* doVals, double tempVal, float sp, int pidx, char ptype) {
    Serial.print(F("DATA:"));
    Serial.print(ms);
    for (int i = 0; i < nChannels; i++) {
        Serial.print(',');
        Serial.print(doVals[i], 3);
    }
    Serial.print(',');
    Serial.print(tempVal, 3);
    Serial.print(',');
    for (int i = 0; i < nChannels; i++) {
        Serial.print((long)(doOutput[i]) * 200);  // ms open time
        if (i < nChannels - 1) Serial.print(',');
    }
    Serial.print(',');
    Serial.print(sp, 2);
    Serial.print(',');
    Serial.print(pidx);
    Serial.print(',');
    Serial.println(ptype);
}

// ---- run modes --------------------------------------------------------------

void runMeasure() {
    loopStart = millis();
    double doVals[MAX_CHANNELS];
    double tempVal;

    if (!ardoxy.measureAll(nChannels, doVals, &tempVal)) {
        Serial.println(F("MSG:Sensor read error"));
        return;
    }

    unsigned long elapsed = millis() - progStart;
    emitData(elapsed, doVals, tempVal, 0.0, 0, 'm');

    long remaining = sampInterval - (long)(millis() - loopStart);
    if (remaining > 0) delay(remaining);
}

void runSetpoint() {
    if (millis() > progEnd) {
        Ardoxy::closeRelays(nChannels, relayPins);
        ardoxy.end();
        Serial.println(F("DONE"));
        state = IDLE;
        return;
    }

    loopStart = millis();
    double doVals[MAX_CHANNELS];
    double tempVal;

    if (!ardoxy.measureAll(nChannels, doVals, &tempVal)) {
        Serial.println(F("MSG:Sensor read error"));
        Ardoxy::closeRelays(nChannels, relayPins);
        return;
    }

    for (int i = 0; i < nChannels; i++) {
        doInput[i] = doVals[i];
        valvePIDs[i]->Compute();
    }

    Ardoxy::scheduleRelays(nChannels, doOutput, relayPins, sampInterval - ((long)nChannels * 40 + 500));

    unsigned long elapsed = millis() - progStart;
    emitData(elapsed, doVals, tempVal, DOSetpoint, 0, 's');

    long remaining = sampInterval - (long)(millis() - loopStart);
    if (remaining > 0) delay(remaining);
}

void runSequence() {
    if (phaseIdx >= nPhases) {
        Ardoxy::closeRelays(nChannels, relayPins);
        ardoxy.end();
        Serial.println(F("DONE"));
        state = IDLE;
        return;
    }

    // Phase advance
    if (millis() > phaseMarks[phaseIdx + 1]) {
        phaseIdx++;
        if (phaseIdx >= nPhases) {
            Ardoxy::closeRelays(nChannels, relayPins);
            ardoxy.end();
            Serial.println(F("DONE"));
            state = IDLE;
            return;
        }
        if (phaseTypes[phaseIdx] == 'c') {
            samplesSinceCalc = 0;
        }
        Serial.print(F("MSG:Phase "));
        Serial.println(phaseIdx);
    }

    char ptype = phaseTypes[phaseIdx];

    if (ptype == 'p') {
        Ardoxy::closeRelays(nChannels, relayPins);
        long remaining = sampInterval - (long)(millis() - loopStart);
        if (remaining > 0) delay(remaining);
        return;
    }

    loopStart = millis();
    double doVals[MAX_CHANNELS];
    double tempVal;

    if (!ardoxy.measureAll(nChannels, doVals, &tempVal)) {
        Serial.println(F("MSG:Sensor read error"));
        Ardoxy::closeRelays(nChannels, relayPins);
        return;
    }

    if (ptype == 'h') {
        for (int i = 0; i < nChannels; i++) {
            holdSP[i] = phaseSetpoints[phaseIdx];
            doInput[i] = doVals[i];
            valvePIDs[i]->Compute();
        }
        Ardoxy::scheduleRelays(nChannels, doOutput, relayPins, sampInterval - ((long)nChannels * 40 + 500));
    } else if (ptype == 'c') {
        // Change mode: per-channel rate PIDs
        long phaseMsRemaining = (long)(phaseMarks[phaseIdx + 1] - millis());
        float durationMinRemaining = phaseMsRemaining / 60000.0;
        if (durationMinRemaining < 0.01) durationMinRemaining = 0.01;

        samplesSinceCalc++;
        rateReCalc = (int)round(60000.0 / sampInterval);
        bool doRateRecalc = (samplesSinceCalc >= rateReCalc || samplesSinceCalc == 1);
        for (int i = 0; i < nChannels; i++) {
            if (doRateRecalc) {
                seqRateSP[i] = (phaseSetpoints[phaseIdx] - doVals[i]) / durationMinRemaining;
            }
            seqRateInput[i] = (doVals[i] - doFloatPrev[i]) * 60.0 / ((float)sampInterval / 1000.0);
            seqRatePIDs[i]->Compute();
        }
        if (doRateRecalc) samplesSinceCalc = 0;
        Ardoxy::scheduleRelays(nChannels, doOutput, relayPins, sampInterval - ((long)nChannels * 40 + 500));
    }

    for (int i = 0; i < nChannels; i++) {
        doFloatPrev[i] = doVals[i];
    }

    unsigned long elapsed = millis() - progStart;
    emitData(elapsed, doVals, tempVal, phaseSetpoints[phaseIdx], phaseIdx, ptype);

    long remaining = sampInterval - (long)(millis() - loopStart);
    if (remaining > 0) delay(remaining);
}

// ---- serial command parser --------------------------------------------------

void processCommand(char* buf) {
    // Tokenise: CATEGORY:KEY:VALUE or CATEGORY:KEY
    char* cat = strtok(buf, ":");
    if (cat == NULL) return;

    if (strcmp_P(cat, PSTR("CMD")) == 0) {
        char* key = strtok(NULL, ":");
        if (key == NULL) return;

        if (strcmp_P(key, PSTR("STATUS")) == 0) {
            Serial.print(F("STATUS:"));
            if (state == IDLE)            Serial.println(F("IDLE"));
            else if (state == CONFIGURED) Serial.println(F("CONFIGURED"));
            else if (state == PAUSED)     Serial.println(F("PAUSED"));
            else                          Serial.println(F("RUNNING"));
            return;
        }

        if (strcmp_P(key, PSTR("STOP")) == 0) {
            if (state == RUNNING || state == PAUSED) {
                Ardoxy::closeRelays(nChannels, relayPins);
                ardoxy.end();
                state = CONFIGURED;   // config stays valid; allow immediate restart
            }
            Serial.println(F("ACK:OK"));
            return;
        }

        if (strcmp_P(key, PSTR("PAUSE")) == 0) {
            if (state == RUNNING) {
                Ardoxy::closeRelays(nChannels, relayPins);
                pauseStart = millis();
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
                unsigned long pausedFor = millis() - pauseStart;
                progStart += pausedFor;
                progEnd   += pausedFor;
                for (int i = 0; i <= nPhases; i++) {
                    phaseMarks[i] += pausedFor;
                }
                state = RUNNING;
                Serial.println(F("ACK:OK"));
                Serial.println(F("MSG:Running"));
            } else {
                Serial.println(F("ACK:ERR:Not paused"));
            }
            return;
        }

        if (strcmp_P(key, PSTR("START")) == 0) {
            if (state == IDLE) {
                Serial.println(F("ACK:ERR:Not configured"));
                return;
            }
            // Setup relay pins
            for (int i = 0; i < nChannels; i++) {
                pinMode(relayPins[i], OUTPUT);
                digitalWrite(relayPins[i], HIGH);
            }
            windowSize = sampInterval / 200;
            for (int i = 0; i < nChannels; i++) {
                holdSP[i] = DOSetpoint;
                Ardoxy::configurePID(*valvePIDs[i], Kp, Ki, Kd, sampInterval, windowSize);
            }
            for (int i = 0; i < nChannels; i++) {
                Ardoxy::configurePID(*seqRatePIDs[i], 0, Ki, 0, sampInterval, windowSize);
                seqRatePIDs[i]->SetMode(MANUAL);
            }
            ardoxy.begin();

            progStart = millis();

            if (mode == MEASURE) {
                progEnd = 0;  // runs indefinitely until STOP
            } else if (mode == SETPOINT) {
                progEnd = progStart + experimentDuration * 60000UL;
            } else if (mode == SEQUENCE) {
                phaseIdx = 0;
                phaseMarks[0] = progStart;
                for (int i = 0; i < nPhases; i++) {
                    phaseMarks[i + 1] = phaseMarks[i] + phaseDurations[i];
                }
                for (int i = 0; i < nChannels; i++) {
                    holdSP[i] = phaseSetpoints[0];
                    seqRateSP[i] = 0.0;
                    doFloatPrev[i] = 0.0;
                    seqRatePIDs[i]->SetMode(AUTOMATIC);
                }
                samplesSinceCalc = 0;
            }

            state = RUNNING;
            Serial.println(F("ACK:OK"));
            Serial.println(F("MSG:Running"));
            return;
        }
        return;
    }

    if (strcmp_P(cat, PSTR("CFG")) == 0) {
        if (state == RUNNING || state == PAUSED) {
            Serial.println(F("ACK:ERR:Running"));
            return;
        }
        char* key = strtok(NULL, ":");
        char* val = strtok(NULL, ":");
        if (key == NULL) return;

        if (strcmp_P(key, PSTR("MODE")) == 0) {
            if (val == NULL) return;
            if (strcmp_P(val, PSTR("MEASURE")) == 0)  mode = MEASURE;
            else if (strcmp_P(val, PSTR("SETPOINT")) == 0) mode = SETPOINT;
            else if (strcmp_P(val, PSTR("SEQUENCE")) == 0) mode = SEQUENCE;
        } else if (strcmp_P(key, PSTR("NCHANNELS")) == 0) {
            nChannels = constrain(atoi(val), 1, MAX_CHANNELS);
        } else if (strcmp_P(key, PSTR("RELAY")) == 0) {
            // CFG:RELAY:<ch>:<pin>
            int ch  = atoi(val);
            char* pinStr = strtok(NULL, ":");
            if (ch >= 0 && ch < MAX_CHANNELS && pinStr != NULL) {
                relayPins[ch] = atoi(pinStr);
            }
        } else if (strcmp_P(key, PSTR("INTERVAL")) == 0) {
            sampInterval = atol(val);
        } else if (strcmp_P(key, PSTR("DURATION")) == 0) {
            experimentDuration = atol(val);
        } else if (strcmp_P(key, PSTR("SETPOINT")) == 0) {
            DOSetpoint = atof(val);
        } else if (strcmp_P(key, PSTR("KP")) == 0) {
            Kp = atof(val);
        } else if (strcmp_P(key, PSTR("KI")) == 0) {
            Ki = atof(val);
        } else if (strcmp_P(key, PSTR("KD")) == 0) {
            Kd = atof(val);
        } else if (strcmp_P(key, PSTR("NPHASES")) == 0) {
            nPhases = constrain(atoi(val), 0, MAX_PHASES);
        } else if (strcmp_P(key, PSTR("PHASE")) == 0) {
            // CFG:PHASE:<idx>:<setpoint>:<duration_min>:<type>
            int idx = atoi(val);
            if (idx < 0 || idx >= MAX_PHASES) { Serial.println(F("ACK:ERR:Phase idx")); return; }
            char* spStr   = strtok(NULL, ":");
            char* durStr  = strtok(NULL, ":");
            char* typeStr = strtok(NULL, ":");
            if (spStr && durStr && typeStr) {
                phaseSetpoints[idx] = atof(spStr);
                phaseDurations[idx] = (long)(atof(durStr) * 60000.0);
                phaseTypes[idx]     = typeStr[0];
            } else {
                Serial.println(F("ACK:ERR:Phase fmt"));
                return;
            }
        }

        state = CONFIGURED;
        Serial.println(F("ACK:OK"));
        return;
    }
}

void readSerial() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (recvIdx > 0) {
                recvBuf[recvIdx] = '\0';
                processCommand(recvBuf);
                recvIdx = 0;
            }
        } else {
            if (recvIdx < RECV_BUF - 1) {
                recvBuf[recvIdx++] = c;
            }
        }
    }
}

// ---- setup / loop -----------------------------------------------------------

void setup() {
    Serial.begin(19200);
    Serial.println(F("MSG:ArdoxyOS ready"));
}

void loop() {
    readSerial();

    if (state != RUNNING) return;

    if (mode == MEASURE) {
        runMeasure();
    } else if (mode == SETPOINT) {
        runSetpoint();
    } else if (mode == SEQUENCE) {
        runSequence();
    }
}
