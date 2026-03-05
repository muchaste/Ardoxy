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
      CMD:STATUS
    Arduino -> PC:
      ACK:OK
      ACK:ERR:<msg>
      STATUS:<IDLE|CONFIGURED|RUNNING>
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
typedef enum { IDLE, CONFIGURED, RUNNING } State;
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

// One extra PID for change-rate control in sequence mode (channel 0)
double seqRateInput = 0.0, seqRateOutput = 0.0, seqRateSP = 0.0;
PID seqRatePID(&seqRateInput, &seqRateOutput, &seqRateSP, 0, 1, 0, REVERSE);

// ---- serial parsing ---------------------------------------------------------
char   recvBuf[RECV_BUF];
int    recvIdx = 0;

// ---- runtime state ----------------------------------------------------------
unsigned long progStart, progEnd, loopStart;
int     windowSize;

// sequence runtime
int           phaseIdx = 0;
unsigned long phaseMarks[MAX_PHASES + 1];
double        doFloatPrev = 0.0;
int           rateReCalc;
int           samplesSinceCalc;

// ---- helpers ----------------------------------------------------------------

void closeAllValves() {
    for (int i = 0; i < nChannels; i++) {
        digitalWrite(relayPins[i], HIGH);        // HIGH = normally-closed = valve shut
    }
}

void configurePIDs() {
    windowSize = sampInterval / 200;
    for (int i = 0; i < nChannels; i++) {
        holdSP[i] = DOSetpoint;
        valvePIDs[i]->SetTunings(Kp, Ki, Kd);
        valvePIDs[i]->SetOutputLimits(0, windowSize);
        valvePIDs[i]->SetSampleTime(sampInterval);
        valvePIDs[i]->SetMode(AUTOMATIC);
    }
    seqRatePID.SetTunings(0, Ki, 0);
    seqRatePID.SetOutputLimits(0, windowSize);
    seqRatePID.SetSampleTime(sampInterval);
}

// Sort channel indices ascending by doOutput, write to sorted[].
void sortChannelsByOutput(int sorted[]) {
    for (int i = 0; i < nChannels; i++) sorted[i] = i;
    for (int i = 0; i < nChannels - 1; i++) {
        for (int j = i + 1; j < nChannels; j++) {
            if (doOutput[sorted[j]] < doOutput[sorted[i]]) {
                int tmp = sorted[i]; sorted[i] = sorted[j]; sorted[j] = tmp;
            }
        }
    }
}

// Open all valves > 0 simultaneously, close sequentially by time difference.
// Valves whose open time reaches the cap stay open for the full interval.
void scheduleValves() {
    int sorted[MAX_CHANNELS];
    sortChannelsByOutput(sorted);

    long openTimes[MAX_CHANNELS];
    bool fullyOpen[MAX_CHANNELS];
    long measureDur = (long)nChannels * 40 + 500;   // conservative overhead
    long maxOpenTime = sampInterval - measureDur;

    for (int i = 0; i < nChannels; i++) {
        openTimes[i] = (long)(doOutput[sorted[i]]) * 200;
        fullyOpen[i] = (openTimes[i] >= maxOpenTime);
        if (fullyOpen[i]) openTimes[i] = maxOpenTime;
    }

    // Open all that have non-zero time
    for (int i = 0; i < nChannels; i++) {
        if (openTimes[i] > 0) {
            digitalWrite(relayPins[sorted[i]], LOW);
        }
    }

    // Close sequentially (shortest first); fully-open channels are left open
    long elapsed = 0;
    for (int i = 0; i < nChannels; i++) {
        if (openTimes[i] > 0 && !fullyOpen[i]) {
            delay(openTimes[i] - elapsed);
            elapsed = openTimes[i];
            digitalWrite(relayPins[sorted[i]], HIGH);
        }
    }
}

// ---- measurement + output helpers -------------------------------------------

bool measureAll(float* doVals, float* tempVal) {
    int result = 1;
    result &= ardoxy.measureTemp();
    long rawTemp = ardoxy.readoutTemp();
    if (rawTemp == 0 && result == 0) return false;
    *tempVal = rawTemp / 1000.0;

    for (int i = 0; i < nChannels; i++) {
        result &= ardoxy.measureDO(i + 1);
        long rawDO = ardoxy.readoutDO(i + 1);
        doVals[i] = rawDO / 1000.0;
    }
    return result != 0;
}

void emitData(unsigned long ms, float* doVals, float tempVal, float sp, int pidx, char ptype) {
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

void runMeasury() {
    loopStart = millis();
    float doVals[MAX_CHANNELS];
    float tempVal;

    if (!measureAll(doVals, &tempVal)) {
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
        closeAllValves();
        ardoxy.end();
        Serial.println(F("DONE"));
        state = IDLE;
        return;
    }

    loopStart = millis();
    float doVals[MAX_CHANNELS];
    float tempVal;

    if (!measureAll(doVals, &tempVal)) {
        Serial.println(F("MSG:Sensor read error"));
        closeAllValves();
        return;
    }

    for (int i = 0; i < nChannels; i++) {
        doInput[i] = doVals[i];
        valvePIDs[i]->Compute();
    }

    scheduleValves();

    unsigned long elapsed = millis() - progStart;
    emitData(elapsed, doVals, tempVal, DOSetpoint, 0, 's');

    long remaining = sampInterval - (long)(millis() - loopStart);
    if (remaining > 0) delay(remaining);
}

void runSequence() {
    if (phaseIdx >= nPhases) {
        closeAllValves();
        ardoxy.end();
        Serial.println(F("DONE"));
        state = IDLE;
        return;
    }

    // Phase advance
    if (millis() > phaseMarks[phaseIdx + 1]) {
        phaseIdx++;
        if (phaseIdx >= nPhases) {
            closeAllValves();
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
        closeAllValves();
        long remaining = sampInterval - (long)(millis() - loopStart);
        if (remaining > 0) delay(remaining);
        return;
    }

    loopStart = millis();
    float doVals[MAX_CHANNELS];
    float tempVal;

    if (!measureAll(doVals, &tempVal)) {
        Serial.println(F("MSG:Sensor read error"));
        closeAllValves();
        return;
    }

    float currentDO = doVals[0];   // sequence mode uses channel 0

    if (ptype == 'h') {
        holdSP[0] = phaseSetpoints[phaseIdx];
        doInput[0] = currentDO;
        valvePIDs[0]->Compute();
        scheduleValves();
    } else if (ptype == 'c') {
        // Change mode: rate PID on channel 0
        long phaseMsRemaining = (long)(phaseMarks[phaseIdx + 1] - millis());
        float durationMinRemaining = phaseMsRemaining / 60000.0;
        if (durationMinRemaining < 0.01) durationMinRemaining = 0.01;

        samplesSinceCalc++;
        rateReCalc = (int)round(60000.0 / sampInterval);
        if (samplesSinceCalc >= rateReCalc || samplesSinceCalc == 1) {
            seqRateSP = (phaseSetpoints[phaseIdx] - currentDO) / durationMinRemaining;
            samplesSinceCalc = 0;
        }

        seqRateInput = (currentDO - doFloatPrev) * 60.0 / ((float)sampInterval / 1000.0);
        seqRatePID.Compute();
        doOutput[0] = seqRateOutput;
        scheduleValves();
    }

    doFloatPrev = currentDO;

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
            if (state == IDLE)       Serial.println(F("IDLE"));
            else if (state == CONFIGURED) Serial.println(F("CONFIGURED"));
            else                     Serial.println(F("RUNNING"));
            return;
        }

        if (strcmp_P(key, PSTR("STOP")) == 0) {
            if (state == RUNNING) {
                closeAllValves();
                ardoxy.end();
                state = CONFIGURED;   // config stays valid; allow immediate restart
            }
            Serial.println(F("ACK:OK"));
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
            configurePIDs();
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
                holdSP[0] = phaseSetpoints[0];
                seqRateSP = 0.0;
                samplesSinceCalc = 0;
                doFloatPrev = 0.0;
                seqRatePID.SetMode(AUTOMATIC);
                valvePIDs[0]->SetMode(AUTOMATIC);
            }

            state = RUNNING;
            Serial.println(F("ACK:OK"));
            Serial.println(F("MSG:Running"));
            return;
        }
        return;
    }

    if (strcmp_P(cat, PSTR("CFG")) == 0) {
        if (state == RUNNING) {
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
        runMeasury();
    } else if (mode == SETPOINT) {
        runSetpoint();
    } else if (mode == SEQUENCE) {
        runSequence();
    }
}
