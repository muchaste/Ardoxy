/*
  Ardoxy.h - Library for communicating with a FireSting Oxygen sensor.
  Created by Stefan Mucha, October 23, 2021.
  Cite: Mucha S (2025) J Exp Biol 228(1):jeb249207. https://doi.org/10.1242/jeb.249207
*/

#ifndef Ardoxy_h
#define Ardoxy_h

#include "Arduino.h"
#include <SoftwareSerial.h>

class PID;  // forward declaration — allows configurePID without pulling in PID_v1.h

#define numChars 60
#define ARDOXY_MAX_CHANNELS 8

class Ardoxy
{
  public:
    Ardoxy( HardwareSerial& device) {hwStream = &device;}
    Ardoxy( SoftwareSerial& device) {swStream = &device;}
    bool begin();
    void end();
    int getVer();
    int setTempComp(int channel);
    int measure(char command[]);
    int measureSeq(int chan);
    int measureDO(int chan);
    int measureTemp();
    long readout(char command[]);
    long readoutDO(int chan);
    long readoutTemp();
    static int calcDays(int startDay, int startMonth, int startYear, int endDay, int endMonth, int endYear);
    int measureAll(int nChannels, double doVals[], double* tempVal);
    static void scheduleRelays(int nChannels, double outputs[], const int relayPins[], unsigned long maxOpenMs);
    static void closeRelays(int nChannels, const int relayPins[]);
    static void configurePID(PID& pid, double kp, double ki, double kd, unsigned long sampInterval, int outputLimit);

  private:
    HardwareSerial* hwStream;
    SoftwareSerial* swStream;
    Stream* stream;
    int ver;
    const int delayPerCheck = 2;
    int ndx = 0;                                                            // index for storing in the array
    char receivedChars[numChars];                                           // Array to hold incoming data
    char endMarker = '\r';                                                  // declare the character that marks the end of a serial transmission
    char rc;                                                                // temporary variable to hold the last received character
    char measCommand[20];                                                   // Buffer for measurement command
    
    // Core communication methods
    enum class ResponseType { ECHO_ONLY, VALUE_EXTRACTION };
    int sendCommandForEcho(const char* command);
    long sendCommandForValue(const char* command);
    long sendCommandForTokenValue(const char* command, int tokenIndex, unsigned long delayMs = 0);
    void clearSerialBuffer();
    void setActiveStream();
    bool waitForResponse(unsigned long timeoutMs = 1000);
    bool establishConnection(long baudRate);
    bool testConnection();
};

#endif
