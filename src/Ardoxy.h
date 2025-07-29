/*
  Ardoxy.h - Library for communicating with a FireSting Oxygen sensor.
  Created by Stefan Mucha, October 23, 2021.
*/

#ifndef Ardoxy_h
#define Ardoxy_h

#include "Arduino.h"
#include <SoftwareSerial.h>

#define numChars 60

class Ardoxy
{
  public:
    Ardoxy( HardwareSerial& device) {hwStream = &device;}
    Ardoxy( SoftwareSerial& device) {swStream = &device;}
    void begin();
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
    void clearSerialBuffer();
    void setActiveStream();
    bool waitForResponse(unsigned long timeoutMs = 1000);
};

#endif
