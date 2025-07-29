/*
  Ardoxy.h - Library for communicating with a FireSting Oxygen sensor.
  Created by Stefan Mucha, October 23, 2021.
*/

#include <Arduino.h>
#include <Ardoxy.h>
#include <SoftwareSerial.h>

// Begin function to establish connection and set baud rate to 19200 if necessary
void Ardoxy::begin()
{
    // Helper lambda to establish a connection
    auto establishConnection = [this](Stream* serialStream, long baudRate) -> bool {
        // Cast to specific serial type and initialize
        if (serialStream == hwStream) {
            static_cast<HardwareSerial*>(serialStream)->begin(baudRate);
        } else if (serialStream == swStream) {
            static_cast<SoftwareSerial*>(serialStream)->begin(baudRate);
        } else {
            return false; // Unknown stream type
        }

        delay(3000);

        // Clear the serial buffer
        while (serialStream->available() > 0) {
            char t = serialStream->read();
            delay(2);
        }

        // Send a test command
        serialStream->write("MSR 1\r");
        delay(300);

        if (serialStream->available()) {
            while (serialStream->available() > 0) {
                char t = serialStream->read();
                delay(2);
            }
            return true;
        }
        return false;
    };

    // Attempt connection at 19200
    Stream* activeStream = hwStream ? (Stream*)hwStream : (Stream*)swStream;

    if (establishConnection(activeStream, 19200)) {
        Serial.println("Connection Established at Baudrate 19200");
    } else if (establishConnection(activeStream, 115200)) {
        Serial.println("Connection Established at Baudrate 115200");

        // Send command to change baud rate to 19200
        activeStream->write("#BAUD 19200\r");
        delay(300);

        // Reinitialize at 19200
        if (!establishConnection(activeStream, 19200)) {
            Serial.println("Failed to switch to Baudrate 19200");
            return;
        }
        Serial.println("Baudrate successfully switched to 19200");
    } else {
        Serial.println("Failed to establish connection at any baudrate");
        return;
    }

    // Retrieve firmware version
    ver = getVer();
    Serial.print("Firmware Version: ");
    Serial.println(ver);
}

// End function
void Ardoxy::end()
{
  if (hwStream)
  {
    hwStream->end();
  }
  else
  {
    swStream->end();
  }
}

// Function to read the firmware version from the device
int Ardoxy::getVer()
{
  setActiveStream();
  clearSerialBuffer();

  // Send command to FireSting
  stream->write("#VERS\r");
  stream->flush();
  delay(170); // Let FireSting finish before reading incoming serial data

  if(!stream->available()){ 
    return 0; // No incoming data, connection problem
  }

  bool received = false;
  while (stream->available() > 0 && !received && ndx <= numChars-1) {
    delay(2);
    rc = stream->read();
    if (rc != endMarker && ndx < numChars-1) {
      receivedChars[ndx] = rc;
      ndx++;
    }
    else {
      receivedChars[ndx] = '\0';  // terminate the string
      ndx = 0;
      received = true;
      if(strncmp("#VERS", receivedChars, (strlen("#VERS")-1)) == 0){ 
        // get the 4th value of the return string (values separated by spaces)
        char *verChar = strtok(receivedChars, " ");
        for(int i = 0; i < 3; i++){
          verChar=strtok(NULL, " ");
        }
        return atol(verChar);
      }
      else{
        return 9; // Mismatch
      }
    }
  }
  return 0;
}

// Core communication helper methods
void Ardoxy::clearSerialBuffer()
{
  while(stream->available() > 0){
    char t = stream->read();
    delay(2);
  }
}

void Ardoxy::setActiveStream()
{
  stream = !hwStream? (Stream*)swStream : hwStream;
}

bool Ardoxy::waitForResponse(unsigned long timeoutMs)
{
  unsigned long startTime = millis();
  while (!stream->available()) {
    if (millis() - startTime > timeoutMs) {
      return false; // Timeout
    }
    delay(delayPerCheck);
  }
  return true;
}

int Ardoxy::sendCommandForEcho(const char* command)
{
  setActiveStream();
  clearSerialBuffer();
  
  // Send command to FireSting
  stream->write(command);
  stream->flush();
  
  if (!waitForResponse()) {
    return 0; // Timeout error
  }
  
  bool received = false;
  while (stream->available() > 0 && !received && ndx <= numChars-1) {
    delay(2);
    rc = stream->read();
    if (rc != endMarker && ndx < numChars-1) {
      receivedChars[ndx] = rc;
      ndx++;
    }
    else {
      receivedChars[ndx] = '\0';  // terminate the string
      ndx = 0;
      received = true;
      if(strncmp(command, receivedChars, (strlen(command)-1)) == 0){ 
        return 1; // Echo matches command
      }
      else{
        return 9; // Mismatch
      }
    }
  }
  return 9; // Should not reach here
}

long Ardoxy::sendCommandForValue(const char* command)
{
  setActiveStream();
  clearSerialBuffer();
  
  stream->write(command);
  stream->flush();
  
  if (!waitForResponse()) {
    return 0; // Timeout error
  }
  
  bool received = false;
  while (stream->available() > 0 && !received && ndx <= numChars-1) {
    delay(2);
    rc = stream->read();
    if (rc != endMarker && ndx < numChars-1) {
      receivedChars[ndx] = rc;
      ndx++;
    }
    else {
      receivedChars[ndx] = '\0';
      ndx = 0;
      if(strncmp(command, receivedChars, (strlen(command)-1)) == 0){
        received = true;
        char* separator = strrchr(receivedChars, ' ');
        return atol(separator);
      }
      else{
        return 0; // Mismatch
      }
    }
  }
  return 0;
}

int Ardoxy::setTempComp(int chan)
{
  sprintf(measCommand, "WRT %d 0 0 -300000\r", chan);
  int result = sendCommandForEcho(measCommand);
  
  if (result == 1) {
    Serial.println("Compensation for sample temperature activated.");
  }
  
  return result;
}


// Measure function: send measurement command to firesting via Serial communication
// Returns:
// 1 when echo matches command
// 0 when there is no echo (connection problem)
// 9 when there is a mismatch (usually due to timing or connection issues)
int Ardoxy::measure(char command[])
{
  return sendCommandForEcho(command);
}

// Measure Sequence function: same as measure function but with pre-set measurement command
int Ardoxy::measureSeq(int chan)
{
  if(ver >= 400){
    sprintf(measCommand, "MEA %d 47\r", chan);
  } else {
    sprintf(measCommand, "SEQ %d\r", chan);
  }
  
  return sendCommandForEcho(measCommand);
}

// Measure DO function: same as measure function but with pre-set measurement command
int Ardoxy::measureDO(int chan)
{
  if(ver >= 400){
    sprintf(measCommand, "MEA %d 1\r", chan);
  } else {
    sprintf(measCommand, "MSR %d\r", chan);
  }
  
  return sendCommandForEcho(measCommand);
}

// Measure Temperature function: same as measure function but with pre-set measurement command
int Ardoxy::measureTemp()
{
  int chan = 1;
  if(ver >= 400){
    sprintf(measCommand, "MEA %d 2\r", chan);
  } else {
    sprintf(measCommand, "TMP %d\r", chan);
  }
  
  return sendCommandForEcho(measCommand);
}

// Readout values from Firesting memory
// Returns:
// numerical value (air saturation or temperature) - refer to Firesting Protocol
// 0 if there is a communication mismatch
long Ardoxy::readout(char command[])
{
  return sendCommandForValue(command);
}

// readoutDO - Similar to readout function but with pre-set DO-readout command
long Ardoxy::readoutDO(int chan)
{
  sprintf(measCommand, "RMR %d 3 4 1\r", chan);
  return sendCommandForValue(measCommand);
}

// readoutTemp - Similar to readout function but with pre-set temperature-readout command
long Ardoxy::readoutTemp()
{
  sprintf(measCommand, "RMR 1 3 5 1\r");
  return sendCommandForValue(measCommand);
}

// Calculate duration in days between two given dates
// Returns:
// integer of number of days (if start and end date are the same, dur = 1)
int Ardoxy::calcDays(int startDay, int startMonth, int startYear, int endDay, int endMonth, int endYear) {
  const int monthDays[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  const int leapMonthDays[12] = {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  int dayDuration = 0;
  // option 1: it is the day of acclimation start
  if (endDay == startDay && endMonth == startMonth && endYear == startYear){
    dayDuration = 1;
  } else if (endDay > startDay && endMonth == startMonth && endYear == startYear){
    // option 2: it is past the start date and the same month
    dayDuration = int(endDay - startDay + 1);
  } else if (endMonth > startMonth && endYear == startYear){
    // option 3: it is past the start month but the same year (-> the day can be different)
    // sum up the days of the months between starting month and current month
    if((endYear > 1970) && !(endYear%4) && ((endYear%100) || !(endYear%400))){
      for (int j = startMonth; j < int(endMonth); j++){
        dayDuration += leapMonthDays[j-1];
      }
    } else {
      for (int j = startMonth; j < int(endMonth); j++){
        dayDuration += monthDays[j-1];
      }
    }
    // subtract the start date and add the current day
    dayDuration -= startDay;
    dayDuration += endDay;
  } else if(endYear > startYear){
    // option 4: it is past the starting year
    if((startYear > 1970) && !(startYear%4) && ((startYear%100) || !(startYear%400))){
      // option 4.1: the starting year was a leap year
      for (int j = startMonth; j <= 12; j++){
        dayDuration += leapMonthDays[j-1];
      }
      for(int j = 1; j <= int(endMonth); j++){        // crucial here: j starts at 1 and has to run until it is less or even the current month (for the case of january)
        dayDuration += monthDays[j-1];
      }
      // subtract start day and the rest days of the current month (minus 1)
      dayDuration -= (startDay + monthDays[endMonth-1] - endDay - 1);
    } else if((endYear > 1970) && !(endYear%4) && ((endYear%100) || !(endYear%400))){
      // option 4.2: the current year is a leap year
      for (int j = startMonth; j <= 12; j++){
        dayDuration += monthDays[j-1];
      }
      for(int j = 1; j <= int(endMonth); j++){
        dayDuration += leapMonthDays[j-1];
      }
      // subtract start day and the rest days of the current month (minus 1)
      dayDuration -= (startDay + leapMonthDays[endMonth-1] - endDay - 1);
      Serial.println(dayDuration);
    } else {
      // option 4.3: no leap years
      for (int j = startMonth; j <= 12; j++){
        dayDuration += monthDays[j-1];
      }
      for(int j = 1; j <= int(endMonth); j++){
        dayDuration += monthDays[j-1];
      }
      // subtract start day and the rest days of the current month (minus 1)
      dayDuration -= (startDay + monthDays[endMonth-1] - endDay - 1);
    }
  }
  return dayDuration;
}
