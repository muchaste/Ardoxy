/*
  Ardoxy.h - Library for communicating with a FireSting Oxygen sensor.
  Created by Stefan Mucha, October 23, 2021.
*/

#include <Arduino.h>
#include <Ardoxy.h>
#include <SoftwareSerial.h>


void Ardoxy::begin()
{
  if (hwStream)
  {
    hwStream->begin(19200);
    delay(3000);
    while(hwStream->available()>0){
      char t = hwStream->read();
      delay(2);
    }
    hwStream->write("MSR 1\r");
    delay(300);
    if(hwStream->available()){
      Serial.println("Hardware Serial Connection Established, Baudrate 19200");
      while(hwStream->available()>0){
        char t = hwStream->read();
        delay(2);
      }
      ver = getVer();
      Serial.print("Firmware Version: ");
      Serial.println(ver);
    } else {
      hwStream->begin(115200);
      delay(3000);
      while(hwStream->available()>0){
        char t = hwStream->read();
        delay(2);
      }
      hwStream->write("MSR 1\r");
      delay(300);
      if(hwStream->available()){
        Serial.println("Hardware Serial Connection Established, Baudrate 115200");
        while(hwStream->available()>0){
          char t = hwStream->read();
          delay(2);
        }
        ver = getVer();
        Serial.print("Firmware Version: ");
        Serial.println(ver);
      } else {
        Serial.println("Couldn't establish connection");
      }
    }
  }
  else
  {
    swStream->begin(19200);
    delay(3000);
    while(swStream->available()>0){
      char t = swStream->read();
      delay(2);
    }
    swStream->write("MSR 1\r");
    delay(300);
    if(swStream->available()){
      Serial.println("Software Serial Connection Established, Baudrate 19200");
      while(swStream->available()>0){
        char t = swStream->read();
        delay(2);
      }
      ver = getVer();
      Serial.print("Firmware Version: ");
      Serial.println(ver);
    } else {
      swStream->begin(115200);
      delay(3000);
      while(swStream->available()>0){
        char t = swStream->read();
        delay(2);
      }
      swStream->write("MSR 1\r");
      delay(300);
      if(swStream->available()){
        Serial.println("Software Serial Connection Established, Baudrate 115200");
        while(swStream->available()>0){
          char t = swStream->read();
          delay(2);
        }
        ver = getVer();
        Serial.print("Firmware Version: ");
        Serial.println(ver);
      } else {
        Serial.println("Couldn't establish connection");
      }
    }
  }
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
  int result;
  // Set source Stream
  stream = !hwStream? (Stream*)swStream : hwStream;

  // Empty Serial buffer
  while(stream->available() > 0){
    char t = stream->read();
    delay(2);
  }

  // Send command to FireSting
  stream->write("#VERS\r");
  stream->flush();
  bool received = false;      // Switch to continue reading incoming data until end marker was received
  delay(170);         // Let Firesting finish measurement before reading incoming serial data

  if(!stream->available()){ // If there is no incoming data, there is a connection problem
    result = 0;
  }
  else{
    while (stream->available() > 0 && received == false && ndx <= numChars-1) {
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
        if(strncmp("#VERS", receivedChars, (strlen("#VERS")-1)) == 0){ // Compare command and received string (FS echoes the command)
          // get the 4th value of the return string (values separated by spaces)
          char *verChar = strtok(receivedChars, " ");
          for(int i = 0; i < 3; i++){
            verChar=strtok(NULL, " ");
          }
          result = atol(verChar);                         // parse the character to integers - the air saturation values are returned as [% air saturation x 1000], temperature as [°C x 1000]
        }
        else{
          result = 9;             // return 9 if there is a mismatch
        }
      }
    }
  }
  return result;
}

// Measure function: send measurement command to firesting via Serial communication
// Returns:
// 1 when echo matches command
// 0 when there is no echo (connection problem)
// 9 when there is a mismatch (usually due to timing or connection issues)
int Ardoxy::measure(char command[])//, int serialDelay=300)
{
  int result;
  unsigned long startTime; // Used for timeout

  // Set source Stream
  stream = !hwStream? (Stream*)swStream : hwStream;

  // Empty Serial buffer
  while(stream->available() > 0){
    char t = stream->read();
    delay(2);
  }

  // Send command to FireSting
  stream->write(command);
  stream->flush();
  bool received = false;      // Switch to continue reading incoming data until end marker was received

  // Wait for data with timeout mechanism
  startTime = millis();
  while (!stream->available()) {
      if (millis() - startTime > 1000) {
          result = 0; // Indicate a timeout error
          return result;
      }
      delay(delayPerCheck);
  }

  while (stream->available() > 0 && received == false && ndx <= numChars-1) {
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
      if(strncmp(command, receivedChars, (strlen(command)-1)) == 0){ // Compare command and received string (FS echoes the command)
        result = 1;
      }
      else{
        result = 9;             // return 9 if there is a mismatch
      }
    }
  }
  return result;
}

// Measure Sequence function: same as measure function but with pre-set measurement command
int Ardoxy::measureSeq(int chan)//, int serialDelay=700)
{
  int result;
  unsigned long startTime; // Used for timeout

  // Paste Channel in measurement command
  if(ver >= 400){
    sprintf(measCommand, "MEA %d 47\r", chan);           // insert channel in measurement command
  } else if (ver < 400) {
    sprintf(measCommand, "SEQ %d\r", chan);           // insert channel in measurement command
  }

  // Set source Stream
  stream = !hwStream? (Stream*)swStream : hwStream;

  // Empty Serial buffer
  while(stream->available() > 0){
    char t = stream->read();
    delay(2);
  }

  // Send command to FireSting
  stream->write(measCommand);
  stream->flush();
  bool received = false;      // Switch to continue reading incoming data until end marker was received

  // Wait for data with timeout mechanism
  startTime = millis();
  while (!stream->available()) {
      if (millis() - startTime > 1000) {
          result = 0; // Indicate a timeout error
          return result;
      }
      delay(delayPerCheck);
  }

  while (stream->available() > 0 && received == false && ndx <= numChars-1) {
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
      if(strncmp(measCommand, receivedChars, (strlen(measCommand)-1)) == 0){ // Compare command and received string (FS echoes the command)
        result = 1;
      }
      else{
        result = 9;             // return 9 if there is a mismatch
      }
    }
  }
  return result;
}

// Measure DO function: same as measure function but with pre-set measurement command
int Ardoxy::measureDO(int chan)//, int serialDelay=100)
{
  int result;
  unsigned long startTime; // Used for timeout

  // Paste Channel in measurement command
  if(ver >= 400){
    sprintf(measCommand, "MEA %d 1\r", chan);           // insert channel in measurement command
  } else if (ver < 400) {
    sprintf(measCommand, "MSR %d\r", chan);           // insert channel in measurement command
  }


  // Set source Stream
  stream = !hwStream? (Stream*)swStream : hwStream;

  // Empty Serial buffer
  while(stream->available() > 0){
    char t = stream->read();
    delay(2);
  }

  // Send command to FireSting
  stream->write(measCommand);
  stream->flush();
  bool received = false;      // Switch to continue reading incoming data until end marker was received

  // Wait for data with timeout mechanism
  startTime = millis();
  while (!stream->available()) {
      if (millis() - startTime > 1000) {
          result = 0; // Indicate a timeout error
          return result;
      }
      delay(delayPerCheck);
  }

  while (stream->available() > 0 && received == false && ndx <= numChars-1) {
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
      if(strncmp(measCommand, receivedChars, (strlen(measCommand)-1)) == 0){ // Compare command and received string (FS echoes the command)
        result = 1;
      }
      else{
        result = 9;             // return 9 if there is a mismatch
      }
    }
  }
  return result;
}

// Measure DO function: same as measure function but with pre-set measurement command
int Ardoxy::measureTemp()//int serialDelay=300)
{
  int result;
  int chan = 1;
  unsigned long startTime; // Used for timeout

  // Paste Channel in measurement command
  if(ver >= 400){
    sprintf(measCommand, "MEA %d 2\r", chan);           // insert channel in measurement command
  } else if (ver < 400) {
    sprintf(measCommand, "TMP %d\r", chan);           // insert channel in measurement command
  }

  // Set source Stream
  stream = !hwStream? (Stream*)swStream : hwStream;

  // Empty Serial buffer
  while(stream->available() > 0){
    char t = stream->read();
    delay(2);
  }

  // Send command to FireSting
  stream->write(measCommand);
  stream->flush();
  bool received = false;      // Switch to continue reading incoming data until end marker was received

  // Wait for data with timeout mechanism
  startTime = millis();
  while (!stream->available()) {
      if (millis() - startTime > 1000) {
          result = 0; // Indicate a timeout error
          return result;
      }
      delay(delayPerCheck);
  }

  while (stream->available() > 0 && received == false && ndx <= numChars-1) {
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
      if(strncmp(measCommand, receivedChars, (strlen(measCommand)-1)) == 0){ // Compare command and received string (FS echoes the command)
        result = 1;
      }
      else{
        result = 9;             // return 9 if there is a mismatch
      }
    }
  }
  //}
  return result;
}

// Readout values from Firesting memory
// Returns:
// numerical value (air saturation or temperature) - refer to Firesting Protocol
// 0 if there is a communication mismatch
long Ardoxy::readout(char command[])
{                                       // receives serial data and stores it in array until endmarker is received
  long valInt;                                                                          // receives parsed numerical value
  unsigned long startTime; // Used for timeout

  // Set source Stream
  stream = !hwStream? (Stream*)swStream : hwStream;

  // Empty Serial buffer
  while(stream->available() > 0){
    char t = stream->read();
    delay(2);
  }

  stream->write(command);
  stream->flush();
  bool received = false;

  // Wait for data with timeout mechanism
  startTime = millis();
  while (!stream->available()) {
      if (millis() - startTime > 1000) {
          valInt = 0; // Indicate a timeout error
          return valInt;
      }
      delay(delayPerCheck);
  }

  while (stream->available() > 0 && received == false && ndx <= numChars-1) {          // only read serial data if the buffer was emptied before and it's new data
    delay(2);
    rc = stream->read();
    if (rc != endMarker && ndx < numChars-1) {
      receivedChars[ndx] = rc;                                                        // store the latest character in character array
      ndx++;
    }
    else {
      receivedChars[ndx] = '\0';                                                      // terminate the string if the end marker is received
      ndx = 0;
      if(strncmp(command, receivedChars, (strlen(command)-1)) == 0){
        received = true;
        char* separator = strrchr(receivedChars, ' ');    // the value is separated by a space -> strrchr finds the last occurrence of " " in receivedChars and points
                                                          // to the following part of the string
        valInt = atol(separator);                         // parse the character to integers - the air saturation values are returned as [% air saturation x 1000], temperature as [°C x 1000]
      }
      else{
        valInt = 0;
      }
    }
  }
  return valInt;
}

// readoutDO - Similar to readout function but with pre-set DO-readout command
long Ardoxy::readoutDO(int chan)
{                                       // receives serial data and stores it in array until endmarker is received
  long valInt;                                                                          // receives parsed numerical value
  unsigned long startTime; // Used for timeout

  // Set source Stream
  stream = !hwStream? (Stream*)swStream : hwStream;

  // Empty Serial buffer
  while(stream->available() > 0){
    char t = stream->read();
    delay(2);
  }

  // Paste Channel in measurement command
  sprintf(measCommand, "RMR %d 3 4 1\r", chan);           // insert channel in measurement command

  stream->write(measCommand);
  stream->flush();
  bool received = false;

  // Wait for data with timeout mechanism
  startTime = millis();
  while (!stream->available()) {
      if (millis() - startTime > 1000) {
          valInt = 0; // Indicate a timeout error
          return valInt;
      }
      delay(delayPerCheck);
  }

  while (stream->available() > 0 && received == false && ndx <= numChars-1) {          // only read serial data if the buffer was emptied before and it's new data
    delay(2);
    rc = stream->read();
    if (rc != endMarker && ndx < numChars-1) {
      receivedChars[ndx] = rc;                                                        // store the latest character in character array
      ndx++;
    }
    else {
      receivedChars[ndx] = '\0';                                                      // terminate the string if the end marker is received
      ndx = 0;
      if(strncmp(measCommand, receivedChars, (strlen(measCommand)-1)) == 0){
        received = true;
        char* separator = strrchr(receivedChars, ' ');    // the value is separated by a space -> strrchr finds the last occurrence of " " in receivedChars and points
                                                          // to the following part of the string
        valInt = atol(separator);                         // parse the character to integers - the air saturation values are returned as [% air saturation x 1000], temperature as [°C x 1000]
      }
      else{
        valInt = 0;
      }
    }
  }
  return valInt;
}

// readoutDO - Similar to readout function but with pre-set DO-readout command
long Ardoxy::readoutTemp()
{                                       // receives serial data and stores it in array until endmarker is received
  long valInt;                                                                          // receives parsed numerical value
  unsigned long startTime; // Used for timeout

  // Set source Stream
  stream = !hwStream? (Stream*)swStream : hwStream;

  // Empty Serial buffer
  while(stream->available() > 0){
    char t = stream->read();
    delay(2);
  }

  // Paste Channel in measurement command
  sprintf(measCommand, "RMR 1 3 5 1\r");           // insert channel in measurement command

  stream->write(measCommand);
  stream->flush();
  bool received = false;

  // Wait for data with timeout mechanism
  startTime = millis();
  while (!stream->available()) {
      if (millis() - startTime > 1000) {
          valInt = 0; // Indicate a timeout error
          return valInt;
      }
      delay(delayPerCheck);
  }

  while (stream->available() > 0 && received == false && ndx <= numChars-1) {          // only read serial data if the buffer was emptied before and it's new data
    delay(2);
    rc = stream->read();
    if (rc != endMarker && ndx < numChars-1) {
      receivedChars[ndx] = rc;                                                        // store the latest character in character array
      ndx++;
    }
    else {
      receivedChars[ndx] = '\0';                                                      // terminate the string if the end marker is received
      ndx = 0;
      if(strncmp(measCommand, receivedChars, (strlen(measCommand)-1)) == 0){
        received = true;
        char* separator = strrchr(receivedChars, ' ');    // the value is separated by a space -> strrchr finds the last occurrence of " " in receivedChars and points
                                                          // to the following part of the string
        valInt = atol(separator);                         // parse the character to integers - the air saturation values are returned as [% air saturation x 1000], temperature as [°C x 1000]
      }
      else{
        valInt = 0;
      }
    }
  }
  return valInt;
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
