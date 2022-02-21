#ifndef _WP_SD_H
#define _WP_SD_H

#include <SD.h>

File dataFile;
File numFile;
char trackfn[] = ".file_track";
bool SDConnected;
bool Logging;
char fileName[20];
int curNum;

void try_SD_begin(int chipSelect)
{
  Logging = false;
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    SDConnected = false;
  }
  Serial.println("card initialized.");
  SDConnected = true;
  curNum = -1;
}

void toggle_Logging()
{
  Logging = !Logging;
  // if we are to start logging
  if(Logging)
  {
    // check if the current Number hasn't been initialized
    // (either by reading the file or stored in memory)
    if(curNum == -1 && SD.exists(trackfn)) 
    { 
      // open the tracker file
      numFile = SD.open(trackfn);
      char strBuff[10]; // create a str buffer to store the file inc int
      byte index = 0; // init index
      char nextChar; // track the next character in the file
      // while we are still on the first line and there are characters available
      while(nextChar != '\n' && numFile.available())
      {
         nextChar = numFile.read(); // read the next char
         strBuff[index++] = nextChar; // store that char in the str buffer
         strBuff[index] = '\0'; // backfill the end of the sting with null chars
      }
      int tmpNum = atoi(strBuff); // convert str to int
      curNum = ++tmpNum; // increment the tracked number
      numFile.close(); // close the file from reading
    }
    // otherwise, the current number is stored in memory
    else
    {
      curNum = ++curNum; // just increment it
    }
    // create the new filename
    sprintf(fileName, "log%i.csv", curNum);

    SD.remove(trackfn);
    // now reopen the file so we can store the new number
    numFile = SD.open(trackfn, FILE_WRITE);

    // if the file opened okay, write to it:
    if (numFile) 
    {
      Serial.println("Updating tracking file...");
      numFile.println(curNum);
      // close the file:
      numFile.close();
      Serial.println("Done.");
    } else {
      // if the file didn't open, print an error:
      Serial.println("error updating tracking file");
    }
    Serial.println((String)"Began Logging in: " + fileName);
  }
  else
  {
    Serial.println("Logging stopped.");
  }
}

void try_Log_Data(String dataString)
{
  File dataFile = SD.open(fileName, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    // Serial.println(dataString);
  } 
  else 
  {
    // if the file isn't open, pop up an error:
    Serial.println((String) "error opening " + fileName);
  }
}

#endif
