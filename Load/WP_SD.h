#ifndef _WP_SD_H
#define _WP_SD_H

#include <SD.h>

File dataFile;
File numFile;
char trackfn[] = "file_track.txt";
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
  if(Logging)
  {
    if(curNum == -1 && SD.exists(trackfn)) 
    { 
      numFile = SD.open(trackfn);
      char strBuff[10];
      byte index = 0;
      char nextChar;
      while(nextChar != '\n' && numFile.available())
      {
         nextChar = numFile.read();
         strBuff[index++] = nextChar;
         strBuff[index] = '\0';
      }
      int tmpNum = atoi(strBuff);
      curNum = ++tmpNum;
      numFile.close();
    }
    else{
      curNum = ++curNum;
    }
    sprintf(fileName, "log%i.csv", curNum);

    //need to also replace contents of numFile with new num.
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
    Serial.println(dataString);
  } else {
    // if the file isn't open, pop up an error:
    Serial.println("error opening datalog.txt");
  }
}

#endif
