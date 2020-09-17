#include <SD.h>        // SD card
#include "RTClib.h"    // real time clock

const byte chipSelect      = 10; // Adafruit SD shields and modules: pin 10
RTC_PCF8523         rtc;
DateTime            now;

void setup() {
  delay(3000);
  // Check if SD Card is working...
  if (!SD.begin(chipSelect)) {
    SerialUSB.println("E: SD Card failed, or not present");
    // return; // don't do anything more:
  } else {
    SerialUSB.println("SD is working...");
  }
  // Check if the RT Clock ready
  if (! rtc.begin()) {
    SerialUSB.println("Couldn't find RT Clock");
    return;
  }
  printCurrentTime();
  if (! rtc.initialized()) {
    SerialUSB.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    SerialUSB.println("Init clock with current time");
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  } else {
    SerialUSB.println("RTC is running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    SerialUSB.println("Update with current time!");
  }
  printCurrentTime();
  delay(5000);
  printCurrentTime();
   delay(5000);
  printCurrentTime();
}

void loop() {
  // put your main code here, to run repeatedly:

}

void printCurrentTime(){
  // print current time and date // 2 ms
  now = rtc.now();
  SerialUSB.print("Now: ");
  SerialUSB.print(now.year(), DEC);
  SerialUSB.print('/');
  SerialUSB.print(now.month(), DEC);
  SerialUSB.print('/');
  SerialUSB.print(now.day(), DEC);
  SerialUSB.print(" ");
  SerialUSB.print(now.hour(), DEC);
  SerialUSB.print(':');
  SerialUSB.print(now.minute(), DEC);
  SerialUSB.print(':');
  SerialUSB.print(now.second(), DEC);
  SerialUSB.println();
}

