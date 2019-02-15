#include <NMEAGPS.h>

//======================================================================
//  Program: NMEAsimple.ino
//
//  Description:  This program shows simple usage of NeoGPS
//
//  Prerequisites:
//     1) NMEA.ino works with your device (correct TX/RX pins and baud rate)
//     2) At least one of the RMC, GGA or GLL sentences have been enabled in NMEAGPS_cfg.h.
//     3) Your device at least one of those sentences (use NMEAorder.ino to confirm).
//     4) LAST_SENTENCE_IN_INTERVAL has been set to one of those sentences in NMEAGPS_cfg.h (use NMEAorder.ino).
//     5) LOCATION and ALTITUDE have been enabled in GPSfix_cfg.h
//
//  'Serial' is for debug output to the Serial Monitor window.
//
//  License:
//    Copyright (C) 2014-2017, SlashDevin
//
//    This file is part of NeoGPS
//
//    NeoGPS is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    NeoGPS is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with NeoGPS.  If not, see <http://www.gnu.org/licenses/>.
//
//======================================================================

#include <GPSport.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "MPU9250.h"

//////////////////////
//      BME280      //
//////////////////////
#define SEALEVELPRESSURE_HPA (1013.25)
#define START_TEMP    -30
#define TEMP_INC      5
Adafruit_BME280 bme; // I2C

//////////////////////
//      MPU9250     //
//////////////////////
// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire1,0x68);

//////////////////////
//        GPS       //
//////////////////////
#define gpsPort Serial1
#define LOG_RATE 1000 // Log every 5 seconds
unsigned long lastLog = 0; // Global var to keep of last time we logged

// Log File Definitions:
#define LOG_FILE_PREFIX "log" // Name of the log file.
#define MAX_LOG_FILES 100 // Number of log files that can be made
#define LOG_FILE_SUFFIX "csv" // Suffix of the log file
char logFileName[13]; // Char string to store the log file name
// Data to be logged:
#define LOG_COLUMN_COUNT 8
char * log_col_names[LOG_COLUMN_COUNT] = {
  (char *)"longitude", 
  (char *)"latitude", 
  (char *)"altitude", 
  (char *)"speed", 
  (char *)"heading", 
  (char *)"date", 
  (char *)"time", 
  (char *)"satellites"}; // log_col_names is printed at the top of the file.

NMEAGPS  gps; // This parses the GPS characters
gps_fix  fix; // This holds on to the latest values

//////////////////////
//     SD CARD      //
//////////////////////
// SPI chip select for the micro SD card
const int SDCHIPSELECT = 10;

void setup()
{
  DEBUG_PORT.begin(9600);
  while (!Serial)
    ;
  DEBUG_PORT.print( F("BALLOON.INO: started\n") );

  // ------------ initialize GPS UART port ------------
  DEBUG_PORT.print("Initializing GPS...");
  gpsPort.begin(9600);
  DEBUG_PORT.println("GPS initialization done.");

  // ------------ initialize SD card SPI port ------------
  DEBUG_PORT.print("Initializing SD card...");

  if (!SD.begin(SDCHIPSELECT)) {
    DEBUG_PORT.println("SD card initialization failed!");
    while (1);
  }
  DEBUG_PORT.println("SD card initialization done.");

  updateFileName(); // Each time we start, create a new file, increment the number
  printHeader(); // Print a header at the top of the new file

  // ------------ initialize BME280 i2c port ------------
  DEBUG_PORT.print("Initializing BME280...");

  if (!bme.begin()) {
    DEBUG_PORT.println("BME280 initialization failed!");
    while (1);
  }
  DEBUG_PORT.println("BME280 initialization done.");

  // ------------ initialize MPU9250 i2c port ------------
  DEBUG_PORT.print("Initializing MPU9250...");

  if (!IMU.begin()) {
    DEBUG_PORT.println("MPU9250 initialization failed!");
    while (1);
  }
  DEBUG_PORT.println("MPU9250 initialization done.");

  // ------------ enable the builtin LED for blinking feedback ------------
  pinMode( LED_BUILTIN , OUTPUT );

  // uncomment when not underground
//  waitForFix();
  
}

//--------------------------

void loop()
{
  if ((lastLog + LOG_RATE) <= millis())
  { // If it's been LOG_RATE milliseconds since the last log:
    while (gps.available( gpsPort )) {
      fix = gps.read();
      IMU.readSensor();
      
      File logFile = SD.open(logFileName, FILE_WRITE); // Open the log file
  
      // add below to if condition when not underground
      // && fix.valid.location && fix.valid.time
      if (logFile)
      { // Print longitude, latitude, altitude (in m), speed (in kph), heading
        // in (degrees), date, time, and number of satellites.
               
        DEBUG_PORT.print(fix.longitude());
        DEBUG_PORT.print(',');
        DEBUG_PORT.print(fix.latitude());
        DEBUG_PORT.print(',');
        DEBUG_PORT.print(fix.altitude());
        DEBUG_PORT.print(',');
        DEBUG_PORT.print(fix.speed_kph());
        DEBUG_PORT.print(',');
        DEBUG_PORT.print(fix.heading());
        DEBUG_PORT.print(',');

        // print formated date
        DEBUG_PORT.print(fix.dateTime.year);
        DEBUG_PORT.print('/');
        DEBUG_PORT.print(fix.dateTime.month);
        DEBUG_PORT.print('/');
        DEBUG_PORT.print(fix.dateTime.date);
        DEBUG_PORT.print(',');
        
        // print formatted time
        if (fix.dateTime.hours < 10)
          DEBUG_PORT.print( '0' );
        DEBUG_PORT.print(fix.dateTime.hours);
        DEBUG_PORT.print( ':' );
        if (fix.dateTime.minutes < 10)
          DEBUG_PORT.print( '0' );
        DEBUG_PORT.print(fix.dateTime.minutes);
        DEBUG_PORT.print( ':' );
        if (fix.dateTime.seconds < 10)
          DEBUG_PORT.print( '0' );
        DEBUG_PORT.print(fix.dateTime.seconds);
        DEBUG_PORT.print( '.' );
        if (fix.dateTime_cs < 10)
           DEBUG_PORT.print( '0' ); // leading zero for .05, for example
        DEBUG_PORT.print(fix.dateTime_cs);
        DEBUG_PORT.print(',');
        
        DEBUG_PORT.print(fix.satellites);

        // test temperature
        DEBUG_PORT.print(bme.readTemperature());

        // test IMU
        DEBUG_PORT.print(IMU.getAccelX_mss(),6);
        DEBUG_PORT.print("\t");
        DEBUG_PORT.print(IMU.getAccelY_mss(),6);
        DEBUG_PORT.print("\t");
        DEBUG_PORT.print(IMU.getAccelZ_mss(),6);
        DEBUG_PORT.print("\t");
        DEBUG_PORT.print(IMU.getGyroX_rads(),6);
        DEBUG_PORT.print("\t");
        DEBUG_PORT.print(IMU.getGyroY_rads(),6);
        DEBUG_PORT.print("\t");
        DEBUG_PORT.println(IMU.getGyroZ_rads(),6);
        
        DEBUG_PORT.println();
       
        logFile.close();
        
        lastLog = millis(); // Update the lastLog variable
      }
    }
    
    


  }

  
  
}


// printHeader() - prints our eight column names to the top of our log file
void printHeader()
{
  File logFile = SD.open(logFileName, FILE_WRITE); // Open the log file

  if (logFile) // If the log file opened, print our column names to the file
  {
    int i = 0;
    for (; i < LOG_COLUMN_COUNT; i++)
    {
      DEBUG_PORT.print(log_col_names[i]);
      if (i < LOG_COLUMN_COUNT - 1) // If it's anything but the last column
        DEBUG_PORT.print(','); // print a comma
      else // If it's the last column
        DEBUG_PORT.println(); // print a new line
    }
    logFile.close(); // close the file
  }
}

// updateFileName() - Looks through the log files already present on a card,
// and creates a new file with an incremented file index.
static void updateFileName()
{
  int i = 0;
  for (; i < MAX_LOG_FILES; i++)
  {
    memset(logFileName, 0, strlen(logFileName)); // Clear logFileName string
    // Set logFileName to "log.csv":
    sprintf(logFileName, "%s%d.%s", LOG_FILE_PREFIX, i, LOG_FILE_SUFFIX);
    if (!SD.exists(logFileName)) // If a file doesn't exist
    {
      break; // Break out of this loop. We found our index
    }
    else // Otherwise:
    {
      DEBUG_PORT.print(logFileName);
      DEBUG_PORT.println(" exists"); // Print a debug statement
    }
  }
  DEBUG_PORT.print("File name: ");
  DEBUG_PORT.println(logFileName); // Debug print the file name
}


//----------------------------------------------------------------
//  This routine waits for GPSisr to provide 
//  a fix that has a valid location.
//
//  The LED is slowly flashed while it's waiting.
static void waitForFix()
{
  DEBUG_PORT.print( F("Waiting for GPS fix...") );

  uint16_t lastToggle = millis();

  for (;;) {
    if (gps.available()) {
      if (gps.read().valid.location)
        break; // Got it!
    }

    // Slowly flash the LED until we get a fix
    if ((uint16_t) millis() - lastToggle > 500) {
      lastToggle += 500;
      digitalWrite( LED_BUILTIN , !digitalRead(LED_BUILTIN ) );
      DEBUG_PORT.write( '.' );
    }
  }
  DEBUG_PORT.println();

  digitalWrite( LED_BUILTIN , LOW );

  gps.overrun( false ); // we had to wait a while...

} // waitForFix
