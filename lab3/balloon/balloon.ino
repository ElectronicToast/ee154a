/*======================================================================
*  Program: balloon.ino
*
*  Description:  This program is designed for an Arduino Due to record
*                telemetry data on a high altitude balloon. It takes
*                the following measurements:
*
*                  - GP3906 GPS Reciever (UART)
*                        - position
*                        - compass heading
*                  - BME280 Temperature/Humidity/Pressure Sensor (I2C)
*                        - external temperature
*                        - external pressure
*                        - external humidity
*                        - altitude
*                  - MPU9250 IMU (I2C)
*                        - attitude
*                        - attitude rate
*                        - internal temperature
*                        - acceleration
*                        - compas heading
*                  - 810 kOhm Thermistor (ADC)
*                        - battery temperature
*                  - X Ohm Shunt Resistor (ADC)
*                        - battery current
*
*                The data is sampled periodically and stored in a
*                comma separated values file on a micro-SD card over
*                SPI.
*                
*  Authors: David Elliott, Ray Sun, Evan Yeh                
*           EE 154a Winter 2019
*           California Institute of Technology
*  ======================================================================
*/

#include <NMEAGPS.h>
#include <GPSport.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "MPU9250.h"
  
//////////////////////
//        GPS       //
//////////////////////
#define gpsPort Serial1
unsigned long lastLog = 0; // Global var to keep of last time we logged

NMEAGPS  gps; // This parses the GPS characters
gps_fix  fix; // This holds on to the latest values

//////////////////////
//      BME280      //
//////////////////////
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C (pin 20,21)

//////////////////////
//      MPU9250     //
//////////////////////
// MPU-9250 sensor on I2C bus 1 (near AREF) with address 0x68
MPU9250 IMU(Wire1,0x68);

//////////////////////
//    THERM_PIN     //
//////////////////////
const int THERM_PIN = 0;

//////////////////////
//    SHUNT_PIN     //
//////////////////////
const int SHUNT_PIN_H = 1;
const int SHUNT_PIN_L = 2;

//////////////////////
//     SD CARD      //
//////////////////////
// SPI chip select for the micro SD card
const int SDCHIPSELECT = 10;

//////////////////////
//      LOGGING     //
//////////////////////
#define LOG_RATE 2000 // Log every 1 second
// Log File Definitions:
#define LOG_FILE_PREFIX "log" // Name of the log file.
#define MAX_LOG_FILES 100 // Number of log files that can be made
#define LOG_FILE_SUFFIX "csv" // Suffix of the log file
char logFileName[13]; // Char string to store the log file name
// Data to be logged:
#define LOG_COLUMN_COUNT 24
char * log_col_names[LOG_COLUMN_COUNT] = {
  (char *)"GPS_longitude", 
  (char *)"GPS_latitude", 
  (char *)"GPS_altitude", 
  (char *)"GPS_speed", 
  (char *)"GPS_heading", 
  (char *)"GPS_date", 
  (char *)"GPS_time", 
  (char *)"GPS_satellites",
  (char *)"BME280_temperature",
  (char *)"BME280_humidity",
  (char *)"BME280_pressure",
  (char *)"BME280_altitude",
  (char *)"IMU_AccelX",
  (char *)"IMU_AccelY",
  (char *)"IMU_AccelZ",
  (char *)"IMU_GyroX",
  (char *)"IMU_GyroY",
  (char *)"IMU_GyroZ",
  (char *)"IMU_MagX",
  (char *)"IMU_MagY",
  (char *)"IMU_MagZ",
  (char *)"IMU_Temp",
  (char *)"Therm_V",
  (char *)"BattTemp_V"
  }; // log_col_names is printed at the top of the file.

void setup()
{
  DEBUG_PORT.begin(9600);
  while (!Serial);
  DEBUG_PORT.print( F("BALLOON.INO: started\n") );

  // ------------ initialize GPS UART port ------------
  DEBUG_PORT.print("Initializing GPS...");
  gpsPort.begin(9600);
  DEBUG_PORT.println("GPS initialization done.");

  // ------------ initialize BME280 I2C port ------------
  DEBUG_PORT.print("Initializing BME280...");

  if (!bme.begin()) {
    DEBUG_PORT.println("BME280 initialization failed!");
    while (1);
  }
  DEBUG_PORT.println("BME280 initialization done.");
  
  // ------------ initialize MPU9250 I2C port ------------
  DEBUG_PORT.print("Initializing MPU9250...");

  if (!IMU.begin()) {
    DEBUG_PORT.println("MPU9250 initialization failed!");
    while (1);
  }
  DEBUG_PORT.println("MPU9250 initialization done.");
  
  // ------------ initialize SD card SPI port ------------
  DEBUG_PORT.print("Initializing SD card...");

  if (!SD.begin(SDCHIPSELECT)) {
    DEBUG_PORT.println("SD card initialization failed!");
    while (1);
  }
  DEBUG_PORT.println("SD card initialization done.");

  // ------------ create new log file  ------------
  // Each time we start, create a new file, increment the number
  updateFileName(); 
  // Print a header at the top of the new file
  File logFile = SD.open(logFileName, FILE_WRITE); // Open the log file
  if(logFile) 
  {
    logFile.print(buildHeader());
    logFile.close();
  }
  DEBUG_PORT.print(buildHeader());

  // ------------ enable the builtin LED for waitForFix() ------------
  pinMode( LED_BUILTIN , OUTPUT );

  // uncomment when not underground
  //  waitForFix();     
  
}

//--------------------------

void loop()
{
  if ((lastLog + LOG_RATE) <= millis()) { // If it's been LOG_RATE milliseconds since the last log:
    while (gps.available( gpsPort )) {
      fix = gps.read();
      IMU.readSensor();
      
      File logFile = SD.open(logFileName, FILE_WRITE); // Open the log file
  
      // add below to if condition when not underground
      // && fix.valid.location && fix.valid.time
      if (logFile) { 
        String data = buildData();
        
        DEBUG_PORT.print(data);
        logFile.print(data);
        
        logFile.close();
        
        lastLog = millis(); // Update the lastLog variable
      }
    }
  }  
}

// builds and returns a String of comma-separated telemetry data
// used to print to both DEBUG_PORT and log file
String buildData()
{
  String data = "";
  data += String(fix.longitude(), 6);
  data += String(',');
  data += String(fix.latitude(), 6);
  data += String(',');
  data += String(fix.altitude(), 6);
  data += String(',');
  data += String(fix.speed_kph(), 6);
  data += String(',');
  data += String(fix.heading(), 6);
  data += String(',');

  // print formated date
  data += String(fix.dateTime.year);
  data += String('/');
  data += String(fix.dateTime.month);
  data += String('/');
  data += String(fix.dateTime.date);
  data += String(',');
  
  // print formatted time
  if (fix.dateTime.hours < 10)
    data += String('0' );
  data += String(fix.dateTime.hours);
  data += String(':' );
  if (fix.dateTime.minutes < 10)
    data += String('0' );
  data += String(fix.dateTime.minutes);
  data += String(':' );
  if (fix.dateTime.seconds < 10)
    data += String('0' );
  data += String(fix.dateTime.seconds);
  data += String('.' );
  if (fix.dateTime_cs < 10)
     data += String(  '0' ); // leading zero for .05, for example
  data += String(fix.dateTime_cs);
  data += String(',');
  data += String(fix.satellites);
  data += String(',');        

  // ====================== Take BME280 Data ======================
  // temperature (C), humidity (%), pressure (kPa), altitude (m)
  data += String(bme.readTemperature(), 6);
  data += String(",");
  data += String(bme.readHumidity(), 6);
  data += String(",");
  data += String(bme.readPressure() / 1000.0F, 6);
  data += String(",");
  data += String(bme.readAltitude(SEALEVELPRESSURE_HPA), 6);
  data += String(",");
  
  // ====================== Take IMU Data ======================
  // accel XYZ (m/s^2), gyro XYZ (rad/s^2), mag XYZ (uT), temperature (C)
  data += String(IMU.getAccelX_mss(), 6);
  data += String(",");
  data += String(IMU.getAccelY_mss(), 6);
  data += String(",");
  data += String(IMU.getAccelZ_mss(), 6);
  data += String(",");
  data += String(IMU.getGyroX_rads(), 6);
  data += String(",");
  data += String(IMU.getGyroY_rads(), 6);
  data += String(",");
  data += String(IMU.getGyroZ_rads(), 6);
  data += String(",");
  data += String(IMU.getMagX_uT(), 6);
  data += String(",");
  data += String(IMU.getMagY_uT(), 6);
  data += String(",");
  data += String(IMU.getMagZ_uT(), 6);
  data += String(",");
  data += String(IMU.getTemperature_C(), 6);
  data += String(",");

  // ====================== Take Thermistor Value ======================
  // thermistor voltage (V)
  data += String(analogRead(THERM_PIN), 6);
  data += String(",");

  // ====================== Take Shunt Resistor Value ======================
  // shunt resistor voltage drop (V)
  data += String(analogRead(SHUNT_PIN_H) - analogRead(SHUNT_PIN_L), 6);
  
  data += '\n';
  return data;
}

// buildHeader() - builds the column headers to put at the top of the file
//                 returns as a string
String buildHeader()
{
  String header = "";

  int i = 0;
  for (; i < LOG_COLUMN_COUNT; i++)
  {
    header += String(log_col_names[i]);
    if (i < LOG_COLUMN_COUNT - 1) // If it's anything but the last column
      header += String(','); // print a comma
    else // If it's the last column
      header += String('\n'); // print a new line
  }
  return header;
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
