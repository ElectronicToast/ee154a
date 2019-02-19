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
*                  - 10 kOhm Thermistor (ADC)
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
*
*  Pinout:
*           Busses
*               I2C0    BME280
*               I2C1    MPU9250
*               SPI     SD card 
*
*           Analog
*               A0      Current shunt, high reading 
*               A1      Current shunt, low reading 
*               A2      Battery thermistor
*               A3      External thermistor 
*
*           Digital:
*               D10     SD card SPI chip select line 
*     
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

#define DEBUG
#define N_TRIES_MAX 10  //how many times to attempt initialization of each sensor

#ifdef DEBUG
  #define DEBUG_PRINT(x) DEBUG_PORT.println(x)
#else
  #define DEBUG_PRINT(x)
#endif

// LED
bool led_state = LOW;


//////////////////////
//        ADC       //
//////////////////////
#define ADC_N_BITS 10
#define ADC_MAX_VAL 1023
#define ADC_VREF 3.3

//////////////////////
//        GPS       //
//////////////////////
#define     gpsPort     Serial1
unsigned long lastLog = 0; // Global var to keep of last time we logged

NMEAGPS  gps; // This parses the GPS characters
gps_fix  fix; // This holds on to the latest values

//////////////////////
//      BME280      //
//////////////////////
#define     SEALEVELPRESSURE_HPA    (1013.25)
Adafruit_BME280 bme; // I2C (pin 20,21)

//////////////////////
//      MPU9250     //
//////////////////////
// MPU-9250 sensor on I2C bus 1 (near AREF) with address 0x68
MPU9250 IMU(Wire1,0x68);

//////////////////////
//    THERM_PIN     //
//////////////////////
const int TMST_BAT_PIN = 2;
const int TMST_EXT_PIN = 3;

#define TMST_CAL_SLOPE -0.0713408
#define TMST_CAL_INT 60.537128

//////////////////////
//    SHUNT_PIN     //
//////////////////////
#define SHUNT_RES_OHM   3.0         // Value of shunt resistor
#define SHUNT_SCALE_H     4.13832853025937
#define SHUNT_SCALE_L     4.323499491353

const int SHUNT_PIN_H = 0;
const int SHUNT_PIN_L = 1;

//////////////////////
//     SD CARD      //
//////////////////////
// SPI chip select for the micro SD card
#define SD_CS_PIN           10

//////////////////////
//      LOGGING     //
//////////////////////
#define LOG_RATE            2000    // Log every 1 second
// Log File Definitions:
#define LOG_FILE_PREFIX     "log"   // Name of the log file.
#define MAX_LOG_FILES       100     // Number of log files that can be made
#define LOG_FILE_SUFFIX     "csv"   // Suffix of the log file

char logFileName[13]; // Char string to store the log file name

// Data to be logged:
#define LOG_COLUMN_COUNT    27

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
  (char *)"Therm_Batt_V",
  (char *)"Therm_Ext_V",
  (char *)"Batt_Shunt_H_V",
  (char *)"Batt_Shunt_L_V",
  (char *)"Batt_I",
  }; // log_col_names is printed at the top of the file.

void setup()
{
  #ifdef DEBUG
  DEBUG_PORT.begin(9600);
  #endif
  
  DEBUG_PRINT( F("BALLOON.INO: started\n") );
  // ------------ enable the builtin LED for debug ------------
  pinMode( LED_BUILTIN , OUTPUT );
  digitalWrite(LED_BUILTIN, LOW);
  
  // ------------ initialize GPS UART port ------------
  DEBUG_PRINT("Initializing GPS...");
  gpsPort.begin(9600);
  DEBUG_PRINT("GPS initialization done.");

  // ------------ initialize BME280 I2C port ------------
  DEBUG_PRINT("Initializing BME280...");
  for(int n_tries = 0; n_tries < N_TRIES_MAX; n_tries++){
    if (!bme.begin()) {
      DEBUG_PRINT("BME280 initialization failed!");
    } else {
      DEBUG_PRINT("BME280 initialization done.");
      break;
    }
  }
  
  // ------------ initialize MPU9250 I2C port ------------
  DEBUG_PRINT("Initializing MPU9250...");
  for(int n_tries = 0; n_tries < N_TRIES_MAX; n_tries++){
    if (!IMU.begin()) {
      DEBUG_PRINT("MPU9250 initialization failed!");
    } else {
      DEBUG_PRINT("MPU9250 initialization done.");
      break;
    }
  }
    
  while(true) {  
    // ------------ initialize SD card SPI port ------------
    DEBUG_PRINT("Initializing SD card...");
    if (!SD.begin(SD_CS_PIN)) {
      DEBUG_PRINT("SD card initialization failed!");
    } else {
      DEBUG_PRINT("SD card initialization done.");
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
      DEBUG_PRINT(buildHeader());
      break;
    }
  }
}

//--------------------------

void loop()
{
  unsigned long t = millis();
  if ((lastLog + LOG_RATE) <= t) { // If it's been LOG_RATE milliseconds since the last log:
    while (gps.available( gpsPort )) {
      lastLog = t; // Update the lastLog variable
      
      led_state = !led_state;
      digitalWrite(LED_BUILTIN, led_state);
      
      fix = gps.read();
      IMU.readSensor();
      
      File logFile = SD.open(logFileName, FILE_WRITE); // Open the log file
  
      if (logFile) { 
        String data = buildData();
        DEBUG_PRINT(data);
        logFile.print(data);
        logFile.close();
      }
    }
    //DEBUG_PORT.println("Done getting GPS readings");
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

  // ====================== Take Thermistor Values ======================
  // Thermistor voltages (V)

  int tmst_bat_raw = analogRead(TMST_BAT_PIN);
  int tmst_ext_raw = analogRead(TMST_EXT_PIN);
  double tmst_bat = TMST_CAL_SLOPE * tmst_bat_raw + TMST_CAL_INT;
  double tmst_ext = TMST_CAL_SLOPE * tmst_ext_raw + TMST_CAL_INT;
  
  data += String(tmst_bat, 6);
  data += String(",");
  data += String(tmst_ext, 6);
  data += String(",");

  // ====================== Take Shunt Resistor Value ======================
  // shunt resistor voltage readings
  //double currsense_v = readAnalogVoltage(SHUNT_PIN_L);
  double shunt_h = readAnalogVoltage(SHUNT_PIN_H);
  double shunt_l = readAnalogVoltage(SHUNT_PIN_L);
  data += String(shunt_h, 6);
  data += String(",");
  data += String(shunt_l, 6);
  data += String(",");
  //data += String(currsense_v, 6);
  //data += String(",");
  // Compute the current 
  double batt_curr = (SHUNT_SCALE_H * shunt_h - SHUNT_SCALE_L * shunt_l) / SHUNT_RES_OHM;
  //double batt_curr = (currsense_v - 0.185) * 100.0 / (0.546 - 0.185);
  data += String(batt_curr, 6);
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

double readAnalogVoltage(int pin) {
  int raw = analogRead(pin);
  return (double)raw * (double)ADC_VREF / (double)ADC_MAX_VAL;
}

// updateFileName() - Looks through the log files already present on a card,
// and creates a new file with an incremented file index.
static void updateFileName() {
  int i = 0;
  for (; i < MAX_LOG_FILES; i++) {
    memset(logFileName, 0, strlen(logFileName)); // Clear logFileName string
    // Set logFileName to "log.csv":
    sprintf(logFileName, "%s%d.%s", LOG_FILE_PREFIX, i, LOG_FILE_SUFFIX);
    if (!SD.exists(logFileName)) {
      break; // Break out of this loop. We found our index
    } else {
      DEBUG_PRINT(logFileName);
      DEBUG_PRINT(" exists"); // Print a debug statement
    }
  }
  DEBUG_PRINT("File name: ");
  DEBUG_PRINT(logFileName); // Debug print the file name
}
