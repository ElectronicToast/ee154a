/**
 * This program samples thermistor voltage, BME280 temperature, and MPU9250 temperature
 * values every second. Values are to be used for calibrating the thermistor. 
 */

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "MPU9250.h"

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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.print("Initializing BME280...");

  if (!bme.begin()) {
    Serial.println("BME280 initialization failed!");
    while (1);
  }
  Serial.println("BME280 initialization done.");

  // ------------ initialize MPU9250 I2C port ------------
  Serial.print("Initializing MPU9250...");

  if (!IMU.begin()) {
    Serial.println("MPU9250 initialization failed!");
    while (1);
  }
  Serial.println("MPU9250 initialization done.");
  Serial.print("Thermistor (Blue - wrapped)");
  Serial.print('\t');
  Serial.print("Thermistor (Yellow)");
  Serial.print('\t');
  Serial.print("BME280");
  Serial.print('\t');
  Serial.print("MPU9250");
  Serial.println();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(analogRead(0));
  Serial.print('\t');
  Serial.print(analogRead(1));
  Serial.print('\t');
  Serial.print(bme.readTemperature());
  Serial.print('\t');
  Serial.print(IMU.getTemperature_C());
  Serial.println();
  delay(1000);
}
