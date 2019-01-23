/*
 * bme280_log.ino - 5 deg increment temperature, pressure, humidity log with Arduino Due
 * 
 * David Elliot, Ray Sun, Adi Telik, Evan Yeh
 * EE 154a Winter 2019
 * California Institute of Technology
 * 
 * Notes:
 *  - Download library from https://github.com/sparkfun/SparkFun_BME280_Arduino_Library/
 */

#include <stdint.h>
#include "SparkFunBME280.h"

#include "Wire.h"
#include "SPI.h"

#define START_TEMP    -30
#define TEMP_INC      5

//Global sensor object
BME280 atmoSensor;

void setup()
{
    //***Driver settings********************************//
    //commInterface can be I2C_MODE or SPI_MODE
    //specify chipSelectPin using arduino pin names
    //specify I2C address.  Can be 0x77(default) or 0x76

    //For I2C, enable the following and disable the SPI section
    //atmoSensor.settings.commInterface = I2C_MODE;
    //atmoSensor.settings.I2CAddress = 0x77;

    //For SPI enable the following and dissable the I2C section
    atmoSensor.settings.commInterface = SPI_MODE;
    atmoSensor.settings.chipSelectPin = 10;


    //***Operation settings*****************************//

    //runMode can be:
    //  0, Sleep mode
    //  1 or 2, Forced mode
    //  3, Normal mode
    atmoSensor.settings.runMode = 3; //Forced mode

    //tStandby can be:
    //  0, 0.5ms
    //  1, 62.5ms
    //  2, 125ms
    //  3, 250ms
    //  4, 500ms
    //  5, 1000ms
    //  6, 10ms
    //  7, 20ms
    atmoSensor.settings.tStandby = 0;

    //filter can be off or number of FIR coefficients to use:
    //  0, filter off
    //  1, coefficients = 2
    //  2, coefficients = 4
    //  3, coefficients = 8
    //  4, coefficients = 16
    atmoSensor.settings.filter = 0;

    //tempOverSample can be:
    //  0, skipped
    //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
    atmoSensor.settings.tempOverSample = 1;

    //pressOverSample can be:
    //  0, skipped
    //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
    atmoSensor.settings.pressOverSample = 1;

    //humidOverSample can be:
    //  0, skipped
    //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
    atmoSensor.settings.humidOverSample = 1;
    delay(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.         
    Serial.begin(57600);

    Serial.print("Starting BME280... result of .begin(): 0x");
    //Calling .begin() causes the settings to be loaded
    Serial.println(atmoSensor.begin(), HEX);
    Serial.println("Temperature (C), Pressure (kPa), Humidity (%)");

    // ###########################################################################
    float currTemp = -100.0;
    float currStoreTemp = -100.0;

    // 16 increments of 5 degrees from -30 to 50 deg C
    for (uint8_t i = 0; i < 16; i++) {
        currStoreTemp = START_TEMP + TEMP_INC * i;
        while (currTemp < currStoreTemp)
            currTemp = atmoSensor.readTempC();
        Serial.print(currTemp);
        Serial.print(", ");
        Serial.print(atmoSensor.readFloatPressure());
        Serial.print(", ");
        Serial.println(atmoSensor.readFloatHumidity());
    }
    Serial.println("Done!");
}

void loop() {
  
}
