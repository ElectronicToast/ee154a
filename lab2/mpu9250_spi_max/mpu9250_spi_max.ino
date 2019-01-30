/*
 * mpu9250_log.ino
 * 
 * David Elliott, Ray Sun, Evan Yeh
 * EE 154a Winter 2019
 * California Institute of Technology
 */

/*
    Hardware setup:
    MPU9250 Breakout --------- Arduino
    VDD ---------------------- 3.3V
    VDDI --------------------- 3.3V
    SDA ----------------------- A4
    SCL ----------------------- A5
    GND ---------------------- GND
*/

#include "MPU9250.h"

#DEFINE     SERIAL_BAUD     115200


// an MPU9250 object with the MPU-9250 sensor on SPI bus 0 and chip select pin 10
MPU9250 IMU(SPI, 10);
int status;

void setup() {
    // serial to display data
    Serial.begin(SERIAL_BAUD);
    while(!Serial) {}
    
    // start communication with IMU 
    status = IMU.begin();
    if (status < 0) {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(status);
        while(1) {}
    }
}

void loop() {
    // read the sensor
    IMU.readSensor();
    
    // display the data
    Serial.print(IMU.getAccelX_mss(),6);
    Serial.print("\t");
    Serial.print(IMU.getAccelY_mss(),6);
    Serial.print("\t");
    Serial.print(IMU.getAccelZ_mss(),6);
    Serial.print("\t");
    Serial.print(IMU.getGyroX_rads(),6);
    Serial.print("\t");
    Serial.print(IMU.getGyroY_rads(),6);
    Serial.print("\t");
    Serial.print(IMU.getGyroZ_rads(),6);
    Serial.print("\t");
    //Serial.print(IMU.getMagX_uT(),6);
    //Serial.print("\t");
    //Serial.print(IMU.getMagY_uT(),6);
    //Serial.print("\t");
    //Serial.print(IMU.getMagZ_uT(),6);
    //Serial.print("\t");
    //Serial.println(IMU.getTemperature_C(),6);
    //delay(100);
}