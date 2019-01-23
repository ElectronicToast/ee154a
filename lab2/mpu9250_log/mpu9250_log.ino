/*
 * mpu9250_log.ino
 * 
 * David Elliott, Ray Sun, Adi Telik, Evan Yeh
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

#include "quaternionFilters.h"
#include "MPU9250.h"

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins

MPU9250 myIMU;

void setup()
{
    Wire.begin();
    Serial.begin(38400);

    pinMode(intPin, INPUT);
    digitalWrite(intPin, LOW);

    byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
    Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
    Serial.print(" I should be "); Serial.println(0x71, HEX);

    if (c == 0x71) {
        Serial.println("MPU9250 is online...");

        // Start by performing self test and reporting values
        myIMU.MPU9250SelfTest(myIMU.SelfTest);
        Serial.print("x-axis self test: acceleration trim within : ");
        Serial.print(myIMU.SelfTest[0],1); Serial.println("% of factory value");
        Serial.print("y-axis self test: acceleration trim within : ");
        Serial.print(myIMU.SelfTest[1],1); Serial.println("% of factory value");
        Serial.print("z-axis self test: acceleration trim within : ");
        Serial.print(myIMU.SelfTest[2],1); Serial.println("% of factory value");
        Serial.print("x-axis self test: gyration trim within : ");
        Serial.print(myIMU.SelfTest[3],1); Serial.println("% of factory value");
        Serial.print("y-axis self test: gyration trim within : ");
        Serial.print(myIMU.SelfTest[4],1); Serial.println("% of factory value");
        Serial.print("z-axis self test: gyration trim within : ");
        Serial.print(myIMU.SelfTest[5],1); Serial.println("% of factory value");

        // Calibrate gyro and accelerometers, load biases in bias registers
        myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
        myIMU.initMPU9250();
        // Initialize device for active mode read of acclerometer, gyroscope, and
        // temperature
        Serial.println("MPU9250 initialized for active data mode....");

        // Read the WHO_AM_I register of the magnetometer, this is a good test of
        // communication
        byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
        Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
        Serial.print(" I should be "); Serial.println(0x48, HEX);

        // Get magnetometer calibration from AK8963 ROM
        myIMU.initAK8963(myIMU.magCalibration);
        // Initialize device for active mode read of magnetometer
        Serial.println("AK8963 initialized for active data mode....");
        //  Serial.println("Calibration values: ");
        Serial.print("X-Axis sensitivity adjustment value ");
        Serial.println(myIMU.magCalibration[0], 2);
        Serial.print("Y-Axis sensitivity adjustment value ");
        Serial.println(myIMU.magCalibration[1], 2);
        Serial.print("Z-Axis sensitivity adjustment value ");
        Serial.println(myIMU.magCalibration[2], 2);
    } else {
        Serial.print("Could not connect to MPU9250: 0x");
        Serial.println(c, HEX);
        while(1);
    }
}

void loop()
{
    if(myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  
        myIMU.readAccelData(myIMU.accelCount);
        myIMU.getAres();

        // Now we'll calculate the accleration value into actual g's
        // This depends on scale being set
        myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes;
        myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes;
        myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes;

        myIMU.readGyroData(myIMU.gyroCount);
        myIMU.getGres();

        // Calculate the gyro value into actual degrees per second
        // This depends on scale being set
        myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
        myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
        myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;

        myIMU.readMagData(myIMU.magCount);
        myIMU.getMres();

        // User environmental x-axis correction in milliGauss, should be
        // automatically calculated
        myIMU.magbias[0] = 0;//+470.;
        // User environmental x-axis correction in milliGauss TODO axis??
        myIMU.magbias[1] = 0;//+120.;
        // User environmental x-axis correction in milliGauss
        myIMU.magbias[2] = 0;//+125.;

        // Calculate the magnetometer values in milliGauss
        // Include factory calibration per data sheet and user environmental
        // corrections
        // Get actual magnetometer value, this depends on scale being set
        myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] -
                myIMU.magbias[0];
        myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] -
                myIMU.magbias[1];
        myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] -
                myIMU.magbias[2];
    } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

    // Must be called before updating quaternions!
    myIMU.updateTime();

    MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx*DEG_TO_RAD,
                            myIMU.gy*DEG_TO_RAD, myIMU.gz*DEG_TO_RAD, myIMU.my,
                            myIMU.mx, myIMU.mz, myIMU.deltat);

    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 500)
    {
        // Print acceleration values in milligs!
        Serial.print("X-acceleration: "); Serial.print(1000*myIMU.ax);
        Serial.print(" mg ");
        Serial.print("Y-acceleration: "); Serial.print(1000*myIMU.ay);
        Serial.print(" mg ");
        Serial.print("Z-acceleration: "); Serial.print(1000*myIMU.az);
        Serial.println(" mg ");

        // Print gyro values in degree/sec
        Serial.print("X-gyro rate: "); Serial.print(myIMU.gx, 3);
        Serial.print(" degrees/sec ");
        Serial.print("Y-gyro rate: "); Serial.print(myIMU.gy, 3);
        Serial.print(" degrees/sec ");
        Serial.print("Z-gyro rate: "); Serial.print(myIMU.gz, 3);
        Serial.println(" degrees/sec");

        // Print mag values in degree/sec
        Serial.print("X-mag field: "); Serial.print(myIMU.mx);
        Serial.print(" mG ");
        Serial.print("Y-mag field: "); Serial.print(myIMU.my);
        Serial.print(" mG ");
        Serial.print("Z-mag field: "); Serial.print(myIMU.mz);
        Serial.println(" mG");

        myIMU.tempCount = myIMU.readTempData();  // Read the adc values
        // Temperature in degrees Centigrade
        myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
        // Print temperature in degrees Centigrade
        Serial.print("Temperature is ");  Serial.print(myIMU.temperature, 1);
        Serial.println(" degrees C");

        myIMU.count = millis();
    } // if (myIMU.delt_t > 500)
}
