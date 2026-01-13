/**********************************************************************************************************
MIT license - 1/11/2026

GY-85 Kalman Filter and Complementary Fiter Demos.
--------------------------------------------------
Soucecode Dependencies (Thanks to those authors):

https://github.com/TKJElectronics/KalmanFilter/blob/master/examples/MPU6050/MPU6050.ino
https://github.com/nopnop2002/esp-idf-gy85/blob/main/euler/main/gy85.cpp
https://github.com/adafruit/Adafruit_ADXL345/blob/master/examples/sensortest/sensortest.ino
https://github.com/adafruit/Adafruit_HMC5883_Unified/blob/master/examples/magsensor/magsensor.ino
https://github.com/Seeed-Studio/Grove_3_Axis_Digital_Gyro/blob/master/examples/ITG3200_gyro/ITG3200_gyro.ino
----------------------
Library Dependencies:

1. Kalman Filter Library
2. Grove 3-Axis Digital Gyro
3. Adafruit ADXL345
4. Adafruit HMC5883 Unified
----------
Hardware:

1. GY-85
2. Arduino Uno R4 WiFi
-------
Usage:

For the short video produced by @NiusRobotLab
**********************************************************************************************************/

#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_HMC5883_U.h>
#include <ITG3200.h>
#include <Kalman.h>
#include "config.h"

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(10001); // Assign a unique ID to this sensor at the same time
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(10002); // Assign a unique ID to this sensor at the same time
ITG3200 gyro;

float temperature;
bool isFirstData = true;
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
Kalman kalmanZ;
double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only
double compAngleX, compAngleY, compAngleZ; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter
double yaw; // Heading
uint32_t timer;

// See: http://www.freescale.com/files/sensors/doc/app_note/AN4248.pdf
void updateYaw(double magX, double magY, double magZ, double kalAngleX, double kalAngleY, double *yaw) {
	double _magX = magX * -1.0; // Invert axis - this it done here, as it should be done after the calibration
	double _magY = magY;
	double _magZ = magZ * -1.0;
	double rollAngle = kalAngleX * DEG_TO_RAD;
	double pitchAngle = kalAngleY * DEG_TO_RAD;
	//double Bfy = _magZ * sin(rollAngle) - _magY * cos(rollAngle);
	double Bfy = _magY * cos(rollAngle) - _magZ * sin(rollAngle);
	double Bfx = _magX * cos(pitchAngle) + _magY * sin(pitchAngle) * sin(rollAngle) + _magZ * sin(pitchAngle) * cos(rollAngle);
	*yaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;
	//yaw *= -1;
}

void setup() {
    Serial.begin(115200);
    /* accelerometer initialization*/
    if(!accel.begin()) {
        Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
        while(1);
    }
    Serial.println("Accelerometer ADXL345 found!");
    accel.setRange(ADXL345_RANGE_16_G); // ADXL345_RANGE_8_G, ADXL345_RANGE_4_G, ADXL345_RANGE_2_G
    if(!mag.begin()) {
        Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
        while(1);
    }
    Serial.println("Magnetometer HMC5883 found!");
    /* gyro initialization*/
    gyro.init();
    Serial.println("Gyroscope ITG3200 found!");
    Serial.print("Gyro performs zeroCalibrating...");
    gyro.zeroCalibrate(200, 10);//sample 200 times to calibrate and it will take 200*10ms
    Serial.println("Done!");
    delay(100);
}

void loop() {
    // get all readings
    sensors_event_t event;
    accel.getEvent(&event);
    mag.getEvent(&event);
    double accX=event.acceleration.x, accY=event.acceleration.y, accZ=event.acceleration.z;
    double magX=event.magnetic.x, magY=event.magnetic.y, magZ=event.magnetic.z;
    float posX, posY, posZ;
    gyro.getAngularVelocity(&posX,&posY,&posZ);
    temperature = gyro.getTemperature();
    // Kalman Filter and Complimentary Filter
    if (isFirstData) {
        isFirstData = false;
        #ifdef RESTRICT_PITCH // Eq. 25 and 26
            double roll  = atan2(accY, accZ) * RAD_TO_DEG;
            double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
        #else // Eq. 28 and 29
            double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
            double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
        #endif
        updateYaw(magX, magY, magZ, kalAngleX, kalAngleY, &yaw);
        kalmanX.setAngle(roll); // Set starting angle
        kalmanY.setAngle(pitch);
        kalmanZ.setAngle(yaw);
        gyroXangle = roll;
        gyroYangle = pitch;
        gyroZangle = yaw;
        compAngleX = roll;
        compAngleY = pitch;
        compAngleZ = yaw;
        timer = micros();
    }
    else {
        double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
        timer = micros();
        // #define RESTRICT_PITCH
        #ifdef GYRO_RAD_TO_DEG
            double gyroXrate = posX / 131.0; // Convert to deg/s, for rad/s
            double gyroYrate = posY / 131.0;
            double gyroZrate = posZ / 131.0;
        #else
            double gyroXrate = posX; // deg/s
            double gyroYrate = posY;
            double gyroZrate = posZ;
        #endif
        /* Yaw estimation */
		updateYaw(magX, magY, magZ, kalAngleX, kalAngleY, &yaw);
		// This fixes the transition problem when the yaw angle jumps between -180 and 180 degrees
		if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90)) {
			kalmanZ.setAngle(yaw);
			kalAngleZ = yaw;
		    } 
        else 
			kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt); // Calculate the angle using a Kalman filter
        /* Roll and Pitch estimation */
        #ifdef RESTRICT_PITCH // Eq. 25 and 26
            double roll  = atan2(accY, accZ) * RAD_TO_DEG;
            double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
        #else // Eq. 28 and 29
            double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
            double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
        #endif
        // #define RESTRICT_PITCH
        #ifdef RESTRICT_PITCH
            // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
            if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
                kalmanX.setAngle(roll);
                compAngleX = roll;
                kalAngleX = roll;
                gyroXangle = roll;
                }
            else
                kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
            if (abs(kalAngleX) > 90) gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
            kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
        #else
            // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
            if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
                kalmanY.setAngle(pitch);
                compAngleY = pitch;
                kalAngleY = pitch;
                gyroYangle = pitch;
                }
            else
                kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
            if (abs(kalAngleY) > 90) gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
            kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
        #endif
        // #define UNBIASED_GYRO_RATE
        #ifdef UNBIASED_GYRO_RATE
            gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
            gyroYangle += kalmanY.getRate() * dt;
            gyroZangle += kalmanZ.getRate() * dt;
        #else
            gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
            gyroYangle += gyroYrate * dt;
            gyroZangle += gyroZrate * dt;
        #endif
        // Calculate the angle using a Complimentary filter
        compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; 
        compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
        compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;
        // Reset the gyro angle when it has drifted too much
        if (gyroXangle < -180 || gyroXangle > 180) gyroXangle = kalAngleX;
        if (gyroYangle < -180 || gyroYangle > 180) gyroYangle = kalAngleY;
        if (gyroZangle < -180 || gyroZangle > 180) gyroZangle = kalAngleZ;
        /* Print Results */
        // #define DISP_RAW_ACC
        #ifdef DISP_RAW_ACC // Accelerometer raw data
            Serial.print("Accel-x:"); Serial.print(accX); Serial.print("\t");
            Serial.print("Accel-y:"); Serial.print(accY); Serial.print("\t");
            Serial.print("Accel-z:"); Serial.print(accZ); Serial.print("\t");
            Serial.print("\t");
        #endif
        // #define DISP_RAW_GYRO
        #ifdef DISP_RAW_GYRO // Gyro raw data
            Serial.print("Gyro-x:"); Serial.print(posX); Serial.print("\t");
            Serial.print("Gyro-y:"); Serial.print(posY); Serial.print("\t");
            Serial.print("Gyro-z:"); Serial.print(posZ); Serial.print("\t");
            Serial.print("\t");
        #endif
        // #define DISP_ROLL
        #ifdef DISP_ROLL // Roll - measurements and filtering results
            Serial.print("Roll-r:"); Serial.print(roll); Serial.print("\t");
            Serial.print("Roll-c:"); Serial.print(compAngleX); Serial.print("\t");
            Serial.print("Roll-k:"); Serial.print(kalAngleX); Serial.print("\t");
            Serial.print("\t");
        #endif
        // #define DDISP_PITCH
        #ifdef DISP_PITCH // Pitch - measurements and filtering results
            Serial.print("Pitch-r:"); Serial.print(pitch); Serial.print("\t");
            Serial.print("Pitch-c:"); Serial.print(compAngleY); Serial.print("\t");
            Serial.print("Pitch-k:"); Serial.print(kalAngleY); Serial.print("\t");
            Serial.print("\t");
        #endif
        // #define DISP_YAW
        #ifdef DISP_YAW // Yaw - measurements and filtering results
            Serial.print("Yaw-r:"); Serial.print(yaw); Serial.print("\t");
            Serial.print("Yaw-c:"); Serial.print(compAngleZ); Serial.print("\t");
            Serial.print("Yaw-k:"); Serial.print(kalAngleZ); Serial.print("\t");
            Serial.print("\t");
        #endif
        // #define DISP_GYRO_ANGLE
        #ifdef DISP_GYRO_ANGLE // Yaw - measurements and filtering results
            Serial.print("GyroX:"); Serial.print(gyroXangle); Serial.print("\t");
            Serial.print("GyroY:"); Serial.print(gyroYangle); Serial.print("\t");
            Serial.print("GyroZ:"); Serial.print(gyroZangle); Serial.print("\t");
        #endif
        // #define DISP_TEMP
        #ifdef DISP_TEMP //temperature
            // temperature = tempRaw / 340.0 + 36.53; //MPU6050 raw data
            Serial.print(temperature); Serial.print("\t");
        #endif
        // ready for next reading
        Serial.print("\r\n");
        delay(2);
    }
}