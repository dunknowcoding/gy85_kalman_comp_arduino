/**********************************************************
MIT license - 1/12/2026

Macros for GY-85 demo.
------------------------
Soucecode Dependencies:

    gy85_kalman_comp_arduino.ino
-------
Usage:

For the short video produced by @NiusRobotLab
**********************************************************
/* Use these macros to meet your needs */

// Functioning settings
// #define GYRO_RAD_TO_DEG // Convert to deg/s, for rad/s
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead
// #define UNBIASED_GYRO_RATE // Calculate gyro angle using the unbiased rate

// Serial display settings, at lease printing one item
// #define DISP_RAW_ACC // print accelerometer raw data
// #define DISP_RAW_GYRO // print gyroscope raw data
#define DISP_ROLL // Roll - measurements and filtering results
#define DISP_PITCH // Pitch - measurements and filtering results
// #define DISP_YAW // Yaw - measurements and filtering results
// #define DISP_GYRO_ANGLE // Display Gyro incrementals, defined by <UNBIASED_GYRO_RATE>
// #define DISP_TEMP // Gyro temperature display