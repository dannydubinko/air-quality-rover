/**
 * @file IMU-measurements.ino
 * @author Joshua Marshall (joshua.marshall@queensu.ca), Thomas Sears (thomas.sears@queensu.ca)
 * @brief Arduino program to read the LSM6DS3 IMU on the Arduino UNO WiFi Rev2.
 * @version 1.2
 * @author Daniel Dubinko (daniel.dubinko@icloud.com)
 * @date 2023-01-29
 *
 * @copyright Copyright (c) 2022
 *
 */

// Need this library installed to access the UNO Wifi Rev2 board's IMU
// For more details, look here: https://www.arduino.cc/reference/en/libraries/arduino_lsm6ds3/
#include <Arduino_LSM6DS3.h>

// Variables to store angular rates from the gyro [degrees/s]
float omega_x, omega_y, omega_z;

// Variables to store accelerations [g's]
float a_x, a_y, a_z;

// Variables to store sample rates from sensor [Hz]
float a_f, g_f;

float ox_bias, oy_bias, oz_bias;

void setup()
{
    // Open the serial port at 115200 bps
    Serial.begin(115200);

    // Wait for serial connection before starting
    while (!Serial)
    {
        delay(10);
    }

    Serial.println();

    // Check that the board is initialized
    if (!IMU.begin())
    {
        // Print an error message if the IMU is not ready
        Serial.print("Failed to initialize IMU :(");
        Serial.print("\n");
        while (1)
        {
            delay(10);
        }
    }

    // Read the sample rate of the accelerometer and gyroscope
    a_f = IMU.accelerationSampleRate();
    g_f = IMU.gyroscopeSampleRate();

    // Print these values to the serial window
    Serial.print("Accelerometer sample rate: ");
    Serial.println(a_f);
    Serial.print("Gyroscope sample rate: ");
    Serial.println(g_f);

    Serial.print("Wait! Calculating Gyro bias.");
    calculate_gyro_bias();
    Serial.print("Done. Starting program.");
}

void loop()
{
    // Timing in the loop is controlled by the IMU reporting when
    // it is ready for another measurement.
    // The accelerometer and gyroscope output at the same rate and
    // will give us their measurements at a steady frequency.

    // Read from the accelerometer
    if (IMU.accelerationAvailable())
    {
        IMU.readAcceleration(a_x, a_y, a_z);
        Serial.print("Acceleration x,y,z");
        /// Print the accelerometer measurements to the Serial Monitor
        Serial.print(a_x);
        Serial.print("\t");
        Serial.print(a_y);
        Serial.print("\t");
        Serial.print(a_z);
        Serial.print(" g\t\t");
    }

    // Read from the gyroscope
    if (IMU.gyroscopeAvailable())
    {
        IMU.readGyroscope(omega_x, omega_y, omega_z);
        Serial.print("Gyro x,y,z");
        omega_x -= ox_bias;
        omega_y -= oy_bias;
        omega_z -= oz_bias;

        // Print the gyroscope measurements to the Serial Monitor
        Serial.print(omega_x);
        Serial.print("\t");
        Serial.print(omega_y);
        Serial.print("\t");
        Serial.print(omega_z);
        Serial.print(" deg/s\n");

    }

    delay(1000);
}

void calculate_gyro_bias(){
    float omega_x_[10];
    float omega_y_[10];
    float omega_z_[10];

    for(int reading=0;reading<10;reading++){
        if (IMU.gyroscopeAvailable())
        {
            IMU.readGyroscope(omega_x, omega_y, omega_z);
            omega_x_[reading] = omega_x;
            omega_y_[reading] = omega_y;
            omega_z_[reading] = omega_z;
        }
        delay(10);
    }

    ox_bias = sum_arr(omega_x_) / 10;
    oy_bias = sum_arr(omega_y_) / 10;
    oz_bias = sum_arr(omega_z_) / 10;
}

float sum_arr(float *arr){
    float sum = 0;
    for(int i=0;i<sizeof(arr);i++)
    {
        sum += arr[i];
    }

    return sum;
}