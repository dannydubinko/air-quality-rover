**
 * @file c02-measurement-scd30.ino
 * @author Thomas Sears (thomas.sears@queensu.ca)
 * @brief Basic functions of reading CO2 measurements from the SCD30 sensor
 * @version 1.0
 * @date 2022-12-05
 *
 * @copyright Copyright (c) 2022
 *
 */

// Need this library installed to read from the SCD30
// For more details, look here: https://github.com/adafruit/Adafruit_SCD30
// and https://adafruit.github.io/Adafruit_SCD30/html/index.html
#include <Adafruit_SCD30.h>

Adafruit_SCD30 scd;

void setup()
{
    // Open the serial port at 115200 bps
    Serial.begin(115200);

    // Wait for serial connection before starting
    while (!Serial)
    {
        delay(10);
    }

    Serial.println("__SCD30 demo__");

    // Initialize the sensor
    if (!scd.begin())
    {
        Serial.println("Sensor not found :(");
        while (1)
        {
            delay(10); // This will stay here forever if a sensor isn't found
        }
    }

    // Set the measurement interval [2-1800 s]
    scd.setMeasurementInterval(2);

    // Read the measurement interval [s]
    Serial.print("Measurement Interval: ");
    Serial.print(scd.getMeasurementInterval());
    Serial.println(" seconds");
}

void loop()
{
    // The SCD30 will report when it is ready. We can poll that function until it is true.
    if (scd.dataReady())
    {
        // The library also has a handy check if data can't be read
        if (!scd.read())
        {
            Serial.println("Error reading sensor data");
            return;
        }

        Serial.print("Temp: ");
        Serial.print(scd.temperature);
        Serial.print(" dC\t");

        Serial.print("RH: ");
        Serial.print(scd.relative_humidity);
        Serial.print(" %\t");

        Serial.print("CO2: ");
        Serial.print(scd.CO2, 3);
        Serial.println(" ppm");
    }
}
