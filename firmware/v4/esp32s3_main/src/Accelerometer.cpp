#include "Accelerometer.h"
#include "ErrorHandling.h"
#include "Logging.h"
#include "SharedData.h" // Include the shared data header
#include <Wire.h>
#include <LSM6.h>

LSM6 imu;
char report[80]; // Define the report buffer

const int numReadings = 50;
int accelXReadings[numReadings];
int accelYReadings[numReadings];
int accelZReadings[numReadings];
int gyroXReadings[numReadings];
int gyroYReadings[numReadings];
int gyroZReadings[numReadings];
int readIndex = 0;
int totalAccelX = 0;
int totalAccelY = 0;
int totalAccelZ = 0;
int totalGyroX = 0;
int totalGyroY = 0;
int totalGyroZ = 0;

void initAccelerometer()
{
    xTaskCreatePinnedToCore(
        AccelerometerTask,   // Task function
        "AccelerometerTask", // Name of the task
        10000,               // Stack size in words
        NULL,                // Task input parameter
        1,                   // Priority of the task
        NULL,                // Task handle
        1);                  // Core where the task should run
}

void AccelerometerTask(void *parameter)
{
    Wire.begin(SDA_PIN_ACC, SCL_PIN_ACC);

    if (!imu.init())
    {
        handleError(ERROR_IMU_INIT, "Failed to detect and initialize IMU!");
        while (1)
            ;
    }
    imu.enableDefault();

    logMessage(LOG_INFO, "Accelerometer initialized on custom I2C bus.");

    // Initialize all readings to zero
    for (int i = 0; i < numReadings; i++)
    {
        accelXReadings[i] = 0;
        accelYReadings[i] = 0;
        accelZReadings[i] = 0;
        gyroXReadings[i] = 0;
        gyroYReadings[i] = 0;
        gyroZReadings[i] = 0;
    }

    while (true)
    {
        imu.read();

        // Subtract the last reading from the total
        totalAccelX = totalAccelX - accelXReadings[readIndex];
        totalAccelY = totalAccelY - accelYReadings[readIndex];
        totalAccelZ = totalAccelZ - accelZReadings[readIndex];
        totalGyroX = totalGyroX - gyroXReadings[readIndex];
        totalGyroY = totalGyroY - gyroYReadings[readIndex];
        totalGyroZ = totalGyroZ - gyroZReadings[readIndex];

        // Read new values
        accelXReadings[readIndex] = imu.a.x;
        accelYReadings[readIndex] = imu.a.y;
        accelZReadings[readIndex] = imu.a.z;
        gyroXReadings[readIndex] = imu.g.x;
        gyroYReadings[readIndex] = imu.g.y;
        gyroZReadings[readIndex] = imu.g.z;

        // Add the new reading to the total
        totalAccelX = totalAccelX + accelXReadings[readIndex];
        totalAccelY = totalAccelY + accelYReadings[readIndex];
        totalAccelZ = totalAccelZ + accelZReadings[readIndex];
        totalGyroX = totalGyroX + gyroXReadings[readIndex];
        totalGyroY = totalGyroY + gyroYReadings[readIndex];
        totalGyroZ = totalGyroZ + gyroZReadings[readIndex];

        // Advance to the next position in the array
        readIndex = readIndex + 1;

        // If we're at the end of the array, wrap around to the beginning
        if (readIndex >= numReadings)
        {
            readIndex = 0;
        }

        // Calculate the average
        int averageAccelX = totalAccelX / numReadings;
        int averageAccelY = totalAccelY / numReadings;
        int averageAccelZ = totalAccelZ / numReadings;
        int averageGyroX = totalGyroX / numReadings;
        int averageGyroY = totalGyroY / numReadings;
        int averageGyroZ = totalGyroZ / numReadings;

        snprintf(report, sizeof(report), "A: %6d %6d %6d    G: %6d %6d %6d",
                 averageAccelX, averageAccelY, averageAccelZ,
                 averageGyroX, averageGyroY, averageGyroZ);
        // logMessage(LOG_DEBUG, report);

        // Update shared data with the averaged values
        updateAccelerometerData(averageAccelX, averageAccelY, averageAccelZ);
        updateGyroData(averageGyroX, averageGyroY, averageGyroZ);

        vTaskDelay(10 / portTICK_PERIOD_MS); // Delay for 5ms
    }
}
