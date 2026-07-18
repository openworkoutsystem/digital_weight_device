#include "Accelerometer.h"
#include "ErrorHandling.h"
#include "Logging.h"
#include "SharedData.h"
#include <Wire.h>
#include "SparkFun_BMI270_Arduino_Library.h"

// SparkFun Micro 6DoF BMI270 breakout, default address 0x68, on the LP-I2C
// bus (Wire1). The breakout carries its own 2.2k pull-ups. beginI2C() must
// upload Bosch's mandatory 8 KB config blob after every power-on, so init
// takes a couple hundred ms — it runs inside this task, never blocking boot.
static BMI270 imu;

char report[96];

const int numReadings = 50;
float accelXReadings[numReadings];
float accelYReadings[numReadings];
float accelZReadings[numReadings];
float gyroXReadings[numReadings];
float gyroYReadings[numReadings];
float gyroZReadings[numReadings];
int readIndex = 0;
float totalAccelX = 0;
float totalAccelY = 0;
float totalAccelZ = 0;
float totalGyroX = 0;
float totalGyroY = 0;
float totalGyroZ = 0;

void initAccelerometer()
{
    // Single-core C5: no core pinning; low priority — telemetry-grade data,
    // must never contend with the control tick.
    xTaskCreate(
        AccelerometerTask,   // Task function
        "AccelerometerTask", // Name of the task
        10000,               // Stack size in words
        NULL,                // Task input parameter
        1,                   // Priority of the task
        NULL);               // Task handle
}

void AccelerometerTask(void *parameter)
{
    // Wire1 = bus 1 = the LP-I2C controller: hardware master fixed to
    // GPIO2/3 (rear MTMS/MTDI pads). If this begin/init fails, fall back
    // per MIGRATION_PLAN.md (IDF LP-I2C driver, or bit-bang on the same
    // wiring) — the pins are ordinary GPIOs after boot either way.
    if (!Wire1.begin(SDA_PIN_ACC, SCL_PIN_ACC, 400000))
    {
        handleError(ERROR_IMU_INIT, "Wire1 (LP-I2C) begin failed on GPIO2/3");
        while (true)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }

    int attempts = 0;
    while (imu.beginI2C(BMI2_I2C_PRIM_ADDR, Wire1) != BMI2_OK)
    {
        attempts++;
        if (attempts >= 5)
        {
            handleError(ERROR_IMU_INIT, "BMI270 init failed (0x68 on LP-I2C)");
            while (true)
            {
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    logMessage(LOG_INFO, "BMI270 initialized on LP-I2C (GPIO2/3).");

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
        // Library defaults: ±8 g accel, ±2000 dps gyro, 100 Hz ODR — plenty
        // for the smoothed telemetry this feeds. Values arrive already
        // scaled to g and deg/s.
        if (imu.getSensorData() != BMI2_OK)
        {
            handleError(ERROR_I2C_PROCESS, "BMI270 read failed");
            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }
        float ax = imu.data.accelX;
        float ay = imu.data.accelY;
        float az = imu.data.accelZ;
        float gx = imu.data.gyroX;
        float gy = imu.data.gyroY;
        float gz = imu.data.gyroZ;

        // Remove last values from totals
        totalAccelX -= accelXReadings[readIndex];
        totalAccelY -= accelYReadings[readIndex];
        totalAccelZ -= accelZReadings[readIndex];
        totalGyroX -= gyroXReadings[readIndex];
        totalGyroY -= gyroYReadings[readIndex];
        totalGyroZ -= gyroZReadings[readIndex];

        // Store new readings
        accelXReadings[readIndex] = ax;
        accelYReadings[readIndex] = ay;
        accelZReadings[readIndex] = az;
        gyroXReadings[readIndex] = gx;
        gyroYReadings[readIndex] = gy;
        gyroZReadings[readIndex] = gz;

        // Add to totals
        totalAccelX += accelXReadings[readIndex];
        totalAccelY += accelYReadings[readIndex];
        totalAccelZ += accelZReadings[readIndex];
        totalGyroX += gyroXReadings[readIndex];
        totalGyroY += gyroYReadings[readIndex];
        totalGyroZ += gyroZReadings[readIndex];

        // Advance to the next position in the array
        readIndex = readIndex + 1;

        // If we're at the end of the array, wrap around to the beginning
        if (readIndex >= numReadings)
        {
            readIndex = 0;
        }

        // Calculate the average
        float averageAccelX = totalAccelX / numReadings;
        float averageAccelY = totalAccelY / numReadings;
        float averageAccelZ = totalAccelZ / numReadings;
        float averageGyroX = totalGyroX / numReadings;
        float averageGyroY = totalGyroY / numReadings;
        float averageGyroZ = totalGyroZ / numReadings;

        snprintf(report, sizeof(report), "A(g): %0.2f %0.2f %0.2f  G(dps): %0.1f %0.1f %0.1f",
             averageAccelX, averageAccelY, averageAccelZ,
             averageGyroX, averageGyroY, averageGyroZ);
        // logMessage(LOG_DEBUG, report);

        // Update shared data with the averaged values
        updateAccelerometerData(averageAccelX, averageAccelY, averageAccelZ);
        updateGyroData(averageGyroX, averageGyroY, averageGyroZ);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
