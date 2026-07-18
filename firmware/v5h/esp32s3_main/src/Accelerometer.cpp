#include "Accelerometer.h"
#include "ErrorHandling.h"
#include "Logging.h"
#include "SharedData.h"
#include <Wire.h>

// MPU6050 I2C address (AD0 low = 0x68, AD0 high = 0x69)
#define MPU6050_ADDR 0x68

// MPU6050 register map (subset)
#define REG_SMPLRT_DIV 0x19
#define REG_CONFIG 0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_PWR_MGMT_1 0x6B
#define REG_ACCEL_XOUT_H 0x3B

// Scale factors for selected full-scale ranges
// Using ±2g for accel and ±500°/s for gyro
static const float ACCEL_LSB_PER_G = 16384.0f; // ±2g
static const float GYRO_LSB_PER_DPS = 65.5f;   // ±500°/s

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

static bool mpu6050WriteByte(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

static bool mpu6050ReadBytes(uint8_t startReg, uint8_t *buffer, size_t length)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(startReg);
    if (Wire.endTransmission(false) != 0) // restart for read
    {
        return false;
    }
    size_t received = Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)length);
    if (received != length)
    {
        return false;
    }
    for (size_t i = 0; i < length; i++)
    {
        buffer[i] = Wire.read();
    }
    return true;
}

static bool mpu6050Init()
{
    // Wake up device
    if (!mpu6050WriteByte(REG_PWR_MGMT_1, 0x00))
        return false;
    // Set sample rate: SampleRate = GyroOutputRate / (1 + SMPLRT_DIV)
    // GyroOutputRate default 8kHz -> set divider to 7 for 1kHz
    if (!mpu6050WriteByte(REG_SMPLRT_DIV, 0x07))
        return false;
    // CONFIG: DLPF set to 0x03 (~44Hz accel, 42Hz gyro bandwidth)
    if (!mpu6050WriteByte(REG_CONFIG, 0x03))
        return false;
    // Gyro config: ±500°/s (0x08)
    if (!mpu6050WriteByte(REG_GYRO_CONFIG, 0x08))
        return false;
    // Accel config: ±2g (0x00)
    if (!mpu6050WriteByte(REG_ACCEL_CONFIG, 0x00))
        return false;
    return true;
}

static bool mpu6050ReadScaled(float &ax, float &ay, float &az, float &gx, float &gy, float &gz)
{
    uint8_t raw[14];
    if (!mpu6050ReadBytes(REG_ACCEL_XOUT_H, raw, sizeof(raw)))
    {
        return false;
    }
    int16_t raX = (raw[0] << 8) | raw[1];
    int16_t raY = (raw[2] << 8) | raw[3];
    int16_t raZ = (raw[4] << 8) | raw[5];
    int16_t rgX = (raw[8] << 8) | raw[9];
    int16_t rgY = (raw[10] << 8) | raw[11];
    int16_t rgZ = (raw[12] << 8) | raw[13];

    ax = (float)raX / ACCEL_LSB_PER_G; // g
    ay = (float)raY / ACCEL_LSB_PER_G;
    az = (float)raZ / ACCEL_LSB_PER_G;
    gx = (float)rgX / GYRO_LSB_PER_DPS; // °/s
    gy = (float)rgY / GYRO_LSB_PER_DPS;
    gz = (float)rgZ / GYRO_LSB_PER_DPS;
    return true;
}

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
    if (!mpu6050Init())
    {
        handleError(ERROR_IMU_INIT, "MPU6050 init failed");
        while (true)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
    logMessage(LOG_INFO, "MPU6050 initialized on custom I2C bus.");

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
        float ax, ay, az, gx, gy, gz;
        if (!mpu6050ReadScaled(ax, ay, az, gx, gy, gz))
        {
            handleError(ERROR_I2C_PROCESS, "MPU6050 read failed");
            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }

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

        vTaskDelay(10 / portTICK_PERIOD_MS); // Delay for 5ms
    }
}
