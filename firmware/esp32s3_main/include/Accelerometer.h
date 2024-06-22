// include/Accelerometer.h
#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <Arduino.h>

#define SDA_PIN_ACC 9 // Define the SDA pin for the accelerometer
#define SCL_PIN_ACC 8 // Define the SCL pin for the accelerometer

void initAccelerometer();
void AccelerometerTask(void *parameter);

#endif // ACCELEROMETER_H
