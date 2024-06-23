// src/main.cpp
#include <Arduino.h>
#include "CANCommunication.h"
#include "I2CCommunication.h"
#include "MotorControl.h"
#include "Accelerometer.h"
#include "SerialCommunication.h"
#include "SharedData.h"

void setup()
{
    initSharedData();
    initSerial();
    initCAN();
    initMotorControl();
    initI2C();
    initAccelerometer();
}

void loop()
{
    readSerialData();     // Read data from the serial port
    sendAggregatedData(); // Send dummy payload to the serial port
    delay(200);
}