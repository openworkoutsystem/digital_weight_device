// src/main.cpp
#include <Arduino.h>
#include "CANCommunication.h"
#include "I2CCommunication.h"
#include "MotorControl.h"
#include "Accelerometer.h"
#include "SerialCommunication.h"
#include "SharedData.h"
#include "WiFiCommunication.h"

void setup()
{
    initSharedData();
    initSerial();
    initCAN();
    initMotorControl();
    initI2C();
    initAccelerometer();
    initWiFi();
}

void loop()
{
    // readSerialData();     // Read data from the serial port
    // sendAggregatedData(); // Send dummy payload to the serial port
    handleWiFiClients();
    // Keep the main loop highly responsive to incoming WiFi clients
    // to minimize command latency while still yielding to other tasks.
    delay(1);
}