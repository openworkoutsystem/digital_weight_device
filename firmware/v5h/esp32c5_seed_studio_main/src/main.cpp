// src/main.cpp
#include <Arduino.h>
#include <WiFi.h>
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
    initAccelerometer(); // BMI270 on LP-I2C (rear GPIO2/3 pads)
    initWiFi();
    // One-line proof of which firmware is running (shows on a late-opened
    // monitor via the first heartbeat too, but this pins the build)
    Serial.printf("[boot] esp32c5_main %s %s moteus_host=%d can=%s\n",
                  __DATE__, __TIME__, MOTEUS_HOST_ID,
                  getCANQuiet() ? "MUTED" : "live");
}

void loop()
{
    // readSerialData();     // Read data from the serial port
    // sendAggregatedData(); // Send dummy payload to the serial port
    handleWiFiClients();

    // 5 s heartbeat: the one periodic line on the serial monitor. Every
    // field is a link-health signal — a silent monitor is indistinguishable
    // from a dead board without this.
    //   wifi: IP when connected  |  can: MUTED until cal {"can_quiet":false}
    //   tx/drop/rx: CAN frames out/failed/in  |  i2c: display polls seen
    //   vbus/pos: last moteus telemetry/feedback  |  m: control mode
    static unsigned long hbMs = 0;
    if (millis() - hbMs >= 5000)
    {
        hbMs = millis();
        Serial.printf("[hb] up=%lus wifi=%s can=%s tx=%lu drop=%lu rx=%lu i2c=%lu/%lu vbus=%.1f pos=%.3f m=%u\n",
                      millis() / 1000UL,
                      WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "connecting",
                      getCANQuiet() ? "MUTED" : "live",
                      (unsigned long)getCANTxOkCount(),
                      (unsigned long)getCANTxDropCount(),
                      (unsigned long)getCANRxCount(),
                      (unsigned long)g_i2cPollCount,
                      (unsigned long)g_i2cRecvCount,
                      controlFeedback.voltage,
                      controlFeedback.m1_position,
                      (unsigned)controlState.mode);
    }

    // Keep the main loop highly responsive to incoming WiFi clients
    // to minimize command latency while still yielding to other tasks.
    delay(1);
}
