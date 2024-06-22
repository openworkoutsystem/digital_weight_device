#ifndef SERIAL_COMMUNICATION_H
#define SERIAL_COMMUNICATION_H

#include <Arduino.h>

void initSerial();
void readSerialData();
void sendDummyPayload();
void sendSerialData(const String &data);
void sendAggregatedData();

#endif // SERIAL_COMMUNICATION_H
