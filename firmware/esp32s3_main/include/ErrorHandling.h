// include/ErrorHandling.h
#ifndef ERROR_HANDLING_H
#define ERROR_HANDLING_H

#include <Arduino.h>

enum ErrorCode
{
    SUCCESS = 0,
    ERROR_CAN_TRANSMIT,
    ERROR_I2C_RECEIVE,
    ERROR_I2C_REQUEST,
    ERROR_I2C_PROCESS,
    ERROR_IMU_INIT
};

void handleError(ErrorCode code, const char *message);

#endif // ERROR_HANDLING_H
