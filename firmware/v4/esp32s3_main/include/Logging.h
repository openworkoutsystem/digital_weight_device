// include/Logging.h
#ifndef LOGGING_H
#define LOGGING_H

#include <Arduino.h>

enum LogLevel
{
    LOG_DEBUG,
    LOG_INFO,
    LOG_WARN,
    LOG_ERROR
};

void logMessage(LogLevel level, const char *message);
void logMessagef(LogLevel level, const char *format, ...);

#endif // LOGGING_H
