// src/Logging.cpp
#include "Logging.h"
#include <stdarg.h>

// Buffer size for log messages
#define LOG_BUFFER_SIZE 256

// Per-event DEBUG lines (I2C poll events etc.) fire multiple times a second
// and drown the serial monitor; the 5 s [hb] heartbeat in main.cpp carries
// those counts instead. Raise to LOG_DEBUG temporarily when chasing a
// specific event-level problem.
static const LogLevel MIN_LOG_LEVEL = LOG_INFO;

void logMessage(LogLevel level, const char *message)
{
    if (level < MIN_LOG_LEVEL)
    {
        return;
    }
    const char *levelStr = nullptr;
    switch (level)
    {
    case LOG_DEBUG:
        levelStr = "DEBUG";
        break;
    case LOG_INFO:
        levelStr = "INFO";
        break;
    case LOG_WARN:
        levelStr = "WARN";
        break;
    case LOG_ERROR:
        levelStr = "ERROR";
        break;
    default:
        levelStr = "UNKNOWN";
        break;
    }
    Serial.printf("[%s] %s\n", levelStr, message);
}

void logMessagef(LogLevel level, const char *format, ...)
{
    char buffer[LOG_BUFFER_SIZE];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    logMessage(level, buffer);
}
