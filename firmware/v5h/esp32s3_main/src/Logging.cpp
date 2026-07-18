// src/Logging.cpp
#include "Logging.h"
#include <stdarg.h>

// Buffer size for log messages
#define LOG_BUFFER_SIZE 256

void logMessage(LogLevel level, const char *message)
{
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
