// src/ErrorHandling.cpp
#include "ErrorHandling.h"
#include "Logging.h"

void handleError(ErrorCode code, const char *message)
{
    logMessagef(LOG_ERROR, "Error [%d]: %s", code, message);
    // Additional error handling logic can be added here
}
