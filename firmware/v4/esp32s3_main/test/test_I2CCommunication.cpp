#include <Arduino.h>
#include <unity.h>
#include "I2CCommunication.h"

void test_handleI2CCommand()
{
    uint16_t cmdid = GET_STATUS;
    uint8_t data[] = {0x01, 0x02, 0x03};
    size_t dataSize = sizeof(data);

    // Mock the function that should be called by handleI2CCommand
    handleI2CCommand(cmdid, data, dataSize);

    // Add assertions to verify the correct function was called with the correct parameters
}

void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_handleI2CCommand);
    UNITY_END();
}

void loop()
{
    // Nothing to do here
}
