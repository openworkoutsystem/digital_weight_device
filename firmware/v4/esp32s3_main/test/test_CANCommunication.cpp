#include <Arduino.h>
#include <unity.h>
#include "CANCommunication.h"

void test_createCANMessage()
{
    uint16_t nodeID = 0x01;
    CANCommandID commandID = SET_POS;
    uint8_t data_length = 8;

    twai_message_t message = createCANMessage(nodeID, commandID, data_length);

    TEST_ASSERT_EQUAL_HEX16((nodeID << 5) | commandID, message.identifier);
    TEST_ASSERT_EQUAL_UINT8(data_length, message.data_length_code);
}

void test_transmitCANMessage()
{
    // We will need to mock the twai_transmit function to properly test this
}

void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_createCANMessage);
    // RUN_TEST(test_transmitCANMessage);
    UNITY_END();
}

void loop()
{
    // Nothing to do here
}
