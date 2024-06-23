// src/CANCommunication.cpp
#include "CANCommunication.h"
#include "ErrorHandling.h"
#include "Logging.h"
#include <driver/twai.h>

const twai_general_config_t twaiConfig = {
    .mode = TWAI_MODE_NORMAL,
    .tx_io = GPIO_NUM_7,
    .rx_io = GPIO_NUM_4,
    .clkout_io = (gpio_num_t)TWAI_IO_UNUSED,
    .bus_off_io = (gpio_num_t)TWAI_IO_UNUSED,
    .tx_queue_len = 5,
    .rx_queue_len = 5,
    .alerts_enabled = TWAI_ALERT_NONE,
    .clkout_divider = 0,
};

const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

void initCAN()
{
    if (twai_driver_install(&twaiConfig, &t_config, &f_config) != ESP_OK)
    {
        handleError(ERROR_CAN_TRANSMIT, "Failed to install TWAI driver");
        return;
    }
    if (twai_start() != ESP_OK)
    {
        handleError(ERROR_CAN_TRANSMIT, "Failed to start TWAI driver");
        return;
    }
    logMessage(LOG_INFO, "TWAI driver installed and started.");
}

twai_message_t createCANMessage(uint16_t nodeID, CANCommandID commandID, uint8_t data_length)
{
    twai_message_t message;
    message.identifier = (nodeID << 5) | commandID;
    message.flags = TWAI_MSG_FLAG_NONE;
    message.data_length_code = data_length;
    return message;
}

bool transmitCANMessage(twai_message_t &message)
{
    if (twai_transmit(&message, pdMS_TO_TICKS(1000)) != ESP_OK)
    {
        handleError(ERROR_CAN_TRANSMIT, "Failed to transmit CAN message");
        return false;
    }
    return true;
}

void sendMotorPVT(uint16_t nodeID, float position, int16_t vel_ff, int16_t torque_ff)
{
    twai_message_t message = createCANMessage(nodeID, SET_POS, 8);

    memcpy(&message.data[0], &position, sizeof(float));
    memcpy(&message.data[4], &vel_ff, sizeof(int16_t));
    memcpy(&message.data[6], &torque_ff, sizeof(int16_t));

    if (!transmitCANMessage(message))
    {
        handleError(ERROR_CAN_TRANSMIT, "Failed to send position command to ODrive");
    }
}

void sendMotorVelocity(uint16_t nodeID, float velocity)
{
    twai_message_t message = createCANMessage(nodeID, SET_VEL, 4);
    memcpy(&message.data[0], &velocity, sizeof(float));

    if (!transmitCANMessage(message))
    {
        handleError(ERROR_CAN_TRANSMIT, "Failed to send velocity command to ODrive");
    }
}

void sendMotorTorque(uint16_t nodeID, float torque)
{
    twai_message_t message = createCANMessage(nodeID, SET_TORQUE, 4);
    memcpy(&message.data[0], &torque, sizeof(float));

    if (!transmitCANMessage(message))
    {
        handleError(ERROR_CAN_TRANSMIT, "Failed to send torque command to ODrive");
    }
}

void sendMotorMode(uint16_t nodeID, uint32_t controller_mode, uint32_t input_mode, uint32_t axis_state)
{
    twai_message_t message = createCANMessage(nodeID, SET_MODE, 8);

    memcpy(&message.data[0], &controller_mode, sizeof(uint32_t));
    memcpy(&message.data[4], &input_mode, sizeof(uint32_t));

    if (!transmitCANMessage(message))
    {
        handleError(ERROR_CAN_TRANSMIT, "Failed to send mode command to ODrive");
    }

    message = createCANMessage(nodeID, SET_AXIS_STATE, 4);
    memcpy(&message.data[0], &axis_state, sizeof(uint32_t));

    if (!transmitCANMessage(message))
    {
        handleError(ERROR_CAN_TRANSMIT, "Failed to send axis state command to ODrive");
    }
}

void sendMotorRX(uint16_t nodeID, ODriveEndpointID endpointID, float data)
{
    twai_message_t message = createCANMessage(nodeID, SET_RXDO, 8);

    uint8_t operationCode = 1;
    message.data[0] = operationCode;
    message.data[1] = endpointID & 0xFF;
    message.data[2] = (endpointID >> 8) & 0xFF;
    memcpy(&message.data[4], &data, sizeof(data));

    if (!transmitCANMessage(message))
    {
        handleError(ERROR_CAN_TRANSMIT, "Failed to send RX command to ODrive");
    }
}

void sendMotorAbsolutePosition(uint16_t nodeID, float position)
{
    twai_message_t message = createCANMessage(nodeID, SET_ABSOLUTE_POS, 4);
    memcpy(&message.data[0], &position, sizeof(float));

    if (!transmitCANMessage(message))
    {
        handleError(ERROR_CAN_TRANSMIT, "Failed to send absolute position command to ODrive");
    }
}
