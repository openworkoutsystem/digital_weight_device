// include/CANCommunication.h
#ifndef CAN_COMMUNICATION_H
#define CAN_COMMUNICATION_H

#include <Arduino.h>
#include <driver/twai.h> // Include the necessary header for twai_message_t

// Enums for CAN command IDs
enum CANCommandID
{
    SET_MODE = 0x00B,
    SET_POS = 0x00C,
    SET_VEL = 0x00D,
    SET_TORQUE = 0x00E,
    SET_AXIS_STATE = 0x007,
    GET_VBUS = 0x017,
    GET_POSVEL = 0x009,
    GET_MOTORERR = 0x003,
    CAN_GET_STATUS = 0x001, // Renamed to avoid conflict
    SET_ABSOLUTE_POS = 0x019,
    SET_RXDO = 0x004
};

// Enums for ODrive endpoint IDs
enum ODriveEndpointID
{
    TORQUE_SOFT_MIN_M1 = 0x0101,
    TORQUE_SOFT_MAX_M1 = 0x0102,
    TORQUE_SOFT_MIN_M2 = 0x00FE,
    TORQUE_SOFT_MAX_M2 = 0x00FF
};

// Define CAN node IDs
#define CAN_NODEID_M1 0x000
#define CAN_NODEID_M2 0x001

void initCAN();
void sendMotorPVT(uint16_t nodeID, float position, int16_t vel_ff, int16_t torque_ff);
void sendMotorVelocity(uint16_t nodeID, float velocity);
void sendMotorTorque(uint16_t nodeID, float torque);
void sendMotorMode(uint16_t nodeID, uint32_t controller_mode, uint32_t input_mode, uint32_t axis_state);
void sendMotorRX(uint16_t nodeID, ODriveEndpointID endpointID, float data);
void sendMotorAbsolutePosition(uint16_t nodeID, float position);

// Helper functions
twai_message_t createCANMessage(uint16_t nodeID, CANCommandID commandID, uint8_t data_length);
bool transmitCANMessage(twai_message_t &message);

#endif // CAN_COMMUNICATION_H
