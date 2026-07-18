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
    SET_RXDO = 0x004,
    GET_TXSDO = 0x005, // ODrive reply to an RxSdo read
    SET_LIMITS = 0x00F // Set_Limits: velocity_limit + current_limit
};

// Enums for ODrive endpoint IDs.
// IMPORTANT: endpoint IDs are firmware-version-specific. These values are
// verified against firmware/v5l/odrive_s1/flat_endpoints.json (fw 0.6.12).
// If the ODrive firmware is ever updated again, re-check that file — a
// stale ID fails silently.
enum ODriveEndpointID
{
    TORQUE_SOFT_MIN_M1 = 304, // axis0.config.torque_soft_min
    TORQUE_SOFT_MAX_M1 = 305  // axis0.config.torque_soft_max
};

// Define CAN node IDs (single-motor machine: M1 only)
#define CAN_NODEID_M1 0x000

void initCAN();
void sendMotorPVT(uint16_t nodeID, float position, int16_t vel_ff, int16_t torque_ff);
void sendMotorVelocity(uint16_t nodeID, float velocity);
void sendMotorTorque(uint16_t nodeID, float torque);
void sendMotorMode(uint16_t nodeID, uint32_t controller_mode, uint32_t input_mode, uint32_t axis_state);
void sendMotorRX(uint16_t nodeID, ODriveEndpointID endpointID, float data);
// Request a readback of an endpoint; the ODrive answers with a TxSdo frame
// (GET_TXSDO) carrying the value — used to verify writes actually land.
void sendMotorRXRead(uint16_t nodeID, ODriveEndpointID endpointID);
// Set_Limits (0x00F): velocity limit (turns/s) and current limit (A). The
// current limit is the torque limit: torque = current * torque_constant.
void sendMotorLimits(uint16_t nodeID, float velLimit, float currentLimit);
void sendMotorAbsolutePosition(uint16_t nodeID, float position);

// Helper functions
twai_message_t createCANMessage(uint16_t nodeID, CANCommandID commandID, uint8_t data_length);
bool transmitCANMessage(twai_message_t &message);

#endif // CAN_COMMUNICATION_H
