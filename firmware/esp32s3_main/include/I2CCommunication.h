#ifndef I2C_COMMUNICATION_H
#define I2C_COMMUNICATION_H

#include <Arduino.h>
#include <Wire.h> // Include the Wire library

// Pin definitions
#define SDA_PIN 5
#define SCL_PIN 6

// Enums for I2C command IDs
enum I2CCommandID
{
    I2C_GET_STATUS = 0,
    SET_STRENGTH = 1,
    SET_DEBUG = 0x0010
};

// Declaration of TwoWire instance for the custom I2C bus
extern TwoWire Wire_lcd;

// Function prototypes
void initI2C();
void receiveI2CEvent(int howMany);
void requestEvent();
void processI2CData();
void handleI2CCommand(uint16_t cmdid, const uint8_t *data, size_t dataSize);

// Specific command handlers
void i2cHandleGetStatus(const uint8_t *data, size_t dataSize);
void i2cHandleSetStrength(const uint8_t *data, size_t dataSize);
void i2cHandleSetDebug(const uint8_t *data, size_t dataSize);

// Command ID struct
struct I2C_CMDID
{
    uint16_t cmdid;
};

// Shared data structures
struct SharedStateData
{
    float voltage;
    float current;
    bool newData; // Flag to indicate new data is available
};
extern SharedStateData sharedStateData;

struct SharedCfgData
{
    bool newData; // Flag to indicate new data is available
    float target_force;
    float set_force;
};
extern SharedCfgData sharedCfgData;

struct ControlState
{
    uint8_t mode; // strength, row, winch or free
    uint8_t power_state;
    uint32_t error_state;
};
extern ControlState controlState;

struct I2C_SET_DEBUG
{
    uint16_t mask = 0;
    uint8_t mode = 0;
    uint8_t level = 0;
    bool toggle_switch = false;
};

struct I2C_GET_STATUS
{
    int8_t mode;
    int8_t new_cmdid_data_flags;
};

struct I2C_CMD_SET_STRENGTH
{
    uint16_t mask;
    float home_linear_position;
    float weight;
    float weight_max;
    float damping_percent;
    bool auto_charge;
    uint8_t dynamic_feedback_mode;
    uint8_t dynamic_feedback_amplitude;
};

struct I2C_CMD_GET_STRENGTH_LIVE
{
    float current_linear_velocity;
    float current_linear_position;
    float current_dynamic_force;
};

struct I2C_CMD_GET_STRENGTH_CFG
{
    bool new_payload_data_flag;
    int8_t new_cmdid_data_flags;
    float home_linear_position;
    float weight;
    float weight_max;
    float damping_percent;
    bool auto_charge;
    uint8_t dynamic_feedback_mode;
    uint8_t dynamic_feedback_amplitude;
};

struct I2C_CMD_SET_ROW
{
    uint16_t mask;
    float gear_factor;
    float drag_factor;
    bool motor_speed_lock;
    uint8_t dynamic_feedback_mode;
    uint8_t dynamic_feedback_amplitude;
};

struct I2C_CMD_GET_ROW_LIVE
{
    float current_speed;
    float current_linear_velocity;
    float current_linear_position;
    float current_dynamic_force;
};

struct I2C_CMD_GET_ROW_CFG
{
    bool new_payload_data_flag;
    int8_t new_cmdid_data_flags;
    float gear_factor;
    float drag_factor;
    bool motor_speed_lock;
    uint8_t dynamic_feedback_mode;
    uint8_t dynamic_feedback_amplitude;
};

// I2C command handler struct
struct I2CCommandHandler
{
    uint16_t cmdID;
    void (*handler)(const uint8_t *data, size_t dataSize);
    const char *name; // Optional: Command name or description
};
extern I2CCommandHandler i2cCommandHandlers[];

extern SemaphoreHandle_t mutex; // Declare mutex

#endif // I2C_COMMUNICATION_H
