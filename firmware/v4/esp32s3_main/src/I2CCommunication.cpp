#include "I2CCommunication.h"
#include "ErrorHandling.h"
#include "Logging.h"
#include "CANCommunication.h"
#include <Wire.h>

// Definition of TwoWire instance for the custom I2C bus
TwoWire Wire_lcd = TwoWire(1);

// Shared state data
SharedStateData sharedStateData = {0};

// Shared configuration data
SharedCfgData sharedCfgData = {0};

// Control state
ControlState controlState = {0};

// I2C command handlers array
I2CCommandHandler i2cCommandHandlers[] = {
    {I2C_GET_STATUS, i2cHandleGetStatus, "Get Status"},
    {SET_STRENGTH, i2cHandleSetStrength, "Set Strength"},
    {SET_DEBUG, i2cHandleSetDebug, "Set Debug"}};

// Define the mutex
SemaphoreHandle_t mutex = xSemaphoreCreateMutex();

void initI2C()
{
    Wire_lcd.onReceive(receiveI2CEvent);
    Wire_lcd.onRequest(requestEvent);
    Wire_lcd.begin(8, SDA_PIN, SCL_PIN, 100000);
    logMessage(LOG_INFO, "I2C slave setup complete, address #8.");
    logMessage(LOG_INFO, "I2C slave event registered.");
}

void handleI2CCommand(uint16_t cmdid, const uint8_t *data, size_t dataSize)
{
    for (auto &handler : i2cCommandHandlers)
    {
        if (handler.cmdID == cmdid)
        {
            Serial.printf("Received command: %s\n", handler.name);
            handler.handler(data, dataSize);
            return;
        }
    }
    Serial.println("Unknown CMDID received.");
}

void receiveI2CEvent(int howMany)
{
    logMessage(LOG_DEBUG, "I2C slave event received.");
    if (howMany >= sizeof(I2C_CMDID))
    {
        logMessage(LOG_DEBUG, "Received I2C data being processed.");
        I2C_CMDID cmd;
        Wire_lcd.readBytes((char *)&cmd, sizeof(I2C_CMDID));

        int payloadSize = howMany - sizeof(I2C_CMDID);
        uint8_t payload[payloadSize];
        Wire_lcd.readBytes(payload, payloadSize);

        handleI2CCommand(cmd.cmdid, payload, payloadSize);
    }
    else
    {
        handleError(ERROR_I2C_RECEIVE, "I2C receive event failed, insufficient data received");
    }
}

void requestEvent()
{
    logMessage(LOG_DEBUG, "I2C slave event requested.");
    if (sharedStateData.newData)
    {
        if (xSemaphoreTake(mutex, 0) == pdTRUE)
        {
            Wire_lcd.write((uint8_t *)&sharedStateData, sizeof(SharedStateData));
            sharedStateData.newData = false;
            xSemaphoreGive(mutex);
        }
        else
        {
            handleError(ERROR_I2C_REQUEST, "Failed to take mutex for I2C request event");
        }
    }
}

void processI2CData()
{
    if (xSemaphoreTake(mutex, 0) == pdTRUE)
    {
        if (sharedCfgData.newData)
        {
            logMessagef(LOG_DEBUG, "Received weight: %f", sharedCfgData.target_force);
            sharedCfgData.newData = false;

            uint32_t torque_control_mode = 0x00000001;
            uint32_t velocity_control_mode = 0x00000002;
            uint32_t position_control_mode = 0x00000003;

            uint32_t idle_mode = 0x00000001;
            uint32_t closed_loop_mode = 0x00000008;

            uint32_t passthrough_input_mode = 0x00000001;
            uint32_t position_input_mode = 0x00000003;
            uint32_t ramptorque_input_mode = 0x00000006;

            float position = 0;
            sendMotorAbsolutePosition(CAN_NODEID_M1, position);

            if (controlState.mode == 0)
            {
                sendMotorMode(CAN_NODEID_M1, velocity_control_mode, passthrough_input_mode, closed_loop_mode);
                sendMotorMode(CAN_NODEID_M2, velocity_control_mode, passthrough_input_mode, closed_loop_mode);
                controlState.mode = 1;
                logMessage(LOG_INFO, "Motor mode set to position control");
            }
            else
            {
                sendMotorMode(CAN_NODEID_M1, velocity_control_mode, passthrough_input_mode, idle_mode);
                sendMotorMode(CAN_NODEID_M2, velocity_control_mode, passthrough_input_mode, idle_mode);
                controlState.mode = 0;
                logMessage(LOG_INFO, "Motor mode set to idle");
            }
        }
        xSemaphoreGive(mutex);
    }
    else
    {
        handleError(ERROR_I2C_PROCESS, "Failed to take mutex for processing I2C data");
    }
}

// Specific I2C command handlers
void i2cHandleGetStatus(const uint8_t *data, size_t dataSize)
{
    // Implementation for Get Status command
    Serial.println("Get Status command received but not implemented.");
}

void i2cHandleSetStrength(const uint8_t *data, size_t dataSize)
{
    // Implementation for Set Strength command
    Serial.println("Set Strength command received.");
    I2C_CMD_SET_STRENGTH cmd;
    memcpy(&cmd, data, sizeof(I2C_CMD_SET_STRENGTH));

    if (cmd.mask & 0x0001)
    {
        Serial.printf("Home Linear Position: %f\n", cmd.home_linear_position);
    }
    if (cmd.mask & 0x0002)
    {
        Serial.printf("Weight: %f\n", cmd.weight);
        // Update shared configuration data
        if (xSemaphoreTake(mutex, 0) == pdTRUE)
        {
            sharedCfgData.target_force = cmd.weight * 9.81;
            xSemaphoreGive(mutex);
        }
    }
    if (cmd.mask & 0x0004)
    {
        Serial.printf("Weight Max: %f\n", cmd.weight_max);
    }
    if (cmd.mask & 0x0008)
    {
        Serial.printf("Damping Percent: %f\n", cmd.damping_percent);
    }
    if (cmd.mask & 0x0010)
    {
        Serial.printf("Auto Charge: %d\n", cmd.auto_charge);
    }
    if (cmd.mask & 0x0020)
    {
        Serial.printf("Dynamic Feedback Mode: %d\n", cmd.dynamic_feedback_mode);
    }
    if (cmd.mask & 0x0040)
    {
        Serial.printf("Dynamic Feedback Amplitude: %d\n", cmd.dynamic_feedback_amplitude);
    }
}

void i2cHandleSetDebug(const uint8_t *data, size_t dataSize)
{
    // Implementation for Set Debug command
    Serial.println("Set Debug command received.");
    I2C_SET_DEBUG cmd;
    memcpy(&cmd, data, sizeof(I2C_SET_DEBUG));

    Serial.printf("Mode: %d\n", cmd.mode);
    Serial.printf("Level: %d\n", cmd.level);
    Serial.printf("Toggle Switch: %d\n", cmd.toggle_switch);

    if (xSemaphoreTake(mutex, 0) == pdTRUE)
    {
        sharedCfgData.newData = true;
        xSemaphoreGive(mutex);
    }
}
