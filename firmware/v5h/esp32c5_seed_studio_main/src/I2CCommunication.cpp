#include "I2CCommunication.h"
#include "ErrorHandling.h"
#include "Logging.h"
#include "MotorControl.h"
#include "SharedData.h"
#include <Wire.h>

// ROLE FLIP: the C5 is the I2C MASTER of the display link (see header).
// HP I2C0 = the Arduino global `Wire`; the alias keeps old call sites.
TwoWire &Wire_lcd = Wire;

// Shared state data
SharedStateData sharedStateData = {0};

// Shared configuration data
SharedCfgData sharedCfgData = {0};

// Control state
ControlState controlState = {0};

// Pending actions to be handled in the control task
volatile bool pendingApplyIdle = false;
volatile bool pendingApplyStrength = false;

// I2C command handlers array
I2CCommandHandler i2cCommandHandlers[] = {
    {I2C_GET_STATUS, i2cHandleGetStatus, "Get Status"},
    {SET_STRENGTH, i2cHandleSetStrength, "Set Strength"},
    {SET_ROW, i2cHandleSetRow, "Set Row"},
    {SET_DEBUG, i2cHandleSetDebug, "Set Debug"}};

// Define the mutex
SemaphoreHandle_t mutex = xSemaphoreCreateMutex();

volatile uint32_t g_i2cPollCount = 0;
volatile uint32_t g_i2cRecvCount = 0;

static uint8_t additiveChecksum(const uint8_t *p, size_t n)
{
    uint8_t sum = 0xA5;
    for (size_t i = 0; i < n; i++)
    {
        sum += p[i];
    }
    return sum;
}

// Push the status frame to the display (unchanged 32-byte SharedStateData,
// same seq/checksum trailer the display has always validated).
static void pushStatusFrame()
{
    static SharedStateData lastGood = {0};
    if (xSemaphoreTake(mutex, 0) == pdTRUE)
    {
        lastGood = sharedStateData;
        sharedStateData.newData = false;
        xSemaphoreGive(mutex);
    }
    static uint8_t seq = 0;
    lastGood.seq = ++seq;
    lastGood.checksum = additiveChecksum((const uint8_t *)&lastGood,
                                         offsetof(SharedStateData, checksum));

    Wire_lcd.beginTransmission(8);
    Wire_lcd.write((const uint8_t *)&lastGood, sizeof(SharedStateData));
    uint8_t status = Wire_lcd.endTransmission();
    if (status == 0)
    {
        g_i2cPollCount = g_i2cPollCount + 1;
    }
    else
    {
        static unsigned long lastLogMs = 0;
        if (millis() - lastLogMs > 5000)
        {
            lastLogMs = millis();
            logMessagef(LOG_WARN, "display status push failed (I2C status %u)", status);
        }
    }
}

// Poll the display's command mailbox; dispatch fresh commands into the same
// handlers the old slave path used.
static void pollCommandMailbox()
{
    static uint8_t lastSeqSeen = 0xFF;

    size_t got = Wire_lcd.requestFrom((uint8_t)8, (uint8_t)sizeof(I2C_CMD_MAILBOX));
    if (got != sizeof(I2C_CMD_MAILBOX))
    {
        while (Wire_lcd.available())
        {
            Wire_lcd.read(); // drain a short/garbled response
        }
        return;
    }
    I2C_CMD_MAILBOX mb;
    Wire_lcd.readBytes((uint8_t *)&mb, sizeof(mb));
    if (mb.checksum != additiveChecksum((const uint8_t *)&mb, offsetof(I2C_CMD_MAILBOX, checksum)))
    {
        return; // corrupt poll: the mailbox persists, next poll retries
    }
    if (!mb.has_cmd || mb.seq == lastSeqSeen)
    {
        return; // empty, or already consumed
    }
    lastSeqSeen = mb.seq;
    g_i2cRecvCount = g_i2cRecvCount + 1;
    handleI2CCommand(mb.cmdid, mb.payload, sizeof(mb.payload));
}

// 10 Hz link task: push status, then poll for a command. Both transactions
// together are ~2 ms of bus time at 400 kHz.
static void I2CLinkTask(void *parameter)
{
    (void)parameter;
    for (;;)
    {
        pushStatusFrame();
        pollCommandMailbox();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void initI2C()
{
    // Wiring probe before the driver claims the pins: an idle I2C bus reads
    // HIGH on both lines. 0/0 = wires not connected / shorted.
    pinMode(SDA_PIN, INPUT);
    pinMode(SCL_PIN, INPUT);
    delay(2);
    logMessagef(LOG_INFO, "I2C wiring probe: SDA(23)=%d SCL(24)=%d (want 1/1 = bus idle-high)",
                digitalRead(SDA_PIN), digitalRead(SCL_PIN));

    if (!Wire_lcd.begin(SDA_PIN, SCL_PIN, 400000))
    {
        handleError(ERROR_I2C_RECEIVE, "I2C MASTER INIT FAILED (SDA 23, SCL 24) — display link dead");
        return;
    }
    Wire_lcd.setTimeOut(20);
    logMessage(LOG_INFO, "I2C master up: display link on SDA 23 (D4), SCL 24 (D5), slave addr 8.");

    xTaskCreate(
        I2CLinkTask,   // Task function
        "I2CLinkTask", // Name of the task
        4096,          // Stack size
        NULL,          // Parameter
        1,             // Low priority — telemetry-grade link
        NULL);         // Handle
}

void processI2CData()
{
    if (xSemaphoreTake(mutex, 0) == pdTRUE)
    {
        if (sharedCfgData.newData)
        {
            logMessagef(LOG_DEBUG, "Received weight: %f", sharedCfgData.target_force);
            sharedCfgData.newData = false;

            // Debug toggle (SET_DEBUG): flip the axis between streaming and
            // stopped; the motor control task owns all CAN traffic.
            if (controlState.mode == 0)
            {
                controlState.mode = 1;
                pendingApplyStrength = true;
                logMessage(LOG_INFO, "Motor mode set to position control");
            }
            else
            {
                controlState.mode = 0;
                pendingApplyIdle = true;
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
        // 0 = idle (device must not move at all); >= 1 = active strength
        // mode. Enqueue for the motor control task — same non-blocking,
        // coalesced path the WS/HTTP handlers use, so all mode changes and
        // CAN traffic stay centralized there.
        CommandMsg msg = {};
        if (cmd.weight <= 0.0f)
        {
            msg.type = CMD_IDLE;
            msg.weight_lb = 0.0f;
            if (controlState.mode != 0)
            {
                updateModeData("off");
                updatePulseData("off", 0, 0, 0);
                updateRowData("off", 0, 0, 0);
            }
        }
        else
        {
            // Screen value passes straight through: screen-lb IS the
            // canonical pipeline unit (see CommandMsg.weight_lb).
            msg.type = CMD_STRENGTH;
            msg.weight_lb = cmd.weight;
            updateModeData("strength");
        }
        if (g_cmdQueue == nullptr || xQueueSend(g_cmdQueue, &msg, 0) != pdTRUE)
        {
            handleError(ERROR_I2C_RECEIVE, "Command queue unavailable, weight update dropped");
        }
    }
    if (cmd.mask & 0x0004)
    {
        Serial.printf("Weight Max: %f\n", cmd.weight_max);
    }
    if (cmd.mask & 0x0008)
    {
        Serial.printf("Concentric Percent: %f\n", cmd.concentric_pct);
        setConcentric(cmd.concentric_pct, -1.0f, -1.0f);
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

void i2cHandleSetRow(const uint8_t *data, size_t dataSize)
{
    Serial.println("Set Row command received.");
    I2C_CMD_SET_ROW cmd;
    memcpy(&cmd, data, sizeof(I2C_CMD_SET_ROW));

    // Same non-blocking queue path as weight commands: the motor control
    // task owns the row model and all mode transitions.
    CommandMsg msg = {};
    msg.type = CMD_ROW;
    msg.row_gear = (cmd.mask & 0x0001) ? cmd.gear : 0;
    msg.row_drag = (cmd.mask & 0x0002) ? cmd.drag : 0;
    msg.row_enable = (cmd.mask & 0x0004) ? cmd.row_enable : 0xFF;
    Serial.printf("Row: enable=%d gear=%d drag=%d\n",
                  (int)msg.row_enable, (int)msg.row_gear, (int)msg.row_drag);
    if (g_cmdQueue == nullptr || xQueueSend(g_cmdQueue, &msg, 0) != pdTRUE)
    {
        handleError(ERROR_I2C_RECEIVE, "Command queue unavailable, row update dropped");
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
