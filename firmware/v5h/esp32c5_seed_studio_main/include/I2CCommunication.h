#ifndef I2C_COMMUNICATION_H
#define I2C_COMMUNICATION_H

#include <Arduino.h>
#include <Wire.h> // Include the Wire library

// Pin definitions — XIAO ESP32-C5 pads D4/D5, the same physical pads the S3
// harness used. This is the C5's ONLY high-performance I2C controller, so it
// stays dedicated to the display-slave role (the IMU lives on the LP-I2C).
#define SDA_PIN 23
#define SCL_PIN 24

// Enums for I2C command IDs
enum I2CCommandID
{
    I2C_GET_STATUS = 0,
    SET_STRENGTH = 1,
    SET_ROW = 3,
    SET_DEBUG = 0x0010
};

// ROLE FLIP (2026-07-13): the C5 is the bus MASTER and the display is the
// slave. The C5's Arduino I2C slave driver never ACKs on this silicon (the
// prebuilt IDF libs only carry the legacy slave driver, v2 is compiled
// out), while its master path is proven — and the display's S3 slave
// silicon is the exact combination the old main board ran for months.
// The C5 pushes the status frame at 10 Hz and polls the display's command
// mailbox; all command handlers and the status struct are unchanged.
extern TwoWire &Wire_lcd;

// Command mailbox served by the display's onRequest handler. Layout must
// stay byte-identical with the copy in esp32s3_lillygo_amoled/apps/ows/
// ows.ino. The display keeps the latest command in the mailbox until it is
// replaced; the C5 deduplicates by seq, so a poll lost to a checksum error
// is retried for free on the next poll.
struct I2C_CMD_MAILBOX
{
    uint8_t has_cmd;     // 0 = empty mailbox
    uint8_t seq;         // increments per NEW command from the screen
    uint16_t cmdid;      // I2CCommandID
    uint8_t payload[26]; // command struct (largest: I2C_CMD_SET_STRENGTH, 24)
    uint8_t reserved;
    uint8_t checksum;    // additive, seed 0xA5, over all preceding bytes
};                       // sizeof = 32

// Function prototypes
void initI2C();
void processI2CData();
void handleI2CCommand(uint16_t cmdid, const uint8_t *data, size_t dataSize);

// Specific command handlers
void i2cHandleGetStatus(const uint8_t *data, size_t dataSize);
void i2cHandleSetStrength(const uint8_t *data, size_t dataSize);
void i2cHandleSetRow(const uint8_t *data, size_t dataSize);
void i2cHandleSetDebug(const uint8_t *data, size_t dataSize);

// Command ID struct
struct I2C_CMDID
{
    uint16_t cmdid;
};

// Shared data structures
// NOTE: layout must stay byte-identical with the screen firmware's copy in
// esp32s3_lillygo_amoled/apps/ows/ows.ino (read raw over I2C).
struct SharedStateData
{
    float voltage;
    float current;
    bool newData; // Flag to indicate fresh voltage/current data
    // Weight-update gate state (see MotorControl.cpp): lets the screen sync
    // its display when an update latches, cancels, or comes from WiFi.
    float active_weight;   // latched weight value currently in effect
    float pending_weight;  // requested value awaiting user confirmation
    uint8_t weight_pending; // 1 while a requested value awaits confirmation
    // Row mode live data: echoes of the applied params (0 = row mode off —
    // the screen uses that to detect a dropped enable/disable and resend)
    // plus the per-stroke metrics for the row screen readout.
    uint8_t row_spm;   // strokes per minute
    uint8_t row_drag;  // applied drag 1-10, 0 while row mode is off
    uint8_t row_gear;  // applied gear 1-10, 0 while row mode is off
    float row_watts;   // smoothed per-stroke average power
    // Integrity trailer:
    uint8_t seq;      // incremented per I2C response
    uint8_t checksum; // additive checksum (seed 0xA5) over preceding bytes;
                      // lets the screen reject 0xFF filler / misaligned reads
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

// Pending actions requested from other threads (e.g., WiFi)
// These are applied by the MotorControlTask to centralize CAN TX and avoid cross-task contention.
extern volatile bool pendingApplyIdle;
extern volatile bool pendingApplyStrength;

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
    float concentric_pct; // mask 0x0008: 0-100 concentric-only unloading
                          // (was damping_percent — same offset/size)
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

// Row-mode command from the screen. Replaces the never-wired float-based
// draft struct; layout must stay byte-identical with the screen firmware's
// copy in esp32s3_lillygo_amoled/apps/ows/ows.ino.
struct I2C_CMD_SET_ROW
{
    uint16_t mask;      // 0x0001 = gear, 0x0002 = drag, 0x0004 = row_enable
    uint8_t gear;       // 1-10
    uint8_t drag;       // 1-10
    uint8_t row_enable; // with mask 0x0004: 1 = row mode on, 0 = off
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

// Link health counters for the [hb] heartbeat, i2c=<pushes>/<commands>:
// successful status-frame pushes to the display, and screen commands
// consumed from its mailbox.
extern volatile uint32_t g_i2cPollCount;
extern volatile uint32_t g_i2cRecvCount;

#endif // I2C_COMMUNICATION_H
