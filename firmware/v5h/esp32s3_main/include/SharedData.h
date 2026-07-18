#ifndef SHAREDDATA_H
#define SHAREDDATA_H

#include <Arduino.h>

// FreeRTOS queue for decoupling command ingestion from application
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

struct SharedData
{
    float accelerometer_x;
    float accelerometer_y;
    float accelerometer_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float force;
    float position;
    float velocity;
    float virtual_velocity;
    String status;
    // Debug timestamps (millis) to trace command latency across layers
    unsigned long t_http_received_ms;   // when HTTP request processed
    unsigned long t_cmd_set_ms;         // when WiFi handler set command/pending flags
    unsigned long t_ctrl_apply_ms;      // when control task applied mode change
    unsigned long mode_latency_ms;      // delta (apply - cmd_set) for last mode change
};

struct PulseData
{
    String type;
    int duration;
    int strength;
    int frequency;
    bool updated;
};

struct DetentData
{
    String type;
    int strength;
    float start_position;
    float step_position;
    int total_steps;
    bool updated;
};

struct ForceData
{
    String type;
    int strength;
    int start_strength;
    float start_position;
    float saturation_position;
    bool updated;
};

struct ModeData
{
    String type;
    bool updated;
};

struct RowData
{
    String type;
    int damping;
    int gear_ratio;
    int inertia;
    bool updated;
};

extern SharedData sharedData;
extern PulseData pulseData;
extern DetentData detentData;
extern ForceData forceData;
extern ModeData modeData;
extern RowData rowData;

void initSharedData();
void updateAccelerometerData(float x, float y, float z);
void updateGyroData(float x, float y, float z);
void updateMotorData(float force, float position, float velocity);
void updateStatus(const String &status);
void updateVirtualVelocity(float velocity);

void updatePulseData(const String &type, int duration, int strength, int frequency);
void updateDetentData(const String &type, int strength, float start_position, float step_position, int total_steps);
void updateForceData(const String &type, int strength, int start_strength, float start_position, float saturation_position);
void updateModeData(const String &type);
void updateRowData(const String &type, int damping, int gear_ratio, int inertia);

// Command queue types to avoid blocking WS/HTTP during rapid toggles
enum CommandType {
    CMD_NONE = 0,
    CMD_IDLE,
    CMD_STRENGTH
};

struct CommandMsg {
    CommandType type;
    float weight_kg; // for strength
};

extern QueueHandle_t g_cmdQueue;

#endif
