#ifndef SHAREDDATA_H
#define SHAREDDATA_H

#include <Arduino.h>

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

#endif
