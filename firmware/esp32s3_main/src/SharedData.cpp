#include "SharedData.h"

SharedData sharedData;
PulseData pulseData;
DetentData detentData;
ForceData forceData;
ModeData modeData;
RowData rowData;

void initSharedData()
{
    sharedData.accelerometer_x = 0.0;
    sharedData.accelerometer_y = 0.0;
    sharedData.accelerometer_z = 0.0;
    sharedData.gyro_x = 0.0;
    sharedData.gyro_y = 0.0;
    sharedData.gyro_z = 0.0;
    sharedData.force = 0.0;
    sharedData.position = 0.0;
    sharedData.velocity = 0.0;
    sharedData.virtual_velocity = 0.0;
    sharedData.status = "OK";

    pulseData.updated = false;
    detentData.updated = false;
    forceData.updated = false;
    modeData.updated = false;
    rowData.updated = false;

    // also initialize the type of each struct
    pulseData.type = "off";
    detentData.type = "off";
    forceData.type = "off";
    modeData.type = "off";
    rowData.type = "off";
}

void updateAccelerometerData(float x, float y, float z)
{
    noInterrupts();
    sharedData.accelerometer_x = x;
    sharedData.accelerometer_y = y;
    sharedData.accelerometer_z = z;
    interrupts();
}

void updateGyroData(float x, float y, float z)
{
    noInterrupts();
    sharedData.gyro_x = x;
    sharedData.gyro_y = y;
    sharedData.gyro_z = z;
    interrupts();
}

void updateMotorData(float force, float position, float velocity)
{
    noInterrupts();
    sharedData.force = force;
    sharedData.position = position;
    sharedData.velocity = velocity;
    interrupts();
}

void updateVirtualVelocity(float velocity)
{
    noInterrupts();
    sharedData.virtual_velocity = velocity;
    interrupts();
}

void updateStatus(const String &status)
{
    noInterrupts();
    sharedData.status = status;
    interrupts();
}

void updatePulseData(const String &type, int duration, int strength, int frequency)
{
    noInterrupts();
    pulseData.type = type;
    pulseData.duration = duration;
    pulseData.strength = strength;
    pulseData.frequency = frequency;
    pulseData.updated = true;
    interrupts();
}

void updateDetentData(const String &type, int strength, float start_position, float step_position, int total_steps)
{
    noInterrupts();
    detentData.type = type;
    detentData.strength = strength;
    detentData.start_position = start_position;
    detentData.step_position = step_position;
    detentData.total_steps = total_steps;
    detentData.updated = true;
    interrupts();
}

void updateForceData(const String &type, int strength, int start_strength, float start_position, float saturation_position)
{
    noInterrupts();
    forceData.type = type;
    forceData.strength = strength;
    forceData.start_strength = start_strength;
    forceData.start_position = start_position;
    forceData.saturation_position = saturation_position;
    forceData.updated = true;
    interrupts();
}

void updateModeData(const String &type)
{
    noInterrupts();
    modeData.type = type;
    modeData.updated = true;
    interrupts();
}

void updateRowData(const String &type, int damping, int gear_ratio, int inertia)
{
    noInterrupts();
    rowData.type = type;
    rowData.damping = damping;
    rowData.gear_ratio = gear_ratio;
    rowData.inertia = inertia;
    rowData.updated = true;
    interrupts();
}
