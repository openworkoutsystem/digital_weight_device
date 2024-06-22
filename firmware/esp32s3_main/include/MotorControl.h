#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// Declare controlFeedback structure
struct ControlFeedback
{
    float m1_position;
    float m1_velocity;
    float m2_position;
    float m2_velocity;
    float voltage;
    float current;
};

extern ControlFeedback controlFeedback; // Declare as extern to be used in other files

void initMotorControl();
void MotorControlTask(void *parameter);

#endif // MOTOR_CONTROL_H
