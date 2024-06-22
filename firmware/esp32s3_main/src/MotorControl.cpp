#include "MotorControl.h"
#include "ErrorHandling.h"
#include "CANCommunication.h"
#include "I2CCommunication.h"
#include "SharedData.h" // Include the shared data header
#include <math.h>

ControlFeedback controlFeedback; // Define the controlFeedback structure

// declare functions
float applyForceControl(float target_force, float position);
float applyDetentControl(float target_force, float position);
float applyPulseControl(float target_force);
float applyRowModeControl(float target_force, float velocity, float &row_force_strength);

void initMotorControl()
{
    xTaskCreatePinnedToCore(
        MotorControlTask,   // Task function
        "MotorControlTask", // Name of the task
        10000,              // Stack size in words
        NULL,               // Task input parameter
        1,                  // Priority of the task
        NULL,               // Task handle
        0);                 // Core where the task should run
}

void MotorControlTask(void *parameter)
{
    int counter = 0;
    int lastID = 0;

    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    int64_t prev_time_us = time_us;
    int64_t runInterval_us = 2000; // 500Hz loop time.

    float m1_torque_soft = 0;
    float m2_torque_soft = 0;
    float min_torque_m1 = -0.4;
    float m1_torque_soft_max = 0;
    float m1_torque_soft_min = 0;
    float m2_torque_soft_max = 0;
    float m2_torque_soft_min = 0;

    int torque_matrix_a = 0;
    int torque_matrix_b = 1;
    float torque_interpolation = 0;
    float m1_minus_torque_soft_fn = 0;
    float m1_minus_torque_soft_fm = 0;
    float m1_plus_torque_soft_fn = 0;
    float m1_plus_torque_soft_fm = 0;
    float m2_minus_torque_soft_fn = 0;
    float m2_minus_torque_soft_fm = 0;
    float m2_plus_torque_soft_fn = 0;
    float m2_plus_torque_soft_fm = 0;
    float velocity_delta = 0;

    float vel_factor = 0.5;
    float vel_minus_factor = 0.5;
    float vel_plus_factor = 0.5;

    float m1_velocity = -20;
    float m2_velocity = -20;

    float row_force_strength = 0;

    // a 5x10 matrix to store the torque values for m1 and m2 for each range of strength mode. 5 rows and 10 columns
    // header are force, velocity_delta, -M1_Fn, -M1_Fm, M1_Fn, M1_Fm, -M2_Fn, -M2_Fm, M2_Fn, M2_Fm
    // minus sign indicates the direction of motion of the motor is towrds the device and positive sign indicates the direction of motion of the motor is away from the device towards the user
    // Fn is the static force value and Fm is the velocity coeficient force... such that F = Fn + Fm * v
    float torque_matrix[5][10] = {
        {9.8, 1, -0.7, 0, -0.5, 0, 0.2, 0, 0.2, 0.5}, // 1 lb
        {50, 1, -0.95, 0, -0.8, 0, 0.2, 0, 0.2, 0.5}, // 5 lb
        {200, 1, -1.3, 0, -1, 0, 0, 0, 0, 0},         // 10 lb
        {640, 1, -0.6, 0, -0.6, 0, -0.6, 0, -0.4, 0}, // 16 lb
        {2000, 1, -0.6, 0, -0.6, 0, -6, 0, -5, 0.1}   // 150 lb
    };

    sendMotorRX(CAN_NODEID_M2, TORQUE_SOFT_MIN_M2, (float)-1);

    // init force modulation params
    updateForceData("constant", 100, 30, 0.5, 2); // off, constant, linear
    updateDetentData("off", 50, .2, .3, 10);
    updatePulseData("off", 0, 100, 16);
    updateRowData("off", 20, 3, 0);

    while (true)
    {
        gettimeofday(&tv_now, NULL);
        time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
        if (time_us - prev_time_us < runInterval_us)
        {
            continue;
        }
        prev_time_us = time_us;

        processI2CData();

        twai_message_t message;                                    // Define the message structure
        if (twai_receive(&message, pdMS_TO_TICKS(1000)) == ESP_OK) // Receive a message from the CAN bus
        {
            uint16_t nodeID = (message.identifier >> 5) & 0x003F;
            uint16_t cmdID = message.identifier & 0x001F;

            float position = 0;
            float velocity = 0;
            float voltage = 0;
            float current = 0;
            uint32_t error = 0;

            switch (cmdID) // Check the command ID of the received message
            {
            case GET_POSVEL:
                memcpy(&position, &message.data[0], sizeof(float));
                memcpy(&velocity, &message.data[4], sizeof(float));
                if (nodeID == CAN_NODEID_M1)
                {
                    controlFeedback.m1_position = position;
                    controlFeedback.m1_velocity = velocity;

                    float desiredPosition = 0;

                    // Update shared data in sharedData.h
                    updateMotorData(sharedCfgData.target_force, controlFeedback.m1_position, controlFeedback.m1_velocity);

                    // Apply force modulations
                    // float modulated_force = sharedCfgData.target_force;
                    float modulated_force = applyForceControl(sharedCfgData.target_force, controlFeedback.m1_position);
                    // modulated_force = applyDetentControl(modulated_force, controlFeedback.m1_position);
                    modulated_force = applyPulseControl(modulated_force);
                    modulated_force = applyRowModeControl(modulated_force, controlFeedback.m1_velocity, row_force_strength);

                    // set the torque values for m1 and m2 based on the target weight and velocity and torque_matrix
                    // interpolate the torque values from the matrix based on the target weight and velocity
                    if (modulated_force <= torque_matrix[1][0])
                    {
                        torque_matrix_a = 0;
                        torque_matrix_b = 1;
                        torque_interpolation = (modulated_force - torque_matrix[0][0]) / (torque_matrix[1][0] - torque_matrix[0][0]);
                        m2_velocity = velocity * 5.5 + 2;
                    }
                    else if (modulated_force <= torque_matrix[2][0])
                    {
                        torque_matrix_a = 1;
                        torque_matrix_b = 2;
                        torque_interpolation = (modulated_force - torque_matrix[1][0]) / (torque_matrix[2][0] - torque_matrix[1][0]);
                        m2_velocity = velocity * 5.5 + (2 * (1 - torque_interpolation));
                    }
                    else if (modulated_force <= torque_matrix[3][0])
                    {
                        torque_matrix_a = 2;
                        torque_matrix_b = 3;
                        torque_interpolation = (modulated_force - torque_matrix[2][0]) / (torque_matrix[3][0] - torque_matrix[2][0]);
                        m2_velocity = -60;
                    }
                    else if (modulated_force <= torque_matrix[4][0])
                    {
                        torque_matrix_a = 3;
                        torque_matrix_b = 4;
                        torque_interpolation = (modulated_force - torque_matrix[3][0]) / (torque_matrix[4][0] - torque_matrix[3][0]);
                        m2_velocity = -60;
                    }
                    else
                    {
                        Serial.printf("Target weight out of range: %f\n", modulated_force);
                        // set torque matrix to min
                        torque_matrix_a = 0;
                        torque_matrix_b = 1;
                        torque_interpolation = (modulated_force - torque_matrix[0][0]) / (torque_matrix[1][0] - torque_matrix[0][0]);
                        m2_velocity = velocity * 5.5 + 2;
                    }

                    m1_minus_torque_soft_fn = torque_matrix[torque_matrix_a][2] + (torque_matrix[torque_matrix_b][2] - torque_matrix[torque_matrix_a][2]) * torque_interpolation;
                    m1_minus_torque_soft_fm = torque_matrix[torque_matrix_a][3] + (torque_matrix[torque_matrix_b][3] - torque_matrix[torque_matrix_a][3]) * torque_interpolation;
                    m1_plus_torque_soft_fn = torque_matrix[torque_matrix_a][4] + (torque_matrix[torque_matrix_b][4] - torque_matrix[torque_matrix_a][4]) * torque_interpolation;
                    m1_plus_torque_soft_fm = torque_matrix[torque_matrix_a][5] + (torque_matrix[torque_matrix_b][5] - torque_matrix[torque_matrix_a][5]) * torque_interpolation;
                    m2_minus_torque_soft_fn = torque_matrix[torque_matrix_a][6] + (torque_matrix[torque_matrix_b][6] - torque_matrix[torque_matrix_a][6]) * torque_interpolation;
                    m2_minus_torque_soft_fm = torque_matrix[torque_matrix_a][7] + (torque_matrix[torque_matrix_b][7] - torque_matrix[torque_matrix_a][7]) * torque_interpolation;
                    m2_plus_torque_soft_fn = torque_matrix[torque_matrix_a][8] + (torque_matrix[torque_matrix_b][8] - torque_matrix[torque_matrix_a][8]) * torque_interpolation;
                    m2_plus_torque_soft_fm = torque_matrix[torque_matrix_a][9] + (torque_matrix[torque_matrix_b][9] - torque_matrix[torque_matrix_a][9]) * torque_interpolation;
                    velocity_delta = torque_matrix[torque_matrix_a][1] + (torque_matrix[torque_matrix_b][1] - torque_matrix[torque_matrix_a][1]) * torque_interpolation;
                    vel_factor = ((velocity + velocity_delta) / velocity_delta) * 0.5;
                    vel_factor = (vel_factor > 0.5) ? 0.5 : vel_factor;
                    vel_factor = (vel_factor < -0.5) ? -0.5 : vel_factor;

                    vel_minus_factor = 0.5 - vel_factor;
                    vel_plus_factor = 0.5 + vel_factor;

                    float vel_positive = (velocity > 0) ? abs(velocity) : 0;
                    float vel_negative = (velocity < 0) ? abs(velocity) : 0;

                    m1_torque_soft = (m1_minus_torque_soft_fn + m1_minus_torque_soft_fm * vel_negative) * vel_minus_factor + (m1_plus_torque_soft_fn + m1_plus_torque_soft_fm * vel_positive) * vel_plus_factor;
                    m2_torque_soft = (m2_minus_torque_soft_fn + m2_minus_torque_soft_fm * vel_negative) * vel_minus_factor + (m2_plus_torque_soft_fn + m2_plus_torque_soft_fm * vel_positive) * vel_plus_factor;

                    m1_torque_soft_max = (m1_torque_soft > 0) ? m1_torque_soft : 0;
                    m1_torque_soft_min = (m1_torque_soft <= 0) ? m1_torque_soft : 0;
                    m2_torque_soft_max = (m2_torque_soft > 0) ? m2_torque_soft : 0;
                    m2_torque_soft_min = (m2_torque_soft <= 0) ? m2_torque_soft : 0;

                    // Serial.printf("M1 Torque: %f, M2 Torque: %f\n", m1_torque_soft, m2_torque_soft);

                    if (controlState.mode == 1) // update odrive motor controllers over CAN
                    {
                        sendMotorVelocity(CAN_NODEID_M1, m1_velocity);
                        sendMotorRX(CAN_NODEID_M1, TORQUE_SOFT_MIN_M1, m1_torque_soft_min);
                        m2_velocity = -m2_velocity;
                        m2_torque_soft_max = -m2_torque_soft_max;
                        m2_torque_soft_min = -m2_torque_soft_min;
                        sendMotorVelocity(CAN_NODEID_M2, m2_velocity);
                        sendMotorRX(CAN_NODEID_M2, TORQUE_SOFT_MAX_M2, m2_torque_soft_min);
                        sendMotorRX(CAN_NODEID_M2, TORQUE_SOFT_MIN_M2, m2_torque_soft_max);
                    }
                }
                else if (nodeID == CAN_NODEID_M2) // M2 is only a slave motor... so just update position and velocity info and wait for M1 pos/vel info
                {
                    controlFeedback.m2_position = position;
                    controlFeedback.m2_velocity = velocity;
                }
                break;
            case GET_VBUS:
                memcpy(&voltage, &message.data[0], sizeof(float));
                memcpy(&current, &message.data[4], sizeof(float));
                controlFeedback.voltage = voltage;
                controlFeedback.current = current;
                // Serial.printf("Voltage: %f, Current: %f\n", voltage, current);

                if (xSemaphoreTake(mutex, 0) == pdTRUE)
                {
                    sharedStateData.voltage = voltage;
                    sharedStateData.current = current;
                    sharedStateData.newData = true;
                    xSemaphoreGive(mutex);
                }
                else
                {
                    handleError(ERROR_I2C_RECEIVE, "Failed to take mutex for processing CAN VBUS data");
                }
                break;
            case GET_MOTORERR:
                memcpy(&error, &message.data[0], sizeof(uint32_t));
                if (nodeID == CAN_NODEID_M1)
                {
                    controlState.error_state = error;
                }
                else if (nodeID == CAN_NODEID_M2)
                {
                    controlState.error_state = error;
                }
                break;
            default:
                break;
            }
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

float applyForceControl(float target_force, float position)
{
    float force = target_force;
    noInterrupts(); // Begin critical section
    if (forceData.type == "off")
    {
        force = 0;
    }
    else if (forceData.type == "constant")
    {
        force = (forceData.strength / 100.0) * target_force;
    }
    else if (forceData.type == "linear")
    {
        if (position < forceData.start_position)
        {
            force = 0;
        }
        else if (position > forceData.saturation_position)
        {
            force = (forceData.strength / 100.0) * target_force;
        }
        else
        {
            float scale = (position - forceData.start_position) / (forceData.saturation_position - forceData.start_position);
            // if scale is less than 0, set it to 0
            scale = (scale < 0) ? 0 : scale;
            // if scale is greater than 1, set it to 1
            scale = (scale > 1) ? 1 : scale;
            float strength = forceData.start_strength + scale * (forceData.strength - forceData.start_strength);
            force = (strength / 100.0) * target_force;
        }
    }
    // forceData.updated = false;
    interrupts(); // End critical section
    return force;
}

float applyDetentControl(float target_force, float position)
{
    float force = target_force;
    noInterrupts(); // Begin critical section
    if (detentData.type != "on")
    {
        force = target_force;
    }
    else
    {
        if (position >= detentData.start_position)
        {
            int step = (position - detentData.start_position) / detentData.step_position;
            if (step < detentData.total_steps)
            {
                force = target_force * (1.0 - (detentData.strength / 100.0));
            }
        }
    }
    // detentData.updated = false;
    interrupts(); // End critical section
    return force;
}

float applyPulseControl(float target_force)
{
    noInterrupts(); // Begin critical section
    static float adder_force = 0;
    float force = target_force;
    float max_force = 0.4 * target_force;

    int frequency = pulseData.frequency;
    if (frequency < 3)
    {
        frequency = 3;
    }

    if (pulseData.type == "on")
    {
        unsigned long currentTime = millis();
        float period = 1000.0 / pulseData.frequency; // Period in milliseconds

        // Calculate the time fraction within the period
        float timeFraction = fmod(static_cast<float>(currentTime), period) / period;

        // Calculate sine wave modulation
        adder_force = sin(2 * PI * timeFraction) * (pulseData.strength / 100.0) * max_force;

        // Calculate final force
        force = target_force + adder_force;
    }
    interrupts(); // End critical section

    return force;
}

float applyRowModeControl(float target_force, float velocity, float &row_force_strength)
{
    float force = target_force;
    noInterrupts(); // Begin critical section
    // float delta_velocity_threshold = (0.5 + rowData.gear_ratio / 10);
    float delta_velocity_threshold = 0.5;
    float velocity_slow_rate = 0.02;
    float velocity_gain_rate = 0.005;
    float force_change_rate = 7;

    if (rowData.type != "off")
    {
        float virtual_velocity = sharedData.virtual_velocity;
        float damping_loss = (velocity_slow_rate * (rowData.damping / 100.0)) * (virtual_velocity);
        float velocity_delta = velocity - virtual_velocity;

        if (velocity_delta > 0)
        {
            float force = (velocity_delta / delta_velocity_threshold) * (target_force);
            virtual_velocity += (velocity_delta * velocity_gain_rate);
            row_force_strength += force_change_rate;
            row_force_strength = (row_force_strength > 100) ? 100 : row_force_strength;
        }
        else
        {
            row_force_strength -= force_change_rate;
        }

        // Min force
        row_force_strength = (row_force_strength < 0) ? 0 : row_force_strength;
        force = row_force_strength / 100.0 * force;

        // Min virtual velocity is 0
        virtual_velocity -= damping_loss;
        virtual_velocity = (virtual_velocity < 0) ? 0 : virtual_velocity;
        sharedData.virtual_velocity = virtual_velocity;
    }
    interrupts(); // End critical section
    return force;
}
