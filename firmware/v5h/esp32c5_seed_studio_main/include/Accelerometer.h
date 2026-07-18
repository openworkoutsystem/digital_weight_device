// include/Accelerometer.h
#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <Arduino.h>

// BMI270 on the LP-I2C bus. The C5's LP-I2C is a hardware master fixed to
// LP_GPIO2/3 = GPIO2/3 (it bypasses the GPIO matrix) — on the XIAO those are
// the rear MTMS/MTDI through-hole pads, next to the rear 3V3/GND pads where
// the Qwiic pigtail lands. If Wire1 (bus 1 = LP-I2C) misbehaves, the same
// wiring supports the IDF LP-I2C driver or bit-banging — see MIGRATION_PLAN.md.
#define SDA_PIN_ACC 2 // rear MTMS pad (LP_I2C_SDA)
#define SCL_PIN_ACC 3 // rear MTDI pad (LP_I2C_SCL)

void initAccelerometer();
void AccelerometerTask(void *parameter);

#endif // ACCELEROMETER_H
