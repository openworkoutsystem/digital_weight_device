/**
 * @file weight_numpad.h
 * Full-screen modal numpad for setting the weight from the touch screen.
 * Lives outside src/ so SquareLine Studio re-exports never touch it.
 */
#pragma once

#include <stdint.h>

// Open the numpad over the current screen (modal, on the LVGL top layer).
// Shows currentValue dimmed until the first digit is typed. On the check
// button, the entered value is clamped to [minValue, maxValue] and passed
// to onConfirm on the LVGL thread. Tapping outside the panel cancels.
void weight_numpad_open(int32_t currentValue, int32_t minValue, int32_t maxValue,
                        void (*onConfirm)(int32_t value));

// Same numpad with a custom caption and unit text (unit may be "" for
// unitless values like the row gear/drag settings).
void weight_numpad_open_ex(const char *caption, const char *unit,
                           int32_t currentValue, int32_t minValue, int32_t maxValue,
                           void (*onConfirm)(int32_t value));

bool weight_numpad_is_open(void);
