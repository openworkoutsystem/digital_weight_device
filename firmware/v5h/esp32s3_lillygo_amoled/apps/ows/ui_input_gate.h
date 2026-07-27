/**
 * @file ui_input_gate.h
 * Touch debounce for screen transitions and modal close.
 *
 * Problem it solves (Open Sauce demo, 2026-07): a tap that navigates lands
 * a second time on whatever appears under the finger ~250 ms later — the
 * screen-change animation is faster than a person lifts off / taps again.
 * Worst case is hitting ZERO WEIGHT or the numpad by accident.
 *
 * The gate disables the pointer input device for a short window, so LVGL
 * generates no press/click/gesture events at all — this covers
 * SquareLine-generated handlers too, which we don't want to edit.
 *
 * Lives outside src/ so SquareLine re-exports never touch it.
 */
#pragma once

#include <lvgl.h>
#include <stdint.h>

// Block windows. Screen: covers the 250 ms transition animation plus a
// margin for the finger to lift. Modal: numpad open/close.
#define UI_INPUT_GATE_SCREEN_MS 400
#define UI_INPUT_GATE_MODAL_MS 250

// Ignore touch for `ms` from now. Extends an in-flight block, never
// shortens it. Also drops the current press so a finger still down cannot
// click through when input resumes.
void ui_input_gate_block(uint32_t ms);

// True while touch is being ignored (handlers can double-check).
bool ui_input_gate_blocked(void);

// Call once per UI loop: restores touch when the block expires.
void ui_input_gate_tick(void);

// Register the automatic screen-transition block on a screen object.
void ui_input_gate_attach_screen(lv_obj_t *screen);
