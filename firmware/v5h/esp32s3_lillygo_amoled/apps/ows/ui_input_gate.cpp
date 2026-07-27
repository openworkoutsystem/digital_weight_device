/**
 * @file ui_input_gate.cpp
 * Touch debounce — see ui_input_gate.h.
 */
#include <Arduino.h>
#include <lvgl.h>
#include "ui_input_gate.h"

// Deadline in millis(); signed differences keep the 49-day wrap safe.
static uint32_t s_blockUntil = 0;
static bool s_blocked = false;

static void setPointerEnabled(bool en)
{
    lv_indev_t *indev = lv_indev_get_next(NULL);
    while (indev)
    {
        if (lv_indev_get_type(indev) == LV_INDEV_TYPE_POINTER)
        {
            lv_indev_enable(indev, en);
            // Whatever press is in flight is abandoned: a finger still down
            // when input resumes must lift before it can click anything.
            lv_indev_wait_release(indev);
        }
        indev = lv_indev_get_next(indev);
    }
}

void ui_input_gate_block(uint32_t ms)
{
    uint32_t until = millis() + ms;
    if (!s_blocked || (int32_t)(until - s_blockUntil) > 0)
    {
        s_blockUntil = until;
    }
    if (!s_blocked)
    {
        s_blocked = true;
        setPointerEnabled(false);
    }
}

bool ui_input_gate_blocked(void)
{
    return s_blocked;
}

void ui_input_gate_tick(void)
{
    if (s_blocked && (int32_t)(millis() - s_blockUntil) >= 0)
    {
        s_blocked = false;
        setPointerEnabled(true);
    }
}

static void screenLoadStartCb(lv_event_t *e)
{
    (void)e;
    ui_input_gate_block(UI_INPUT_GATE_SCREEN_MS);
}

void ui_input_gate_attach_screen(lv_obj_t *screen)
{
    if (!screen)
    {
        return;
    }
    lv_obj_add_event_cb(screen, screenLoadStartCb, LV_EVENT_SCREEN_LOAD_START, NULL);
}
