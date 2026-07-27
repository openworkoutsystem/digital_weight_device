/**
 * @file weight_numpad.cpp
 * Full-screen modal numpad for setting the weight from the touch screen.
 */
#include <lvgl.h>
#include <string.h>
#include <stdlib.h>
#include "weight_numpad.h"
#include "ui_input_gate.h"

// Gold accent matching the OWS logo
#define NUMPAD_ACCENT_HEX 0xD4A017

static lv_obj_t *overlay = NULL;
static lv_obj_t *valueLabel = NULL;
static char entry[4]; // up to 3 digits + NUL
static int32_t initialValue = 1;
static int32_t minValue = 1;
static int32_t maxValue = 150;
static void (*confirmCb)(int32_t) = NULL;

static const char *btnMap[] = {
    "7", "8", "9", "\n",
    "4", "5", "6", "\n",
    "1", "2", "3", "\n",
    LV_SYMBOL_BACKSPACE, "0", LV_SYMBOL_OK, ""
};
#define BTN_ID_OK 11

bool weight_numpad_is_open(void)
{
    return overlay != NULL;
}

static void refreshReadout(void)
{
    if (entry[0] == '\0') {
        // Nothing typed yet: show the current value dimmed
        lv_label_set_text_fmt(valueLabel, "%d", (int)initialValue);
        lv_obj_set_style_text_color(valueLabel, lv_color_hex(0x666666), 0);
    } else {
        lv_label_set_text(valueLabel, entry);
        lv_obj_set_style_text_color(valueLabel, lv_color_hex(0xFFFFFF), 0);
    }
}

static void closeNumpad(void)
{
    if (overlay) {
        lv_obj_del(overlay);
        overlay = NULL;
        valueLabel = NULL;
        // The screen behind is now exposed under the finger — on the
        // strength screen that includes the ZERO WEIGHT zone. Debounce.
        ui_input_gate_block(UI_INPUT_GATE_MODAL_MS);
    }
}

static void backdropClicked(lv_event_t *e)
{
    // Only a tap on the backdrop itself cancels; the panel absorbs its own
    if (lv_event_get_target(e) == overlay) {
        closeNumpad();
    }
}

static void btnmClicked(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    uint16_t id = lv_btnmatrix_get_selected_btn(obj);
    if (id == LV_BTNMATRIX_BTN_NONE) {
        return;
    }
    const char *txt = lv_btnmatrix_get_btn_text(obj, id);
    if (!txt) {
        return;
    }

    if (strcmp(txt, LV_SYMBOL_BACKSPACE) == 0) {
        size_t n = strlen(entry);
        if (n > 0) {
            entry[n - 1] = '\0';
        }
        refreshReadout();
    } else if (strcmp(txt, LV_SYMBOL_OK) == 0) {
        int32_t v = (entry[0] != '\0') ? atoi(entry) : initialValue;
        if (v < minValue) v = minValue;
        if (v > maxValue) v = maxValue;
        void (*cb)(int32_t) = confirmCb;
        closeNumpad();
        if (cb) {
            cb(v);
        }
    } else {
        size_t n = strlen(entry);
        if (n < sizeof(entry) - 1) {
            entry[n] = txt[0];
            entry[n + 1] = '\0';
            refreshReadout();
        }
    }
}

// Paint the OK key gold with black glyph
static void btnmDrawPart(lv_event_t *e)
{
    lv_obj_draw_part_dsc_t *dsc = lv_event_get_draw_part_dsc(e);
    if (dsc->class_p == &lv_btnmatrix_class && dsc->type == LV_BTNMATRIX_DRAW_PART_BTN &&
        dsc->id == BTN_ID_OK) {
        if (dsc->rect_dsc) {
            dsc->rect_dsc->bg_color = lv_color_hex(NUMPAD_ACCENT_HEX);
        }
        if (dsc->label_dsc) {
            dsc->label_dsc->color = lv_color_black();
        }
    }
}

void weight_numpad_open(int32_t currentValue, int32_t minV, int32_t maxV,
                        void (*onConfirm)(int32_t value))
{
    weight_numpad_open_ex("SET WEIGHT", "lb", currentValue, minV, maxV, onConfirm);
}

void weight_numpad_open_ex(const char *captionText, const char *unitText,
                           int32_t currentValue, int32_t minV, int32_t maxV,
                           void (*onConfirm)(int32_t value))
{
    if (overlay) {
        return;
    }
    initialValue = currentValue;
    minValue = minV;
    maxValue = maxV;
    confirmCb = onConfirm;
    entry[0] = '\0';

    // Dimming backdrop on the top layer: modal over any screen, absorbs
    // taps and gestures so nothing reaches the UI behind it
    overlay = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(overlay);
    lv_obj_set_size(overlay, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(overlay, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(overlay, LV_OPA_70, 0);
    lv_obj_add_flag(overlay, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(overlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(overlay, backdropClicked, LV_EVENT_CLICKED, NULL);

    lv_obj_t *panel = lv_obj_create(overlay);
    lv_obj_remove_style_all(panel);
    lv_obj_set_size(panel, 520, 232);
    lv_obj_center(panel);
    lv_obj_set_style_bg_color(panel, lv_color_hex(0x101010), 0);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(panel, 8, 0);
    lv_obj_set_style_border_color(panel, lv_color_hex(0x303030), 0);
    lv_obj_set_style_border_width(panel, 1, 0);
    lv_obj_add_flag(panel, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *caption = lv_label_create(panel);
    lv_label_set_text(caption, captionText);
    lv_obj_set_style_text_font(caption, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(caption, lv_color_hex(0x888888), 0);
    lv_obj_align(caption, LV_ALIGN_TOP_LEFT, 24, 18);

    valueLabel = lv_label_create(panel);
    lv_obj_set_width(valueLabel, 120);
    lv_obj_set_style_text_align(valueLabel, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_font(valueLabel, &lv_font_montserrat_48, 0);
    lv_obj_align(valueLabel, LV_ALIGN_TOP_LEFT, 24, 80);

    lv_obj_t *unit = lv_label_create(panel);
    lv_label_set_text(unit, unitText);
    lv_obj_set_style_text_font(unit, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(unit, lv_color_hex(0x888888), 0);
    lv_obj_align(unit, LV_ALIGN_TOP_LEFT, 152, 102);

    lv_obj_t *hint = lv_label_create(panel);
    lv_label_set_text_fmt(hint, "%d - %d  •  tap outside\nto cancel", (int)minValue, (int)maxValue);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(hint, lv_color_hex(0x666666), 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_LEFT, 24, -16);

    lv_obj_t *btnm = lv_btnmatrix_create(panel);
    lv_btnmatrix_set_map(btnm, btnMap);
    lv_obj_set_size(btnm, 300, 216);
    lv_obj_align(btnm, LV_ALIGN_RIGHT_MID, -10, 0);
    lv_obj_set_style_bg_opa(btnm, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(btnm, 0, 0);
    lv_obj_set_style_pad_all(btnm, 4, 0);
    lv_obj_set_style_pad_row(btnm, 6, 0);
    lv_obj_set_style_pad_column(btnm, 6, 0);
    lv_obj_set_style_bg_color(btnm, lv_color_hex(0x222222), LV_PART_ITEMS);
    lv_obj_set_style_bg_color(btnm, lv_color_hex(0x3A3A3A), LV_PART_ITEMS | LV_STATE_PRESSED);
    lv_obj_set_style_text_color(btnm, lv_color_hex(0xFFFFFF), LV_PART_ITEMS);
    lv_obj_set_style_text_font(btnm, &lv_font_montserrat_28, LV_PART_ITEMS);
    lv_obj_set_style_radius(btnm, 6, LV_PART_ITEMS);
    lv_obj_set_style_border_width(btnm, 0, LV_PART_ITEMS);
    lv_obj_set_style_shadow_width(btnm, 0, LV_PART_ITEMS);
    lv_obj_clear_flag(btnm, LV_OBJ_FLAG_CLICK_FOCUSABLE);
    lv_obj_add_event_cb(btnm, btnmClicked, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(btnm, btnmDrawPart, LV_EVENT_DRAW_PART_BEGIN, NULL);

    refreshReadout();

    // The keypad appears under the finger that opened it — debounce so a
    // quick second tap can't immediately punch a digit.
    ui_input_gate_block(UI_INPUT_GATE_MODAL_MS);
}
