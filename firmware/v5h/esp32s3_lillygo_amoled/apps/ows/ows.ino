/**
 * @file      ows.ino
 * @author    Aaron (aaron@lunarmass.com)
 * @copyright Copyright (c) 2023  Lunar Mass Corp.
 * @date      2023-12-16
 */

#ifdef LILYGO_TWRITSTBAND_S3
#error "Current example does not apply to T-Wristband"
#endif
#include <LilyGo_AMOLED.h>
#include "ows.h"
#include <LV_Helper.h>
#include <Wire.h>
#include "weight_numpad.h"

// Explicit prototypes (keeps the Arduino .ino preprocessor honest)
void weightValueClicked(lv_event_t *e);
void numpadWeightConfirmed(int32_t value);
void setWeightDisplay(int32_t value);
void sendWeightCommand(int32_t value);
void zeroZonePressed(lv_event_t *e);
void createZeroWeightZone(void);
void rowScreenEvent(lv_event_t *e);
void gearValueClicked(lv_event_t *e);
void dragValueClicked(lv_event_t *e);
void numpadGearConfirmed(int32_t value);
void numpadDragConfirmed(int32_t value);
void setRowLabels(void);
void sendRowCommand(bool enable);
void mainMcuFrameReceived(int howMany);
void mainMcuMailboxRequested(void);
void concentricValueClicked(lv_event_t *e);
void numpadConcentricConfirmed(int32_t value);
void setConcentricLabel(void);
void sendConcentricCommand(void);

#define SDA_PIN_MCU 43
#define SCL_PIN_MCU 44

LilyGo_Class amoled;
lv_obj_t *label;

// Current weight setting (was tracked by the external encoder; now set via
// the on-screen numpad). 0 lb = motor idle (device does not move at all);
// 1 lb or more = active strength mode. Defaults to 0 (idle), matching the
// main board's power-on state.
static int32_t currentWeight = 0;
#define WEIGHT_MIN 0
#define WEIGHT_MAX 150

TwoWire Wire_main_mcu = TwoWire(1); // Create a new TwoWire instance. The parameter (1) is optional and depends on the hardware.

#define I2C_SLAVE_ADDRESS 0x08

enum I2C_CommandID
{
    CMD_GET_STATUS = 0x0000,
    CMD_SET_STRENGTH = 0x0001,
    CMD_GET_STRENGTH_LIVE = 0x0002,
    CMD_SET_ROW = 0x0003,
    // ... more command IDs
    CMD_SET_DEBUG = 0x0010,
};

struct I2C_CMDID
{
    uint16_t cmd_id;
};

// NOTE: layout must stay byte-identical with the main board's copy in
// esp32s3_main/include/I2CCommunication.h (read raw over I2C).
struct SharedStateData
{
    float voltage;
    float current;
    bool newData; // Flag to indicate fresh voltage/current data
    // Weight-update gate state from the main board: a commanded weight only
    // takes effect after the user confirms it by pulling; it can cancel.
    float active_weight;    // latched weight currently in effect
    float pending_weight;   // requested value awaiting confirmation
    uint8_t weight_pending; // 1 while a requested value awaits confirmation
    // Row mode: param echoes (0 = row mode off on the main board — used to
    // detect a dropped enable/disable and resend) plus stroke metrics.
    uint8_t row_spm;   // strokes per minute
    uint8_t row_drag;  // applied drag 1-10, 0 while row mode is off
    uint8_t row_gear;  // applied gear 1-10, 0 while row mode is off
    float row_watts;   // smoothed per-stroke average power
    // Integrity trailer:
    uint8_t seq;      // incremented per response by the main board
    uint8_t checksum; // additive checksum (seed 0xA5) over preceding bytes
};
SharedStateData sharedStateData;

// ROLE FLIP (2026-07-13): the C5 masters this link — its Arduino I2C slave
// driver never ACKs on that silicon, while THIS board's S3 slave role is
// the exact combination the old main board ran for months. The C5 pushes
// the 32-byte status frame at 10 Hz (mainMcuFrameReceived) and polls this
// command mailbox (mainMcuMailboxRequested); commands are staged by
// sendCommand and held until replaced — the C5 dedups by seq, so lost
// polls retry free. Layout must stay byte-identical with the C5's copy in
// esp32c5_seed_studio_main/include/I2CCommunication.h.
// NOTE: this type must be defined ABOVE the first function definition in
// this file — the .ino preprocessor hoists generated prototypes there.
struct I2C_CMD_MAILBOX
{
    uint8_t has_cmd;     // 0 = empty mailbox
    uint8_t seq;         // increments per NEW command from this screen
    uint16_t cmdid;      // I2C_CommandID
    uint8_t payload[26]; // command struct (largest: I2C_CMD_SET_STRENGTH, 24)
    uint8_t reserved;
    uint8_t checksum;    // additive, seed 0xA5, over all preceding bytes
};                       // sizeof = 32

static I2C_CMD_MAILBOX g_mailbox = {};
// Latest status frame pushed by the C5, consumed by readBatteryLifeTask
static SharedStateData g_rxFrame;
static volatile bool g_rxFresh = false;
static volatile uint32_t g_rxMs = 0;
static portMUX_TYPE g_linkMux = portMUX_INITIALIZER_UNLOCKED;

static uint8_t sharedStateChecksum(const SharedStateData *d)
{
    const uint8_t *p = (const uint8_t *)d;
    uint8_t sum = 0xA5;
    for (size_t i = 0; i < offsetof(SharedStateData, checksum); i++)
    {
        sum += p[i];
    }
    return sum;
}

// Suppress display re-sync briefly after we send a weight, so a stale status
// poll can't snap the label back before the main board registers the command.
static uint32_t lastWeightSentMs = 0;
// Command acknowledgment: a sent weight stays "unacked" until a status poll
// shows the main board acting on it (armed as pending, or applied as
// active). Unacked commands are resent — a dropped I2C write must not let
// the display sync snap the value back as if the user never set it.
static bool weightAckPending = false;
static uint8_t weightSendTries = 0;

struct I2C_SET_DEBUG
{
    uint16_t mask = 0; // one-hot mask to indicate which data is new to be set. ignore if 0.
    uint8_t mode = 0;
    uint8_t level = 0;
    bool toggle_switch = false;
};
I2C_SET_DEBUG i2c_set_debug;

// evually see if we can make this a common file with the other mcu
struct I2C_CMD_SET_STRENGTH
{
    uint16_t mask; // one-hot mask to indicate which data is new to be set. ignore if 0.
    float home_linear_position;
    float weight;
    float weight_max;
    float concentric_pct; // mask 0x0008: 0-100 concentric-only unloading
                          // (was damping_percent — same offset/size)
    bool auto_charge;
    uint8_t dynamic_feedback_mode; // none, chains, vibration, turbulence, bigfish, crank (sharper chains)
    uint8_t dynamic_feedback_amplitude;
};
I2C_CMD_SET_STRENGTH i2c_cmd_set_strength;

// NOTE: layout must stay byte-identical with the main board's copy in
// esp32c5_seed_studio_main/include/I2CCommunication.h.
struct I2C_CMD_SET_ROW
{
    uint16_t mask;      // 0x0001 = gear, 0x0002 = drag, 0x0004 = row_enable
    uint8_t gear;       // 1-10
    uint8_t drag;       // 1-10
    uint8_t row_enable; // with mask 0x0004: 1 = row mode on, 0 = off
};
I2C_CMD_SET_ROW i2c_cmd_set_row;

static uint8_t mailboxChecksum(const I2C_CMD_MAILBOX *mb)
{
    const uint8_t *p = (const uint8_t *)mb;
    uint8_t sum = 0xA5;
    for (size_t i = 0; i < offsetof(I2C_CMD_MAILBOX, checksum); i++)
    {
        sum += p[i];
    }
    return sum;
}

// Row settings (screen-side view; defaults match a mid-damper C2 feel).
// rowActive mirrors "the row screen is showing" — entering the screen
// enables row mode on the main board, leaving it disables it.
static uint8_t currentGear = 1;
static uint8_t currentDrag = 1;
// Concentric-only percentage shown/set on the strength screen (0 = normal)
static uint8_t currentConcentric = 0;
static bool rowActive = false;
static uint32_t lastRowSentMs = 0;
// Like the weight path: a sent row command stays unacked until the 1 Hz
// status poll echoes it back; mismatches are resent (bounded tries).
static uint8_t rowSendTries = 0;

// ---------------- UI Event Queue to keep LVGL single-threaded ----------------
enum UiEventType : uint8_t { UIE_KEY = 1, UIE_ROTARY = 2, UIE_BATT = 3, UIE_WEIGHT_SYNC = 4, UIE_ROW_STATS = 5 };
struct UiEvent {
    UiEventType type;
    uint32_t    u32;   // keys/rotary value
    float       f32;   // battery voltage if needed
};
static QueueHandle_t uiEventQueue = nullptr;

// --------------- I2C Bus Mutex for Wire_main_mcu (main MCU) -----------------
static SemaphoreHandle_t i2c1Mutex = nullptr;

// --------------- Navigation transition guard (avoid double-trigger) ---------
static volatile uint32_t ui_transition_until_ms = 0; // millis until which nav keys are ignored
static inline bool isTransitionBusy()
{
    // Treat any running animations as busy; also honor our time-based guard window
    return (millis() < ui_transition_until_ms) || (lv_anim_count_running() > 0);
}

// void receiveI2CEvent(int howMany)
// {
//     Serial.println("I2C slave event received.");
//     while (Wire_main_mcu.available())
//     {
//         char c = Wire_main_mcu.read(); // receive byte as a character
//         Serial.print(c);               // print the character
//     }
// }

void setup(void)
{
    Serial.begin(115200);
    // Diagnose flaky boots: ESP_RST_BROWNOUT means the 5V rail sagged;
    // anything else with a dead screen points at panel init timing instead.
    esp_reset_reason_t resetReason = esp_reset_reason();
    Serial.printf("Reset reason: %d%s\n", (int)resetReason,
                  resetReason == ESP_RST_BROWNOUT ? " (BROWNOUT - supply sagged)" : "");
    // ROLE FLIP: this display is the I2C SLAVE (addr 8) now — the C5
    // masters the link, pushing status frames and polling the mailbox.
    Wire_main_mcu.onReceive(mainMcuFrameReceived);
    Wire_main_mcu.onRequest(mainMcuMailboxRequested);
    Wire_main_mcu.begin((uint8_t)I2C_SLAVE_ADDRESS, SDA_PIN_MCU, SCL_PIN_MCU, 400000);

    // START OF OLED CODE
    bool rslt = false;

    // Enable ultra-fast boot with immediate splash. Optionally set your known board.
    amoled.setFastBoot(true);
    amoled.setSplashFirst(true);
    // Fast boot cuts panel timing below the RM67162 datasheet (20ms Sleep-Out
    // wait vs 120ms required) which can cause intermittent init failures.
    // Restore the Sleep-Out wait and a saner reset pulse; costs ~150ms.
    amoled.tuneInitDelays(120, 1);
    amoled.tuneResetDelays(20, 30, 50);
    // Boot the panel dimmer than normal (175) to limit inrush on the 5V
    // rail; loop() ramps it up the rest of the way.
    amoled.setBootBrightness(100);
    // If you know your panel, uncomment ONE of these to skip autodetect entirely:
    // amoled.setKnownBoardID(LILYGO_AMOLED_191); // T-Display-S3 AMOLED 1.91"
    // amoled.setKnownBoardID(LILYGO_AMOLED_241); // T4-S3 AMOLED 2.41"

    // Begin LilyGo  1.91 Inch AMOLED board class
    // rslt =  amoled.beginAMOLED_191();

    // Automatically determine the access device
    rslt = amoled.begin();
    // Fast-boot defers touch; initialize it now so LVGL gets touches
    amoled.initTouchNow();
    // Add a small timeout to default Wire as well (LVGL touch / PMU use this)
    Wire.setTimeOut(20);

    if (!rslt)
    {
        while (1)
        {
            Serial.println("The board model cannot be detected, please raise the Core Debug Level to an error");
            delay(1000);
        }
    }

    // Register lvgl helper
    beginLvglHelper(amoled);

    amoled.setHomeButtonCallback([](void *ptr)
                                 {
        Serial.println("Home key pressed!");
        static uint32_t checkMs = 0;
        static uint8_t lastBri = 0;
        if (millis() > checkMs) {
            if (amoled.getBrightness()) {
                lastBri = amoled.getBrightness();
                amoled.setBrightness(0);
            } else {
                amoled.setBrightness(lastBri);
            }
        }
        checkMs = millis() + 200; },
                                 NULL);

    ui_init();

    // Tapping the weight number opens the on-screen numpad (the external
    // encoder is gone). Widen the hit area — a label's clickable region is
    // only as big as its text.
    lv_obj_set_ext_click_area(ui_WeightValue, 40);
    lv_obj_add_event_cb(ui_WeightValue, weightValueClicked, LV_EVENT_CLICKED, NULL);
    // SquareLine's default label text is "1"; reflect the real boot state (0 = idle)
    lv_label_set_text_fmt(ui_WeightValue, "%d", (int)currentWeight);
    // Safety: big red zero-weight zone on the left of the strength screen
    // (built in code, like the numpad, so SquareLine re-exports keep it)
    createZeroWeightZone();
    // Fit the weight readout beside the zero zone: the SquareLine-assigned
    // font is 230 px, so three digits ran underneath the X. 140 px fits
    // "150" with clearance; the unit drops to 32 px beside the baseline.
    lv_obj_set_style_text_font(ui_WeightValue, &ui_font_RowFont2, 0);
    lv_obj_set_x(ui_WeightValue, -96);
    lv_obj_set_style_text_font(ui_WeightUnit, &lv_font_montserrat_32, 0);
    lv_obj_align(ui_WeightUnit, LV_ALIGN_RIGHT_MID, -42, 26);
    // The old "43%" placeholder label now earns its keep as the
    // CONCENTRIC-ONLY percentage: tap it to set 0-100 via the numpad.
    // 0 = normal (force in both directions), 100 = force only while
    // pulling out — holds and lowering fade to the keep-taut floor.
    lv_obj_clear_flag(ui_WeightUnit1, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_text_font(ui_WeightUnit1, &lv_font_montserrat_44, 0);
    lv_obj_set_style_text_color(ui_WeightUnit1, lv_color_hex(0x888888), 0);
    lv_obj_align(ui_WeightUnit1, LV_ALIGN_TOP_RIGHT, -16, 8);
    lv_obj_add_flag(ui_WeightUnit1, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_ext_click_area(ui_WeightUnit1, 40);
    lv_obj_add_event_cb(ui_WeightUnit1, concentricValueClicked, LV_EVENT_CLICKED, NULL);
    setConcentricLabel();

    // ---- Row screen wiring (all screens persist after ui_init) ----
    // Tapping the gear or drag number opens the shared numpad, same pattern
    // as the strength weight. Values are 1-10 (C2-style damper scale).
    // Labels are NOT clickable by default in LVGL 8 — the strength weight
    // label only works because SquareLine sets the flag in the designer;
    // these two never got it, so set it here.
    lv_obj_add_flag(ui_Label10, LV_OBJ_FLAG_CLICKABLE); // gear value
    lv_obj_set_ext_click_area(ui_Label10, 40);
    lv_obj_add_event_cb(ui_Label10, gearValueClicked, LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(ui_Label13, LV_OBJ_FLAG_CLICKABLE); // drag value
    lv_obj_set_ext_click_area(ui_Label13, 40);
    lv_obj_add_event_cb(ui_Label13, dragValueClicked, LV_EVENT_CLICKED, NULL);
    // Entering the row screen enables row mode on the main board; leaving
    // disables it (strength weight is cleared by the main board on entry).
    lv_obj_add_event_cb(ui_RowScreen, rowScreenEvent, LV_EVENT_ALL, NULL);
    // Replace SquareLine's placeholder texts with the real boot state
    setRowLabels();
    lv_label_set_text(ui_Label9, "0"); // average power readout
    // SquareLine's "89%" battery placeholders read as a real level; show a
    // clear no-data marker until the first status frame carries a voltage
    lv_label_set_text(ui_LabelBatteryLife, "--");
    lv_label_set_text(ui_LabelBatteryLife2, "--");
    lv_label_set_text(ui_LabelBatteryLife3, "--");

    // Create UI event queue (route all UI updates via main loop)
    uiEventQueue = xQueueCreate(16, sizeof(UiEvent));
    // Create mutex for I2C bus 1 shared with the main MCU
    i2c1Mutex = xSemaphoreCreateMutex();

    xTaskCreate(
        readBatteryLifeTask,   /* Task function. */
        "ReadBatteryLifeTask", /* String with name of task. */
        10000,                 /* Stack size in bytes. */
        NULL,                  /* Parameter passed as input to the task. */
        1,                     /* Priority of the task. */
        NULL);                 /* Task handle. */

    // Cold power-up can leave the panel deaf to the early splash write (it
    // needs longer after power-on than a warm reboot). Wait for the deferred
    // init replay to finish bringing the panel up, then redraw the splash
    // from this task — the SPI path has no lock, so pixel writes must never
    // come from another task.
    uint32_t finalizeWaitStart = millis();
    while (!amoled.finalizeDone() && millis() - finalizeWaitStart < 2000)
        delay(10);
    amoled.redrawSplash();

    // Hold the splash on screen briefly before loop() starts rendering the
    // UI over it — with the fast boot path, setup() can finish in a few
    // hundred ms and the splash would otherwise barely flash by.
    while (millis() < 1200)
        delay(10);
}

void loop()
{
    // Drain any pending UI events from worker tasks and apply on LVGL thread
    if (uiEventQueue) {
        UiEvent ev;
        // Process multiple events per frame to keep UI snappy
        for (int i = 0; i < 8; ++i) {
            if (xQueueReceive(uiEventQueue, &ev, 0) != pdTRUE) break;
            switch (ev.type) {
                case UIE_KEY:
                    if (ev.u32 == LV_KEY_LEFT || ev.u32 == LV_KEY_RIGHT) {
                        if (!isTransitionBusy()) {
                            dispatchKeyEvent(ev.u32);
                            // Small guard to avoid overlapping screen transitions
                            ui_transition_until_ms = millis() + 220;
                        }
                        // If busy, drop the key; the user can press again after transition
                    } else {
                        dispatchKeyEvent(ev.u32);
                    }
                    break;
                case UIE_ROTARY:
                    // Avoid heavy UI updates during transitions
                    if (!isTransitionBusy()) {
                        dispatchRotaryEvent(ev.u32);
                    }
                    break;
                case UIE_BATT:
                    lv_label_set_text_fmt(ui_LabelBatteryLife,  "%.1fV", ev.f32);
                    lv_label_set_text_fmt(ui_LabelBatteryLife2, "%.1fV", ev.f32);
                    lv_label_set_text_fmt(ui_LabelBatteryLife3, "%.1fV", ev.f32);
                    break;
                case UIE_WEIGHT_SYNC:
                    // Main board reports a different latched weight than we
                    // show (update cancelled, or set via WiFi) — sync display
                    // only, do NOT resend over I2C.
                    setWeightDisplay((int32_t)ev.u32);
                    break;
                case UIE_ROW_STATS:
                    // 1 Hz row readout: average power big number, and the
                    // gear/drag labels re-synced (they can change via WiFi)
                    lv_label_set_text_fmt(ui_Label9, "%d", (int)lroundf(ev.f32));
                    setRowLabels();
                    break;
                default:
                    break;
            }
        }
    }
    // Post-boot brightness ramp: the panel boots dim (setBootBrightness) to
    // avoid a current step on the 5V rail; fade up to the normal level here.
    static bool bootBrightnessRamp = true;
    static uint32_t bootBrightnessMs = 0;
    if (bootBrightnessRamp && millis() > bootBrightnessMs) {
        uint8_t b = amoled.getBrightness();
        if (b >= AMOLED_DEFAULT_BRIGHTNESS) {
            bootBrightnessRamp = false;
        } else {
            uint16_t next = (uint16_t)b + 5;
            amoled.setBrightness(next > AMOLED_DEFAULT_BRIGHTNESS ? AMOLED_DEFAULT_BRIGHTNESS : (uint8_t)next);
            bootBrightnessMs = millis() + 15;
        }
    }
    lv_task_handler();
    delay(5);
}

void readBatteryLifeTask(void *parameter)
{
    for (;;) // infinite loop
    {
        bool resendWeight = false; // decided below, acted on after
        bool resendRow = false;    // decided below, acted on after
        // Consume the latest status frame the C5 pushed (mainMcuFrameReceived);
        // frames arrive at 10 Hz, this task interprets at 1 Hz.
        SharedStateData frame;
        bool fresh = false;
        portENTER_CRITICAL(&g_linkMux);
        if (g_rxFresh)
        {
            frame = g_rxFrame;
            g_rxFresh = false;
            fresh = true;
        }
        uint32_t lastRxMs = g_rxMs;
        portEXIT_CRITICAL(&g_linkMux);

        if (fresh)
        {
            sharedStateData = frame;

            // Reject corrupt frames (0xFF filler from an empty slave
            // response reads as NaN floats; misaligned reads scramble
            // fields) — never display or act on them
            if (sharedStateChecksum(&sharedStateData) != sharedStateData.checksum)
            {
                static uint32_t badFrames = 0;
                Serial.printf("Status frame rejected (checksum), %lu total\n",
                              (unsigned long)++badFrames);
            }
            else
            {
                // Frame verified — safe to act on.
                // Voltage rides in EVERY frame — update the readout from
                // each one. (Gating on newData made the label wait for the
                // 1-in-10 push carrying the flag; with unlucky phase
                // alignment across the 10 Hz push / 1 Hz sample clocks that
                // took a minute after power-on.) Zero = no CAN telemetry
                // yet — keep the placeholder rather than showing 0.0V.
                if (uiEventQueue && sharedStateData.voltage > 0.1f)
                {
                    UiEvent ev{UIE_BATT, 0, sharedStateData.voltage};
                    xQueueSend(uiEventQueue, &ev, 0);
                }
                if (sharedStateData.newData)
                {
                    Serial.print("Voltage: ");
                    Serial.print(sharedStateData.voltage);
                    Serial.print("V, Current: ");
                    Serial.print(sharedStateData.current);
                    Serial.println("A");
                }

                // Weight command acknowledgment: the command counts as seen
                // once the main board reports it armed (pending) or applied
                // (active). A lost I2C write is resent instead of letting
                // the sync below snap the display back to the old value.
                if (weightAckPending)
                {
                    int32_t activeW = (int32_t)lroundf(sharedStateData.active_weight);
                    int32_t pendW = (int32_t)lroundf(sharedStateData.pending_weight);
                    if ((sharedStateData.weight_pending && pendW == currentWeight) ||
                        activeW == currentWeight)
                    {
                        weightAckPending = false; // acknowledged
                    }
                    else if (millis() - lastWeightSentMs > 1000)
                    {
                        if (weightSendTries < 3)
                        {
                            resendWeight = true; // after the bus mutex is released
                        }
                        else
                        {
                            Serial.println("Weight command unacknowledged after 3 tries, adopting main board value");
                            weightAckPending = false;
                        }
                    }
                }

                // Weight display sync: if no command is in flight, nothing
                // is pending confirmation, and the main board's latched
                // weight differs from what we show (update cancelled, or
                // weight set via WiFi), snap the display to it.
                if (!weightAckPending && !resendWeight &&
                    !sharedStateData.weight_pending && millis() - lastWeightSentMs > 3000)
                {
                    int32_t applied = (int32_t)lroundf(sharedStateData.active_weight);
                    if (applied != currentWeight && uiEventQueue)
                    {
                        Serial.printf("Weight sync: main board reports %d, display shows %d\n",
                                      (int)applied, (int)currentWeight);
                        UiEvent ev{UIE_WEIGHT_SYNC, (uint32_t)applied, 0};
                        xQueueSend(uiEventQueue, &ev, 0);
                    }
                }

                // Row acknowledgment: the gear/drag echoes are 0 while row
                // mode is off on the main board, so one comparison covers
                // dropped enables, disables, and param updates alike.
                bool mainRowOn = (sharedStateData.row_gear >= 1 && sharedStateData.row_gear <= 10);
                if (rowSendTries > 0 && millis() - lastRowSentMs > 3000)
                {
                    bool mismatch = rowActive
                                        ? (!mainRowOn ||
                                           sharedStateData.row_gear != currentGear ||
                                           sharedStateData.row_drag != currentDrag)
                                        : mainRowOn;
                    if (!mismatch)
                    {
                        rowSendTries = 0; // acknowledged
                    }
                    else if (rowSendTries < 3)
                    {
                        resendRow = true; // after the bus mutex is released
                    }
                    else
                    {
                        Serial.println("Row command unacknowledged after 3 tries, giving up");
                        rowSendTries = 0;
                    }
                }
                // Param sync: row running, nothing in flight — adopt values
                // changed from the WiFi side so the labels stay honest
                if (rowActive && mainRowOn && rowSendTries == 0)
                {
                    currentGear = sharedStateData.row_gear;
                    currentDrag = sharedStateData.row_drag;
                }
                // Live metrics to the UI thread (only meaningful on the row
                // screen, but the labels exist permanently — cheap update)
                if (rowActive && uiEventQueue)
                {
                    UiEvent ev{UIE_ROW_STATS, (uint32_t)sharedStateData.row_spm, sharedStateData.row_watts};
                    xQueueSend(uiEventQueue, &ev, 0);
                }
            }
        }
        else
        {
            // Link-down watchdog: the C5 pushes at 10 Hz, so 5 s of silence
            // means the link (or the C5) is down
            static uint32_t linkDownLogMs = 0;
            if (millis() - lastRxMs > 5000 && millis() - linkDownLogMs > 5000)
            {
                linkDownLogMs = millis();
                Serial.println("No status frame from main board in 5 s (link down?)");
            }
        }

        if (resendWeight)
        {
            weightSendTries++;
            Serial.printf("Weight command not acknowledged, resending %d (try %u)\n",
                          (int)currentWeight, (unsigned)weightSendTries);
            sendWeightCommand(currentWeight);
        }
        if (resendRow)
        {
            Serial.printf("Row command not acknowledged, resending enable=%d (try %u)\n",
                          (int)rowActive, (unsigned)(rowSendTries + 1));
            sendRowCommand(rowActive);
        }

        // 1s cadence: fast enough for weight-cancel display sync to feel
        // responsive, light enough for the shared I2C bus
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void readBatteryLife()
{
    // Read the battery voltage from GPIO 4 (ADC input)
    uint16_t adcValue = analogRead(4);

    // Convert ADC value to voltage considering the voltage divider and ADC resolution
    float batteryVoltage = (adcValue / 4095.0) * 3.4 * 2; // Multiply by 2 due to voltage divider... 1.1 is the internal reference voltage factor

    // Convert voltage to percentage (4.1V is 100% and 3.6V is 0%)
    float batteryPercent = (batteryVoltage - 3.6) * 100.0 / (4.1 - 3.6);

    // Clamp the percentage to be between 0% and 100%
    if (batteryPercent > 100.0)
    {
        batteryPercent = 100.0;
    }
    else if (batteryPercent < 0.0)
    {
        batteryPercent = 0.0;
    }

    // Update the label with the calculated battery percentage
    lv_label_set_text_fmt(ui_LabelBatteryLife, "%d%%", (int)batteryPercent);
}

void dispatchRotaryEvent(uint32_t value)
{
    lv_obj_t *active_screen = lv_scr_act(); // Get the currently active screen

    // You can use a switch-case or if-else statements to decide based on the active screen
    if (active_screen == ui_StrengthScreen)
    {
        // Call the event handler for the MainScreen
        handleMainScreenEvent(value);
    }
}

void weightValueClicked(lv_event_t *e)
{
    (void)e;
    weight_numpad_open(currentWeight, WEIGHT_MIN, WEIGHT_MAX, numpadWeightConfirmed);
}

void numpadWeightConfirmed(int32_t value)
{
    // Runs on the LVGL thread (numpad button event), so call straight in
    handleMainScreenEvent((uint32_t)value);
}

void setWeightDisplay(int32_t value)
{
    currentWeight = value;
    lv_label_set_text_fmt(ui_WeightValue, "%d", (int)value);
}

void sendWeightCommand(int32_t value)
{
    lastWeightSentMs = millis();
    weightAckPending = true;

    // Populate the I2C_CMD_SET_STRENGTH struct
    i2c_cmd_set_strength.mask = 0x0002;
    i2c_cmd_set_strength.home_linear_position = 0;
    i2c_cmd_set_strength.weight = value;
    i2c_cmd_set_strength.weight_max = 150; // only value updated here
    i2c_cmd_set_strength.concentric_pct = 0;
    i2c_cmd_set_strength.auto_charge = false;
    i2c_cmd_set_strength.dynamic_feedback_mode = 0;
    i2c_cmd_set_strength.dynamic_feedback_amplitude = 0;

    // Send the struct as a data packet
    sendCommand(CMD_SET_STRENGTH, (uint8_t *)&i2c_cmd_set_strength, sizeof(i2c_cmd_set_strength));
}

// ---------------------- Concentric-only percentage --------------------------

void concentricValueClicked(lv_event_t *e)
{
    (void)e;
    weight_numpad_open_ex("CONCENTRIC %", "%", currentConcentric, 0, 100, numpadConcentricConfirmed);
}

void numpadConcentricConfirmed(int32_t value)
{
    currentConcentric = (uint8_t)value;
    setConcentricLabel();
    sendConcentricCommand();
}

void setConcentricLabel(void)
{
    lv_label_set_text_fmt(ui_WeightUnit1, "%d%%", (int)currentConcentric);
}

void sendConcentricCommand(void)
{
    i2c_cmd_set_strength.mask = 0x0008; // concentric_pct only
    i2c_cmd_set_strength.home_linear_position = 0;
    i2c_cmd_set_strength.weight = 0;
    i2c_cmd_set_strength.weight_max = 0;
    i2c_cmd_set_strength.concentric_pct = currentConcentric;
    i2c_cmd_set_strength.auto_charge = false;
    i2c_cmd_set_strength.dynamic_feedback_mode = 0;
    i2c_cmd_set_strength.dynamic_feedback_amplitude = 0;
    sendCommand(CMD_SET_STRENGTH, (uint8_t *)&i2c_cmd_set_strength, sizeof(i2c_cmd_set_strength));
}

void handleMainScreenEvent(uint32_t value)
{
    setWeightDisplay((int32_t)value);
    weightSendTries = 1;
    sendWeightCommand((int32_t)value);
}

void dispatchKeyEvent(uint32_t key)
{
    lv_obj_t *active_screen = lv_scr_act(); // Get the currently active screen

    if (active_screen)
    {
        lv_event_send(active_screen, LV_EVENT_KEY, (void *)&key);
    }

    if (active_screen == ui_StrengthScreen && key == LV_KEY_ENTER)
    {
        // Populate the I2C_CMD_SET_STRENGTH struct
        i2c_set_debug.mask = 0x0004; // one-hot mask to indicate which data is new to be set. ignore if 0.
        i2c_set_debug.toggle_switch = true;

        // Send the struct as a data packet
        sendCommand(CMD_SET_DEBUG, (uint8_t *)&i2c_set_debug, sizeof(i2c_set_debug));
    }
}

void zeroZonePressed(lv_event_t *e)
{
    (void)e;
    // Fires on PRESS (touch-down), not release — an emergency control
    // shouldn't wait for the finger to lift. Full command path: display,
    // ack/retry, cancels any pending increase, main board goes to recoil.
    handleMainScreenEvent(0);
}

void createZeroWeightZone(void)
{
    lv_obj_t *zone = lv_obj_create(ui_StrengthScreen);
    lv_obj_remove_style_all(zone);
    // Left ~30% of the screen, below the top status bar
    lv_obj_set_size(zone, 160, 184);
    lv_obj_align(zone, LV_ALIGN_BOTTOM_LEFT, 3, -3);
    lv_obj_set_style_bg_color(zone, lv_color_hex(0x2A0000), 0);
    lv_obj_set_style_bg_opa(zone, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(zone, 10, 0);
    lv_obj_set_style_border_color(zone, lv_color_hex(0xCC2222), 0);
    lv_obj_set_style_border_width(zone, 2, 0);
    // Flash bright on touch so the action is unmistakable
    lv_obj_set_style_bg_color(zone, lv_color_hex(0xB00000), LV_STATE_PRESSED);
    lv_obj_add_flag(zone, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(zone, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(zone, zeroZonePressed, LV_EVENT_PRESSED, NULL);

    // Big drawn X (children are non-clickable, so touches land on the zone)
    static lv_point_t xLegA[] = {{0, 0}, {84, 84}};
    static lv_point_t xLegB[] = {{84, 0}, {0, 84}};
    lv_obj_t *legA = lv_line_create(zone);
    lv_line_set_points(legA, xLegA, 2);
    lv_obj_set_style_line_width(legA, 12, 0);
    lv_obj_set_style_line_color(legA, lv_color_hex(0xE03030), 0);
    lv_obj_set_style_line_rounded(legA, true, 0);
    lv_obj_align(legA, LV_ALIGN_CENTER, 0, -22);
    lv_obj_t *legB = lv_line_create(zone);
    lv_line_set_points(legB, xLegB, 2);
    lv_obj_set_style_line_width(legB, 12, 0);
    lv_obj_set_style_line_color(legB, lv_color_hex(0xE03030), 0);
    lv_obj_set_style_line_rounded(legB, true, 0);
    lv_obj_align(legB, LV_ALIGN_CENTER, 0, -22);

    lv_obj_t *caption = lv_label_create(zone);
    lv_label_set_text(caption, "ZERO WEIGHT");
    lv_obj_set_style_text_font(caption, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(caption, lv_color_hex(0xE03030), 0);
    lv_obj_align(caption, LV_ALIGN_BOTTOM_MID, 0, -12);
}

// ------------------------------ Row screen ---------------------------------

// Screen enter/leave drives row mode on the main board. LOADED fires when
// the slide-in completes; UNLOAD_START fires the moment navigation away
// begins, so the force is dropped promptly.
void rowScreenEvent(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_SCREEN_LOADED)
    {
        rowActive = true;
        rowSendTries = 0;
        sendRowCommand(true);
    }
    else if (code == LV_EVENT_SCREEN_UNLOAD_START)
    {
        rowActive = false;
        rowSendTries = 0;
        sendRowCommand(false);
    }
}

void gearValueClicked(lv_event_t *e)
{
    (void)e;
    weight_numpad_open_ex("SET GEAR", "", currentGear, 1, 10, numpadGearConfirmed);
}

void dragValueClicked(lv_event_t *e)
{
    (void)e;
    weight_numpad_open_ex("SET DRAG", "", currentDrag, 1, 10, numpadDragConfirmed);
}

void numpadGearConfirmed(int32_t value)
{
    currentGear = (uint8_t)value;
    setRowLabels();
    rowSendTries = 0;
    sendRowCommand(rowActive);
}

void numpadDragConfirmed(int32_t value)
{
    currentDrag = (uint8_t)value;
    setRowLabels();
    rowSendTries = 0;
    sendRowCommand(rowActive);
}

void setRowLabels(void)
{
    lv_label_set_text_fmt(ui_Label10, "%d", (int)currentGear); // gear value
    lv_label_set_text_fmt(ui_Label13, "%d", (int)currentDrag); // drag value
}

void sendRowCommand(bool enable)
{
    lastRowSentMs = millis();
    rowSendTries++;

    i2c_cmd_set_row.mask = 0x0007; // gear + drag + enable-state, always
    i2c_cmd_set_row.gear = currentGear;
    i2c_cmd_set_row.drag = currentDrag;
    i2c_cmd_set_row.row_enable = enable ? 1 : 0;

    sendCommand(CMD_SET_ROW, (uint8_t *)&i2c_cmd_set_row, sizeof(i2c_cmd_set_row));
}

// Stage a command in the mailbox; the C5 collects it on its next 10 Hz
// poll. The mailbox holds the newest command until replaced — the C5
// dedups by seq, and the existing ack/resend logic (driven by the status
// frames) covers a mailbox overwritten before it was collected.
void sendCommand(I2C_CommandID cmd, const uint8_t *data, size_t dataSize)
{
    I2C_CMD_MAILBOX mb = {};
    mb.has_cmd = 1;
    mb.cmdid = static_cast<uint16_t>(cmd);
    if (data != nullptr && dataSize > 0)
    {
        memcpy(mb.payload, data, min(dataSize, sizeof(mb.payload)));
    }
    static uint8_t cmdSeq = 0;
    mb.seq = ++cmdSeq;
    mb.checksum = mailboxChecksum(&mb);

    portENTER_CRITICAL(&g_linkMux);
    g_mailbox = mb;
    portEXIT_CRITICAL(&g_linkMux);
    Serial.printf("Mailbox: staged cmd 0x%04X (seq %u)\n", (unsigned)cmd, (unsigned)mb.seq);
}

// C5 pushed a status frame (I2C slave onReceive, runs in the Wire task):
// copy it out and hand off to readBatteryLifeTask for interpretation.
void mainMcuFrameReceived(int howMany)
{
    if (howMany < (int)sizeof(SharedStateData))
    {
        while (Wire_main_mcu.available())
        {
            Wire_main_mcu.read();
        }
        return;
    }
    SharedStateData f;
    Wire_main_mcu.readBytes((char *)&f, sizeof(f));
    while (Wire_main_mcu.available())
    {
        Wire_main_mcu.read();
    }
    portENTER_CRITICAL(&g_linkMux);
    g_rxFrame = f;
    g_rxFresh = true;
    g_rxMs = millis();
    portEXIT_CRITICAL(&g_linkMux);
}

// C5 is polling the command mailbox (I2C slave onRequest)
void mainMcuMailboxRequested(void)
{
    I2C_CMD_MAILBOX mb;
    portENTER_CRITICAL(&g_linkMux);
    mb = g_mailbox;
    portEXIT_CRITICAL(&g_linkMux);
    Wire_main_mcu.write((uint8_t *)&mb, sizeof(mb));
}