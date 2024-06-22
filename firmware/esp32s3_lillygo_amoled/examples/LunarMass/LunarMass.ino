/**
 * @file      LunarMass.ino
 * @author    Aaron (aaron@lunarmass.com)
 * @copyright Copyright (c) 2023  Lunar Mass Corp.
 * @date      2023-12-16
 */

#ifdef LILYGO_TWRITSTBAND_S3
#error "Current example does not apply to T-Wristband"
#endif
#include <LilyGo_AMOLED.h>
#include "LunarMass.h"
#include <LV_Helper.h>
#include <Wire.h>

#define SDA_PIN_MCU 43
#define SCL_PIN_MCU 44

#include <Adafruit_seesaw.h>
#define SS_SWITCH_SELECT 1
#define SS_SWITCH_UP 2    // 2
#define SS_SWITCH_LEFT 3  // 3
#define SS_SWITCH_DOWN 4  // 4
#define SS_SWITCH_RIGHT 5 // 5
#define SEESAW_ADDR 0x49
Adafruit_seesaw ss;
int32_t encoder_position;
int32_t encoder_diff_position;

LilyGo_Class amoled;
lv_obj_t *label;

TwoWire Wire_main_mcu = TwoWire(1); // Create a new TwoWire instance. The parameter (1) is optional and depends on the hardware.

#define I2C_SLAVE_ADDRESS 0x08

enum I2C_CommandID
{
    CMD_GET_STATUS = 0x0000,
    CMD_SET_STRENGTH = 0x0001,
    CMD_GET_STRENGTH_LIVE = 0x0002,
    // ... more command IDs
    CMD_SET_DEBUG = 0x0010,
};

struct I2C_CMDID
{
    uint16_t cmd_id;
};

struct SharedStateData
{
    float voltage;
    float current;
    bool newData; // Flag to indicate new data is available
};
SharedStateData sharedStateData;

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
    float damping_percent;
    bool auto_charge;
    uint8_t dynamic_feedback_mode; // none, chains, vibration, turbulence, bigfish, crank (sharper chains)
    uint8_t dynamic_feedback_amplitude;
};
I2C_CMD_SET_STRENGTH i2c_cmd_set_strength;

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
    // Wire.begin();
    // register events for incoming I2C data from the main MCU
    // Wire_main_mcu.onReceive(receiveI2CEvent);
    Wire_main_mcu.begin(SDA_PIN_MCU, SCL_PIN_MCU, 100000); // join i2c bus (address optional for master)

    // START OF OLED CODE
    bool rslt = false;

    // Begin LilyGo  1.91 Inch AMOLED board class
    // rslt =  amoled.beginAMOLED_191();

    // Automatically determine the access device
    rslt = amoled.begin();

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

    // START OF ENCODER CODE
    // while (!Serial)
    //     delay(10);
    Serial.println("Looking for seesaw!");

    if (!ss.begin(SEESAW_ADDR))
    {
        Serial.println("Couldn't find seesaw on default address");
        while (1)
            delay(10);
    }
    Serial.println("seesaw started");
    uint32_t version = ((ss.getVersion() >> 16) & 0xFFFF);
    if (version != 5740)
    {
        Serial.print("Wrong firmware loaded? ");
        Serial.println(version);
        while (1)
            delay(10);
    }
    Serial.println("Found Product 5740");

    ss.pinMode(SS_SWITCH_UP, INPUT_PULLUP);
    ss.pinMode(SS_SWITCH_DOWN, INPUT_PULLUP);
    ss.pinMode(SS_SWITCH_LEFT, INPUT_PULLUP);
    ss.pinMode(SS_SWITCH_RIGHT, INPUT_PULLUP);
    ss.pinMode(SS_SWITCH_SELECT, INPUT_PULLUP);

    // get starting position
    encoder_position = ss.getEncoderPosition();
    encoder_diff_position = 0;

    Serial.println("Turning on interrupts");
    ss.enableEncoderInterrupt();
    ss.setGPIOInterrupts((uint32_t)1 << SS_SWITCH_UP, 1);

    // Create a task for encoder detection
    xTaskCreate(
        encoderTask,   /* Task function. */
        "EncoderTask", /* String with name of task. */
        10000,         /* Stack size in words. */
        NULL,          /* Parameter passed as input of the task */
        1,             /* Priority of the task. */
        NULL);         /* Task handle. */
    // END OF ENCODER CODE

    xTaskCreate(
        readBatteryLifeTask,   /* Task function. */
        "ReadBatteryLifeTask", /* String with name of task. */
        10000,                 /* Stack size in bytes. */
        NULL,                  /* Parameter passed as input to the task. */
        1,                     /* Priority of the task. */
        NULL);                 /* Task handle. */
}

void loop()
{
    lv_task_handler();
    delay(5);
}

void encoderTask(void *parameter)
{
    static uint32_t checkMs = 0;
    const unsigned long debounceTime = 150; // 50 milliseconds
    while (1)
    {
        if (millis() > checkMs)
        {
            if (!ss.digitalRead(SS_SWITCH_UP))
            {
                Serial.println("UP pressed!");
                dispatchKeyEvent(LV_KEY_UP);
                checkMs = millis() + debounceTime;
            }
            if (!ss.digitalRead(SS_SWITCH_DOWN))
            {
                Serial.println("DOWN pressed!");
                dispatchKeyEvent(LV_KEY_DOWN);
                checkMs = millis() + debounceTime;
            }
            if (!ss.digitalRead(SS_SWITCH_LEFT))
            {
                Serial.println("LEFT pressed!");
                dispatchKeyEvent(LV_KEY_LEFT);
                checkMs = millis() + debounceTime;
            }
            if (!ss.digitalRead(SS_SWITCH_RIGHT))
            {
                Serial.println("RIGHT pressed!");
                dispatchKeyEvent(LV_KEY_RIGHT);
                checkMs = millis() + debounceTime;
            }
            if (!ss.digitalRead(SS_SWITCH_SELECT))
            {
                Serial.println("SELECT pressed!");
                dispatchKeyEvent(LV_KEY_ENTER);
                checkMs = millis() + debounceTime;
            }
        }

        int32_t new_position = ss.getEncoderPosition();
        int32_t new_diff_position = new_position - encoder_position;
        if (encoder_diff_position != new_diff_position)
        {
            Serial.println(new_position);                                                      // display new position
            encoder_position = encoder_position + (new_diff_position - encoder_diff_position); // and save for next round
            // set min and max values
            if (encoder_position < 1)
                encoder_position = 1;
            if (encoder_position > 150)
                encoder_position = 150;
            // ss.setEncoderPosition(encoder_position); // and set it
            encoder_diff_position = new_position - encoder_position;

            // Update the WeightValue label
            dispatchRotaryEvent(encoder_position);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS); // small delay to prevent task from hogging CPU
    }
}

void readBatteryLifeTask(void *parameter)
{
    for (;;) // infinite loop
    {
        // set DATA_LENGTH to the size of the struct
        const size_t DATA_LENGTH = sizeof(sharedStateData);
        Wire_main_mcu.requestFrom(I2C_SLAVE_ADDRESS, DATA_LENGTH);
        // Wait until all data is received
        if (Wire_main_mcu.available() == DATA_LENGTH)
        {
            // Read the data into the sharedStateData struct
            Wire_main_mcu.readBytes((char *)&sharedStateData, DATA_LENGTH);

            // Now you can use the data received
            Serial.print("Voltage: ");
            Serial.print(sharedStateData.voltage);
            Serial.print("V, Current: ");
            Serial.print(sharedStateData.current);
            Serial.print("A, New Data: ");
            Serial.println(sharedStateData.newData ? "Yes" : "No");
            // Update the label with the battery voltage with one decimal place
            lv_label_set_text_fmt(ui_LabelBatteryLife, "%.1fV", sharedStateData.voltage);
        }
        else
        {
            Serial.println("Failed to receive complete data");
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS); // delay for 1 second
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

void handleMainScreenEvent(uint32_t value)
{
    lv_label_set_text_fmt(ui_WeightValue, "%d", value);

    // Populate the I2C_CMD_SET_STRENGTH struct
    i2c_cmd_set_strength.mask = 0x0002;
    i2c_cmd_set_strength.home_linear_position = 0;
    i2c_cmd_set_strength.weight = value;
    i2c_cmd_set_strength.weight_max = 150; // only value updated here
    i2c_cmd_set_strength.damping_percent = 0;
    i2c_cmd_set_strength.auto_charge = false;
    i2c_cmd_set_strength.dynamic_feedback_mode = 0;
    i2c_cmd_set_strength.dynamic_feedback_amplitude = 0;

    // Send the struct as a data packet
    sendCommand(CMD_SET_STRENGTH, (uint8_t *)&i2c_cmd_set_strength, sizeof(i2c_cmd_set_strength));
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

void sendCommand(I2C_CommandID cmd, const uint8_t *data, size_t dataSize)
{
    // Start I2C transmission
    Wire_main_mcu.beginTransmission(I2C_SLAVE_ADDRESS);

    // Send the command ID
    uint16_t cmdID = static_cast<uint16_t>(cmd);
    Wire_main_mcu.write((uint8_t *)&cmdID, sizeof(cmdID));

    // Send the additional data if any
    if (data != nullptr && dataSize > 0)
    {
        Wire_main_mcu.write(data, dataSize);
    }

    // End the transmission
    byte status = Wire_main_mcu.endTransmission();
    if (status != 0)
    {
        Serial.println("I2C slave error");
        Serial.println(status);
    }
}