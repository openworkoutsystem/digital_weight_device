/**
 * @file      LilyGo_AMOLED.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2023  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2023-05-29
 *
 */

#include "LilyGo_AMOLED.h"
#include <esp_adc_cal.h>
#include <driver/gpio.h>
#include <esp_system.h>
#include "logo.h"

#define SEND_BUF_SIZE           (16384)
#define TFT_SPI_MODE            SPI_MODE0
#define DEFAULT_SPI_HANDLER    (SPI3_HOST)

LilyGo_AMOLED::LilyGo_AMOLED() : boards(NULL)
{
    pBuffer = NULL;
    _brightness = AMOLED_DEFAULT_BRIGHTNESS;
    // Prevent previously set hold
    switch (esp_sleep_get_wakeup_cause()) {
    case ESP_SLEEP_WAKEUP_EXT0 :
    case ESP_SLEEP_WAKEUP_EXT1 :
    case ESP_SLEEP_WAKEUP_TIMER:
    case ESP_SLEEP_WAKEUP_ULP :
        gpio_hold_dis(GPIO_NUM_14);
        gpio_deep_sleep_hold_dis();
        break;
    default :
        break;
    }
}

LilyGo_AMOLED::~LilyGo_AMOLED()
{
    if (pBuffer) {
        free(pBuffer);
        pBuffer = NULL;
    }
}

const char *LilyGo_AMOLED::getName()
{
    if (boards == &BOARD_AMOLED_147) {
        return "1.47 inch";
    } else if (boards == &BOARD_AMOLED_191 ) {
        return "1.91 inch";
    } else if (boards == &BOARD_AMOLED_241) {
        return "2.41 inch";
    }
    return "Unkonw";
}

uint8_t LilyGo_AMOLED::getBoardID()
{
    if (boards == &BOARD_AMOLED_147) {
        return LILYGO_AMOLED_147;
    } else if (boards == &BOARD_AMOLED_191 ) {
        return LILYGO_AMOLED_191;
    } else if (boards == &BOARD_AMOLED_241) {
        return LILYGO_AMOLED_241;
    }
    return LILYGO_AMOLED_UNKOWN;
}

const BoardsConfigure_t *LilyGo_AMOLED::getBoarsdConfigure()
{
    return boards;
}

uint16_t  LilyGo_AMOLED::width()
{
    return boards->display.width;
}

uint16_t  LilyGo_AMOLED::height()
{
    return boards->display.height;
}

void inline LilyGo_AMOLED::setCS()
{
    digitalWrite(boards->display.cs, LOW);
}

void inline LilyGo_AMOLED::clrCS()
{
    digitalWrite(boards->display.cs, HIGH);
}

bool LilyGo_AMOLED::isPressed()
{
    if (boards == &BOARD_AMOLED_147) {
        return TouchDrvCHSC5816::isPressed();
    } else if (boards == &BOARD_AMOLED_191 || boards == &BOARD_AMOLED_241) {
        return TouchDrvCSTXXX::isPressed();
    }
    return false;
}

uint8_t LilyGo_AMOLED::getPoint(int16_t *x, int16_t *y, uint8_t get_point )
{
    uint8_t point = 0;
    if (boards == &BOARD_AMOLED_147) {
        point =  TouchDrvCHSC5816::getPoint(x, y);
    } else if (boards == &BOARD_AMOLED_191 || boards == &BOARD_AMOLED_241) {
        point =  TouchDrvCSTXXX::getPoint(x, y);
    }
    return point;
}

uint16_t LilyGo_AMOLED::getBattVoltage(void)
{
    if (boards) {
        if (boards->pmu) {
            if (boards->pmu) {
                if (boards == &BOARD_AMOLED_147) {
                    return XPowersAXP2101::getBattVoltage();
                } else  if (boards == &BOARD_AMOLED_241) {
                    return PowersSY6970::getBattVoltage();
                }
            }
        } else if (boards->adcPins != -1) {
            esp_adc_cal_characteristics_t adc_chars;
            esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
            uint32_t v1 = 0,  raw = 0;
            raw = analogRead(boards->adcPins);
            v1 = esp_adc_cal_raw_to_voltage(raw, &adc_chars) * 2;
            return v1;
        }
    }
    return 0;
}

uint16_t LilyGo_AMOLED::getVbusVoltage(void)
{
    if (boards) {
        if (boards->pmu) {
            if (boards == &BOARD_AMOLED_147) {
                return XPowersAXP2101::getVbusVoltage();
            } else  if (boards == &BOARD_AMOLED_241) {
                return PowersSY6970::getVbusVoltage();
            }
        }
    }
    return 0;
}

bool LilyGo_AMOLED::isBatteryConnect(void)
{
    if (boards) {
        if (boards->pmu) {
            if (boards == &BOARD_AMOLED_147) {
                return XPowersAXP2101::isBatteryConnect();
            } else  if (boards == &BOARD_AMOLED_241) {
                return PowersSY6970::isBatteryConnect();
            }
        }
    }
    return false;
}

uint16_t LilyGo_AMOLED::getSystemVoltage(void)
{
    if (boards) {
        if (boards->pmu) {
            if (boards == &BOARD_AMOLED_147) {
                return XPowersAXP2101::getSystemVoltage();
            } else  if (boards == &BOARD_AMOLED_241) {
                return PowersSY6970::getSystemVoltage();
            }
        }
    }
    return 0;
}

bool LilyGo_AMOLED::isCharging(void)
{
    if (boards) {
        if (boards->pmu) {
            if (boards == &BOARD_AMOLED_147) {
                return XPowersAXP2101::isCharging();
            } else  if (boards == &BOARD_AMOLED_241) {
                return PowersSY6970::isCharging();
            }
        }
    }
    return false;
}

bool LilyGo_AMOLED::isVbusIn(void)
{
    if (boards) {
        if (boards->pmu) {
            if (boards == &BOARD_AMOLED_147) {
                return XPowersAXP2101::isVbusIn();
            } else  if (boards == &BOARD_AMOLED_241) {
                return PowersSY6970::isVbusIn();
            }
        }
    }
    return false;
}


uint32_t deviceScan(TwoWire *_port, Stream *stream)
{
    stream->println("Devices Scan start.");
    uint8_t err, addr;
    int nDevices = 0;
    for (addr = 1; addr < 127; addr++) {
        _port->beginTransmission(addr);
        err = _port->endTransmission();
        if (err == 0) {
            stream->print("I2C device found at address 0x");
            if (addr < 16)
                stream->print("0");
            stream->print(addr, HEX);
            stream->println(" !");
            nDevices++;
        } else if (err == 4) {
            stream->print("Unknow error at address 0x");
            if (addr < 16)
                stream->print("0");
            stream->println(addr, HEX);
        }
    }
    if (nDevices == 0)
        stream->println("No I2C devices found\n");
    else
        stream->println("Done\n");
    return nDevices;
}

bool LilyGo_AMOLED::initPMU()
{
    bool res = XPowersAXP2101::init(Wire, boards->pmu->sda, boards->pmu->scl, AXP2101_SLAVE_ADDRESS);
    if (!res) {
        return false;
    }

    clearPMU();

    setChargingLedMode(XPOWERS_CHG_LED_CTRL_CHG);

    // ALDO1 = AMOLED logic power & Sensor Power voltage
    setALDO1Voltage(1800);
    enableALDO1();

    // ALDO3 = Level conversion enable and AMOLED power supply
    setALDO3Voltage(3300);
    enableALDO3();

    // BLDO1 = AMOLED LOGIC POWER 1.8V
    setBLDO1Voltage(1800);
    enableBLDO1();

    // No use power channel
    disableDC2();
    disableDC3();
    disableDC4();
    disableDC5();
    disableCPUSLDO();

    // Enable PMU ADC
    enableBattDetection();
    enableVbusVoltageMeasure();
    enableBattVoltageMeasure();

    return res;
}

bool LilyGo_AMOLED::initBUS()
{
    assert(boards);
    log_i("=====CONFIGURE======");
    log_i("RST    > %d", boards->display.rst);
    log_i("CS     > %d", boards->display.cs);
    log_i("SCK    > %d", boards->display.sck);
    log_i("D0     > %d", boards->display.d0);
    log_i("D1     > %d", boards->display.d1);
    log_i("D2     > %d", boards->display.d2);
    log_i("D3     > %d", boards->display.d3);
    log_i("TE     > %d", boards->display.te);
    log_i("Freq   > %d", boards->display.freq);
    log_i("Power  > %d", boards->PMICEnPins);
    log_i("==================");

    pinMode(boards->display.rst, OUTPUT);
    pinMode(boards->display.cs, OUTPUT);

    if (boards->display.te != -1) {
        pinMode(boards->display.te, INPUT);
    }

    if (boards->PMICEnPins != -1) {
        pinMode(boards->PMICEnPins, OUTPUT);
        digitalWrite(boards->PMICEnPins, HIGH);
        // Let the panel supply rail charge before the reset sequence starts
        // loading it. On cold power-up the panel's supply and internal
        // power-on reset need much longer than a warm reboot (where the rail
        // never dropped) — commands sent too early are silently ignored.
        delay(esp_reset_reason() == ESP_RST_POWERON ? 100 : 20);
    }

    //reset display (configurable delays)
    digitalWrite(boards->display.rst, HIGH);
    delay(_boot.resetDelayHigh1Ms);
    digitalWrite(boards->display.rst, LOW);
    delay(_boot.resetDelayLowMs);
    digitalWrite(boards->display.rst, HIGH);
    delay(_boot.resetDelayHigh2Ms);

    spi_bus_config_t buscfg = {
        .data0_io_num = boards->display.d0,
        .data1_io_num = boards->display.d1,
        .sclk_io_num = boards->display.sck,
        .data2_io_num = boards->display.d2,
        .data3_io_num = boards->display.d3,
        .data4_io_num = BOARD_NONE_PIN,
        .data5_io_num = BOARD_NONE_PIN,
        .data6_io_num = BOARD_NONE_PIN,
        .data7_io_num = BOARD_NONE_PIN,
        .max_transfer_sz = (SEND_BUF_SIZE * 16) + 8,
        .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS,
    };

    spi_device_interface_config_t devcfg = {
        .command_bits = boards->display.cmdBit,
        .address_bits = boards->display.addBit,
        .mode = TFT_SPI_MODE,
        .clock_speed_hz = _spiHzOverride > 0 ? _spiHzOverride : boards->display.freq,
        .spics_io_num = -1,
        .flags = SPI_DEVICE_HALFDUPLEX,
        .queue_size = 17,
    };
    esp_err_t ret = spi_bus_initialize(DEFAULT_SPI_HANDLER, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        log_e("spi_bus_initialize fail!");
        return false;
    }
    ret = spi_bus_add_device(DEFAULT_SPI_HANDLER, &devcfg, &spi);
    if (ret != ESP_OK) {
        log_e("spi_bus_add_device fail!");
        return false;
    }
    if (!_boot.splashFirst) {
        // prevent initialization failure
        int retry = _boot.singlePassInit ? 1 : 2;
        while (retry--) {
            const lcd_cmd_t *t = boards->display.initSequence;
            for (uint32_t i = 0; i < boards->display.initSize; i++) {
                writeCommand(t[i].addr, (uint8_t *)t[i].param, t[i].len & 0x1F);
                if (t[i].len & 0x80) {
                    delay(_boot.initDelayLongMs);
                }
                if (t[i].len & 0x20) {
                    delay(_boot.initDelayShortMs);
                }
            }
        }
    }
    return true;
}


bool LilyGo_AMOLED::begin()
{
    // If known board is provided, skip autodetection
    if (_boot.knownBoard != LILYGO_AMOLED_UNKOWN) {
        switch (_boot.knownBoard) {
        case LILYGO_AMOLED_147: return beginAMOLED_147();
        case LILYGO_AMOLED_191: return beginAMOLED_191(true);
        case LILYGO_AMOLED_241: return beginAMOLED_241();
        default: break;
        }
    }

    // Autodetection path (original behavior), with optional scan skips
    //Try find 1.47 inch i2c devices
    Wire.begin(1, 2);
    Wire.beginTransmission(AXP2101_SLAVE_ADDRESS);
    if (Wire.endTransmission() == 0) {
        return beginAMOLED_147();
    }

    log_e("Unable to detect 1.47-inch board model!");

    Wire.end();

    delay(10);

    // Try find 1.91 inch i2c devices
    Wire.begin(3, 2);
    Wire.beginTransmission(CSTXXX_SLAVE_ADDRESS);
    if (Wire.endTransmission() == 0) {
        return beginAMOLED_191(true);
    }
    log_e("Unable to detect 1.91-inch touch board model!");

    Wire.end();

    delay(10);

    // Try find 2.41 inch i2c devices
    Wire.begin(6, 7);
    Wire.beginTransmission(SY6970_SLAVE_ADDRESS);
    if (Wire.endTransmission() == 0) {
        return beginAMOLED_241();
    }
    log_e("Unable to detect 2.41-inch touch board model!");

    Wire.end();

    log_e("Begin 1.91-inch no touch board model");
    return beginAMOLED_191(false);
}


bool LilyGo_AMOLED::beginAutomatic()
{
    return begin();
}

bool LilyGo_AMOLED::beginAMOLED_191(bool touchFunc)
{
    boards = &BOARD_AMOLED_191;

    initBUS();

    if (_boot.splashFirst) {
        // Minimal bring-up: Sleep Out -> wait -> optional brightness -> Display ON
        lcd_cmd_t slp_out = {0x1100, {0x00}, 1};
        writeCommand(slp_out.addr, slp_out.param, slp_out.len);
        delay(_boot.initDelayLongMs);
        // Ensure RGB565 pixel format for immediate RAM writes
        lcd_cmd_t colmod1 = {0x3A00, {0x55}, 0x01};
        writeCommand(colmod1.addr, colmod1.param, colmod1.len);
        // Enable output at brightness 0 (screen stays black), write the
        // splash into RAM while nothing is lit, then raise brightness. This
        // avoids flashing random RAM contents at power-on — a worst-case
        // AMOLED current spike on the 5V rail — while respecting that this
        // panel drops RAM writes issued before Display ON.
        lcd_cmd_t briZero1 = {0x5100, {0x00}, 0x01};
        writeCommand(briZero1.addr, briZero1.param, briZero1.len);
        lcd_cmd_t disp_on = {0x2900, {0x00}, 1};
        writeCommand(disp_on.addr, disp_on.param, disp_on.len);
        drawQuickSplash();
        setBrightness(_boot.bootBrightness);
        startFinalizeInitAsync();
    }

    if (touchFunc && boards->touch && !_boot.deferTouch) {
        if (boards->touch->sda != -1 && boards->touch->scl != -1) {
            Wire.begin(boards->touch->sda, boards->touch->scl);
            if (!_boot.skipDeviceScan) {
                deviceScan(&Wire, &Serial);
            }

            // Try to find touch device
            Wire.beginTransmission(CST816_SLAVE_ADDRESS);
            if (Wire.endTransmission() == 0) {
                TouchDrvCSTXXX::setPins(boards->touch->rst, boards->touch->irq);
                bool res = TouchDrvCSTXXX::begin(Wire, CST816_SLAVE_ADDRESS, boards->touch->sda, boards->touch->scl);
                if (!res) {
                    log_e("Failed to find CST816T - check your wiring!");
                    return false;
                }
                TouchDrvCSTXXX::setMaxCoordinates(RM67162_HEIGHT, RM67162_WIDTH);
            }
        }
    }
    return true;
}


bool LilyGo_AMOLED::beginAMOLED_241()
{
    boards = &BOARD_AMOLED_241;

    initBUS();

    if (_boot.splashFirst) {
        lcd_cmd_t slp_out = {0x1100, {0x00}, 1};
        writeCommand(slp_out.addr, slp_out.param, slp_out.len);
        delay(_boot.initDelayLongMs);
        lcd_cmd_t colmod2 = {0x3A00, {0x55}, 0x01};
        writeCommand(colmod2.addr, colmod2.param, colmod2.len);
        // DISPON at brightness 0, splash into RAM, then raise (see AMOLED_191)
        lcd_cmd_t briZero2 = {0x5100, {0x00}, 0x01};
        writeCommand(briZero2.addr, briZero2.param, briZero2.len);
        lcd_cmd_t disp_on = {0x2900, {0x00}, 1};
        writeCommand(disp_on.addr, disp_on.param, disp_on.len);
        drawQuickSplash();
        setBrightness(_boot.bootBrightness);
        startFinalizeInitAsync();
    }

    if (boards->pmu) {
        Wire.begin(boards->pmu->sda, boards->pmu->scl);
        if (!_boot.skipDeviceScan) {
            deviceScan(&Wire, &Serial);
        }
        PowersSY6970::init(Wire, boards->pmu->sda, boards->pmu->scl, SY6970_SLAVE_ADDRESS);
        if (!_boot.deferPMUADC) {
            PowersSY6970::enableADCMeasure();
        }
        PowersSY6970::disableOTG();
    }

    if (boards->touch && !_boot.deferTouch) {
        // Try to find touch device
        Wire.beginTransmission(CST226SE_SLAVE_ADDRESS);
        if (Wire.endTransmission() == 0) {
            TouchDrvCSTXXX::setPins(boards->touch->rst, boards->touch->irq);
            bool res = TouchDrvCSTXXX::begin(Wire, CST226SE_SLAVE_ADDRESS, boards->touch->sda, boards->touch->scl);
            if (!res) {
                log_e("Failed to find CST226SE - check your wiring!");
                return false;
            }
            TouchDrvCSTXXX::setMaxCoordinates(RM690B0_HEIGHT, RM690B0_WIDTH);
        }
    }

    if (boards->sd && !_boot.deferSD) {
        SPI.begin(boards->sd->sck, boards->sd->miso, boards->sd->mosi);
        // Set mount point to /fs
        if (!SD.begin(boards->sd->cs, SPI, 4000000U, "/fs")) {
            log_e("Failed to dected SDCard!");
        }
        if (SD.cardType() != CARD_NONE) {
            log_i("SD Card Size: %llu MB\n", SD.cardSize() / (1024 * 1024));
        }
    }
    return true;
}


bool LilyGo_AMOLED::beginAMOLED_147()
{
    boards = &BOARD_AMOLED_147;

    if (!initPMU()) {
        log_e("Failed to find AXP2101 - check your wiring!");
        return false;
    }

    if ((ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO) && !_boot.skipDeviceScan) {
        deviceScan(&Wire, &Serial);
    }

    initBUS();

    if (_boot.splashFirst) {
        lcd_cmd_t slp_out = {0x1100, {0x00}, 1};
        writeCommand(slp_out.addr, slp_out.param, slp_out.len);
        delay(_boot.initDelayLongMs);
        lcd_cmd_t colmod3 = {0x3A00, {0x55}, 0x01};
        writeCommand(colmod3.addr, colmod3.param, colmod3.len);
        // DISPON at brightness 0, splash into RAM, then raise (see AMOLED_191)
        lcd_cmd_t briZero3 = {0x5100, {0x00}, 0x01};
        writeCommand(briZero3.addr, briZero3.param, briZero3.len);
        lcd_cmd_t disp_on = {0x2900, {0x00}, 1};
        writeCommand(disp_on.addr, disp_on.param, disp_on.len);
        drawQuickSplash();
        setBrightness(_boot.bootBrightness);
        startFinalizeInitAsync();
    }


    if (boards->display.frameBufferSize) {
        if (psramFound()) {
            pBuffer = (uint16_t *)ps_malloc(boards->display.frameBufferSize);
        } else {
            pBuffer = (uint16_t *)malloc(boards->display.frameBufferSize);
        }
        assert(pBuffer);
    }

    TouchDrvCHSC5816::setPins(boards->touch->rst, boards->touch->irq);
    bool res = TouchDrvCHSC5816::begin(Wire, CHSC5816_SLAVE_ADDRESS, boards->touch->sda, boards->touch->scl);
    if (!res) {
        log_e("Failed to find CHSC5816 - check your wiring!");
        return false;
    }
    TouchDrvCHSC5816::setMaxCoordinates(SH8501_HEIGHT, SH8501_WIDTH);
    TouchDrvCHSC5816::setSwapXY(true);
    TouchDrvCHSC5816::setMirrorXY(false, true);

    // Share I2C Bus
    res = SensorCM32181::begin(Wire, CM32181_SLAVE_ADDRESS, boards->sensor->sda, boards->sensor->scl);
    if (!res) {
        log_e("Failed to find CM32181 - check your wiring!");
        return false;
    }
    /*
        Sensitivity mode selection
            SAMPLING_X1
            SAMPLING_X2
            SAMPLING_X1_8
            SAMPLING_X1_4
    */
    SensorCM32181::setSampling(SensorCM32181::SAMPLING_X2),
                  powerOn();


    // Temperature detect
    beginCore();

    return true;
}

void LilyGo_AMOLED::writeCommand(uint32_t cmd, uint8_t *pdat, uint32_t lenght)
{
    setCS();
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.flags = (SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR);
    t.cmd = 0x02;
    t.addr = cmd;
    if (lenght != 0) {
        t.tx_buffer = pdat;
        t.length = 8 * lenght;
    } else {
        t.tx_buffer = NULL;
        t.length = 0;
    }
    spi_device_polling_transmit(spi, &t);
    clrCS();
}

void LilyGo_AMOLED::setBrightness(uint8_t level)
{
    _brightness = level;
    lcd_cmd_t t = {0x5100, {level}, 0x01};
    writeCommand(t.addr, t.param, t.len);
}

uint8_t LilyGo_AMOLED::getBrightness()
{
    return _brightness;
}

void LilyGo_AMOLED::setAddrWindow(uint16_t xs, uint16_t ys, uint16_t xe, uint16_t ye)
{
    lcd_cmd_t t[3] = {
        {
            0x2A00, {
                (uint8_t)((xs >> 8) & 0xFF),
                (uint8_t)(xs & 0xFF),
                (uint8_t)((xe >> 8) & 0xFF),
                (uint8_t)(xe & 0xFF)
            }, 0x04
        },
        {
            0x2B00, {
                (uint8_t)((ys >> 8) & 0xFF),
                (uint8_t)(ys & 0xFF),
                (uint8_t)((ye >> 8) & 0xFF),
                (uint8_t)(ye & 0xFF)
            }, 0x04
        },
        {
            0x2C00, {
                0x00
            }, 0x00
        },
    };

    for (uint32_t i = 0; i < 3; i++) {
        writeCommand(t[i].addr, t[i].param, t[i].len);
    }
}

// Push (aka write pixel) colours to the TFT (use setAddrWindow() first)
void LilyGo_AMOLED::pushColors(uint16_t *data, uint32_t len)
{
    bool first_send = true;
    uint16_t *p = data;
    assert(p);
    assert(spi);
    setCS();
    do {
        size_t chunk_size = len;
        spi_transaction_ext_t t = {0};
        memset(&t, 0, sizeof(t));
        if (first_send) {
            t.base.flags = SPI_TRANS_MODE_QIO;
            t.base.cmd = 0x32 ;
            t.base.addr = 0x002C00;
            first_send = 0;
        } else {
            t.base.flags = SPI_TRANS_MODE_QIO | SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY;
            t.command_bits = 0;
            t.address_bits = 0;
            t.dummy_bits = 0;
        }
        if (chunk_size > SEND_BUF_SIZE) {
            chunk_size = SEND_BUF_SIZE;
        }
        t.base.tx_buffer = p;
        t.base.length = chunk_size * 16;
        spi_device_polling_transmit(spi, (spi_transaction_t *)&t);
        len -= chunk_size;
        p += chunk_size;
    } while (len > 0);
    clrCS();
}

void LilyGo_AMOLED::pushColors(uint16_t x, uint16_t y, uint16_t width, uint16_t hight, uint16_t *data)
{

    if (boards->display.frameBufferSize) {
        assert(pBuffer);
        uint16_t _x = this->width() - (y + hight);
        uint16_t _y = x;
        uint16_t _h = width;
        uint16_t _w = hight;
        uint16_t *p = data;
        uint32_t cum = 0;
        for (uint16_t j = 0; j < width; j++) {
            for (uint16_t i = 0; i < hight; i++) {
                pBuffer[cum] = ((uint16_t)p[width * (hight - i - 1) + j]);
                cum++;
            }
        }
        setAddrWindow(_x, _y, _x + _w - 1, _y + _h - 1);
        pushColors(pBuffer, width * hight);
    } else {
        setAddrWindow(x, y, x + width - 1, y + hight - 1);
        pushColors(data, width * hight);
    }
}


void LilyGo_AMOLED::beginCore()
{
    // https://docs.espressif.com/projects/esp-idf/zh_CN/v4.4.4/esp32s3/api-reference/peripherals/temp_sensor.html
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5,0,0)
    temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
    temp_sensor_set_config(temp_sensor);
    temp_sensor_start();
#else
    // https://docs.espressif.com/projects/esp-idf/zh_CN/v5.0.1/esp32s3/api-reference/peripherals/temp_sensor.html
    static temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);
    temperature_sensor_install(&temp_sensor_config, &temp_sensor);
    temperature_sensor_enable(temp_sensor);
#endif
}


float LilyGo_AMOLED::readCoreTemp()
{
    float tsens_value;
    // https://docs.espressif.com/projects/esp-idf/zh_CN/v4.4.4/esp32s3/api-reference/peripherals/temp_sensor.html
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5,0,0)
    temp_sensor_read_celsius(&tsens_value);
#else
    // https://docs.espressif.com/projects/esp-idf/zh_CN/v5.0.1/esp32s3/api-reference/peripherals/temp_sensor.html
    temperature_sensor_get_celsius(temp_sensor, &tsens_value);
#endif
    return tsens_value;
}


void LilyGo_AMOLED::attachPMU(void(*cb)(void))
{
    if (boards) {
        if (boards->pmu && (boards == &BOARD_AMOLED_147)) {
            pinMode(BOARD_PMU_IRQ, INPUT_PULLUP);
            attachInterrupt(BOARD_PMU_IRQ, cb, FALLING);
        }
    }
}

uint64_t LilyGo_AMOLED::readPMU()
{
    if (boards) {
        if (boards->pmu && (boards == &BOARD_AMOLED_147)) {
            return XPowersAXP2101::getIrqStatus();
        }
    }
    return 0;
}

void LilyGo_AMOLED::clearPMU()
{
    if (boards) {
        if (boards->pmu && (boards == &BOARD_AMOLED_147)) {
            log_i("clearPMU");
            XPowersAXP2101::clearIrqStatus();
        }
    }
}

void LilyGo_AMOLED::enablePMUInterrupt(uint32_t params)
{
    if (boards) {
        if (boards->pmu && (boards == &BOARD_AMOLED_147)) {
            XPowersAXP2101::enableIRQ(params);
        }
    }
}
void LilyGo_AMOLED::diablePMUInterrupt(uint32_t params)
{
    if (boards) {
        if (boards->pmu && (boards == &BOARD_AMOLED_147)) {
            XPowersAXP2101::disableIRQ(params);
        }
    }
}


void LilyGo_AMOLED::sleep()
{
    assert(boards);

    //Wire amoled to sleep mode
    lcd_cmd_t t = {0x1000, {0x00}, 1}; //Sleep in
    writeCommand(t.addr, t.param, t.len);

    if (boards) {

        if (boards == &BOARD_AMOLED_241) {
            PowersSY6970::disableADCMeasure();
            PowersSY6970::disableOTG();

            // Disable amoled power
            digitalWrite(boards->PMICEnPins, LOW);
            TouchDrvCSTXXX::sleep();

        } else if (boards == &BOARD_AMOLED_147) {
            Serial.println("PMU Disbale AMOLED Power");

            // Turn off Sensor
            SensorCM32181::powerDown();

            // Turn off ADC data monitoring to save power
            disableTemperatureMeasure();
            disableBattDetection();
            disableVbusVoltageMeasure();
            disableBattVoltageMeasure();
            disableSystemVoltageMeasure();
            setChargingLedMode(XPOWERS_CHG_LED_OFF);

            // Disbale amoled power
            disableBLDO1();
            disableALDO3();

            // Don't turn off ALDO1
            // disableALDO1();

            // Keep touch reset to HIGH
            digitalWrite(boards->touch->rst, HIGH);
            gpio_hold_en((gpio_num_t )boards->touch->rst);
            gpio_deep_sleep_hold_en();
            // Enter sleep mode
            TouchDrvCHSC5816::sleep();

        } else {
            if (boards->PMICEnPins != -1) {
                // Disable amoled power
                digitalWrite(boards->PMICEnPins, LOW);
                TouchDrvCSTXXX::sleep();
            }
        }
    }
}

void LilyGo_AMOLED::wakeup()
{
    lcd_cmd_t t = {0x1100, {0x00}, 1};// Sleep Out
    writeCommand(t.addr, t.param, t.len);
}

bool LilyGo_AMOLED::hasTouch()
{
    if (boards) {
        if (boards->touch) {
            return true;
        }
    }
    return false;
}

// --- Fast boot controls ---
void LilyGo_AMOLED::setFastBoot(bool enable)
{
    _boot.fastBoot = enable;
    _boot.skipDeviceScan = enable;
    _boot.singlePassInit = enable;
    _boot.deferTouch = enable;
    _boot.deferSD = enable;
    _boot.deferPMUADC = enable;
    _boot.splashFirst = enable; // enable splash-first by default when fast boot is on
    if (enable) {
        // Ultra-fast defaults (validate on hardware; revert if unstable)
        _boot.resetDelayHigh1Ms = 20;
        _boot.resetDelayLowMs   = 10;
        _boot.resetDelayHigh2Ms = 20;
        _boot.initDelayLongMs   = 20;  // Sleep Out wait
        _boot.initDelayShortMs  = 1;   // command settle
    } else {
        _boot.resetDelayHigh1Ms = 200;
        _boot.resetDelayLowMs   = 300;
        _boot.resetDelayHigh2Ms = 200;
        _boot.initDelayLongMs   = 120;
        _boot.initDelayShortMs  = 10;
    }
}

void LilyGo_AMOLED::setSplashFirst(bool enable)
{
    _boot.splashFirst = enable;
}

void LilyGo_AMOLED::setBootBrightness(uint8_t level)
{
    _boot.bootBrightness = level;
}

bool LilyGo_AMOLED::finalizeDone()
{
    return _finalized;
}

void LilyGo_AMOLED::redrawSplash()
{
    if (!boards) {
        return;
    }
    // The splash pixel array is portrait-oriented (panel power-on default
    // addressing). The finalize replay may have switched MADCTL to the
    // orientation LVGL uses, so force portrait for the draw, then restore
    // whatever MADCTL value the board's init sequence programs.
    uint8_t madctlPortrait = 0x00;
    writeCommand(0x3600, &madctlPortrait, 0x01);
    drawQuickSplash();
    const lcd_cmd_t *t = boards->display.initSequence;
    for (uint32_t i = 0; i < boards->display.initSize; i++) {
        if (t[i].addr == 0x3600) {
            writeCommand(t[i].addr, (uint8_t *)t[i].param, t[i].len & 0x1F);
        }
    }
    setBrightness(_brightness);
}

void LilyGo_AMOLED::setKnownBoardID(AmoledBoardID id)
{
    _boot.knownBoard = id;
}

bool LilyGo_AMOLED::initTouchNow()
{
    if (!boards || !boards->touch) return false;

    if (boards == &BOARD_AMOLED_191) {
        Wire.begin(boards->touch->sda, boards->touch->scl);
        Wire.beginTransmission(CST816_SLAVE_ADDRESS);
        if (Wire.endTransmission() == 0) {
            TouchDrvCSTXXX::setPins(boards->touch->rst, boards->touch->irq);
            bool res = TouchDrvCSTXXX::begin(Wire, CST816_SLAVE_ADDRESS, boards->touch->sda, boards->touch->scl);
            if (!res) return false;
            TouchDrvCSTXXX::setMaxCoordinates(RM67162_HEIGHT, RM67162_WIDTH);
            return true;
        }
        return false;
    }
    if (boards == &BOARD_AMOLED_241) {
        Wire.begin(boards->touch->sda, boards->touch->scl);
        Wire.beginTransmission(CST226SE_SLAVE_ADDRESS);
        if (Wire.endTransmission() == 0) {
            TouchDrvCSTXXX::setPins(boards->touch->rst, boards->touch->irq);
            bool res = TouchDrvCSTXXX::begin(Wire, CST226SE_SLAVE_ADDRESS, boards->touch->sda, boards->touch->scl);
            if (!res) return false;
            TouchDrvCSTXXX::setMaxCoordinates(RM690B0_HEIGHT, RM690B0_WIDTH);
            return true;
        }
        return false;
    }
    if (boards == &BOARD_AMOLED_147) {
        // Already initialized in beginAMOLED_147
        return true;
    }
    return false;
}

bool LilyGo_AMOLED::initSDNow()
{
    if (!boards || !boards->sd) return false;
    if (boards == &BOARD_AMOLED_241) {
        SPI.begin(boards->sd->sck, boards->sd->miso, boards->sd->mosi);
        if (!SD.begin(boards->sd->cs, SPI, 4000000U, "/fs")) {
            return false;
        }
        return true;
    }
    return false;
}

bool LilyGo_AMOLED::enablePMUADCNow()
{
    if (!boards || !boards->pmu) return false;
    if (boards == &BOARD_AMOLED_241) {
        PowersSY6970::enableADCMeasure();
        return true;
    }
    return false;
}

bool LilyGo_AMOLED::startFinalizeInitAsync()
{
    if (_finalizeScheduled || _finalized) return false;
    _finalizeScheduled = true;
    BaseType_t ok = xTaskCreatePinnedToCore(
                        finalizeTaskTrampoline,
                        "amoled_final",
                        4096,
                        this,
                        1,
                        &_finalizeTaskHandle,
                        APP_CPU_NUM
                    );
    return ok == pdPASS;
}

void LilyGo_AMOLED::finalizeTaskTrampoline(void *arg)
{
    reinterpret_cast<LilyGo_AMOLED *>(arg)->finalizeInitSequenceTask();
}

void LilyGo_AMOLED::finalizeInitSequenceTask()
{
    if (!boards) {
        _finalized = true;
        vTaskDelete(NULL);
        return;
    }
    const lcd_cmd_t *t = boards->display.initSequence;
    for (uint32_t i = 0; i < boards->display.initSize; i++) {
        if (t[i].addr == 0x5100) {
            // The stock sequence hardcodes brightness values (0, then the
            // default); with splash-first the app owns brightness (dim boot +
            // ramp), so replay the current level instead of stomping it.
            uint8_t b = _brightness;
            writeCommand(t[i].addr, &b, 0x01);
        } else {
            writeCommand(t[i].addr, (uint8_t *)t[i].param, t[i].len & 0x1F);
        }
        if (t[i].len & 0x80) {
            vTaskDelay(pdMS_TO_TICKS(_boot.initDelayLongMs));
        }
        if (t[i].len & 0x20) {
            vTaskDelay(pdMS_TO_TICKS(_boot.initDelayShortMs));
        }
    }
    // Deliberately no drawing from this task: the SPI path has no lock and
    // pushColors holds CS across chunked transactions, so all pixel writes
    // must come from the main/LVGL task. Cold-boot splash recovery is done
    // there via redrawSplash() once _finalized flips.
    _finalized = true;
    vTaskDelete(NULL);
}

void LilyGo_AMOLED::drawQuickSplash()
{
    uint16_t cx = width() / 2;
    uint16_t cy = height() / 2;
    uint16_t x = (cx > LOGO_WIDTH/2) ? (cx - LOGO_WIDTH/2) : 0;
    uint16_t y = (cy > LOGO_HEIGHT/2) ? (cy - LOGO_HEIGHT/2) : 0;
    pushColors(x, y, LOGO_WIDTH, LOGO_HEIGHT, (uint16_t*)logo);
}

void LilyGo_AMOLED::tuneResetDelays(uint16_t high1Ms, uint16_t lowMs, uint16_t high2Ms)
{
    _boot.resetDelayHigh1Ms = high1Ms;
    _boot.resetDelayLowMs = lowMs;
    _boot.resetDelayHigh2Ms = high2Ms;
}

void LilyGo_AMOLED::tuneInitDelays(uint16_t longMs, uint16_t shortMs)
{
    _boot.initDelayLongMs = longMs;
    _boot.initDelayShortMs = shortMs;
}

void LilyGo_AMOLED::setDisplaySpiHz(int hz)
{
    _spiHzOverride = hz;
}


