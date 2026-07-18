// src/CANCommunication.cpp
//
// moteus register (multiplex) protocol over the ESP32-C5 TWAI-FD controller,
// using the ESP-IDF 5.5 node API. Protocol reference:
// https://mjbots.github.io/moteus/protocol/can/ and protocol/registers/.
//
// Frame ID format: (source << 8) | destination, with bit 15 (0x8000) set
// when a reply is requested. Anything above 0x7FF needs a 29-bit frame, so
// every frame we send is extended. The reply comes back as
// (servo << 8) | host with no reply bit (0x100 for servo 1 -> host 0).
#include "CANCommunication.h"
#include "ErrorHandling.h"
#include "Logging.h"

#include <math.h>
#include <string.h>

#include "esp_twai.h"
#include "esp_twai_onchip.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// ---- moteus multiplex subframe types ----
// Low 2 bits carry a register count of 1..3 (0 = varuint count follows).
#define MP_WRITE_INT8 0x00
#define MP_WRITE_INT16 0x04
#define MP_WRITE_INT32 0x08
#define MP_WRITE_FLOAT 0x0C
#define MP_READ_INT8 0x10
#define MP_READ_INT16 0x14
#define MP_READ_INT32 0x18
#define MP_READ_FLOAT 0x1C
#define MP_REPLY_INT8 0x20
#define MP_REPLY_INT16 0x24
#define MP_REPLY_INT32 0x28
#define MP_REPLY_FLOAT 0x2C
#define MP_WRITE_ERROR 0x30
#define MP_READ_ERROR 0x31
#define MP_NOP 0x50

// ---- moteus registers used here ----
#define REG_MODE 0x000
#define REG_POSITION 0x001
#define REG_VELOCITY 0x002
#define REG_TORQUE 0x003
#define REG_Q_CURRENT 0x004
#define REG_VOLTAGE 0x00D
#define REG_TEMPERATURE 0x00E
#define REG_FAULT 0x00F
#define REG_CMD_POSITION 0x020
#define REG_CMD_VELOCITY 0x021
#define REG_CMD_MAX_TORQUE 0x025

static twai_node_handle_t s_node = nullptr;
static QueueHandle_t s_rxQueue = nullptr;
static volatile bool s_useBRS = true;
// Boots live. For bench sessions that need the bus (moteus calibration,
// hand-driving from tview), mute first with cal {"can_quiet":true} — the
// calibration script sniffs the bus and reminds you if it's forgotten.
static volatile bool s_quiet = false;
static volatile uint32_t s_txDrops = 0;
static volatile uint32_t s_txOk = 0;
static volatile uint32_t s_rxCount = 0;

// Raw frame captured in the RX ISR; parsed in task context.
struct RawFrame
{
    uint32_t id;
    uint8_t len;
    uint8_t data[64];
};

// The node driver references the caller's frame/buffer until the transmit
// completes, so TX payloads live in a static ring instead of on the stack.
// Only the motor control task transmits, so a bare index is safe.
#define TX_RING 8
static uint8_t s_txBuf[TX_RING][64];
static twai_frame_t s_txFrames[TX_RING];
static uint8_t s_txIdx = 0;

// CAN-FD DLC <-> payload length (no dependency on HAL helper names)
static uint16_t fdDlcToLen(uint8_t dlc)
{
    static const uint16_t table[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    return table[dlc & 0x0F];
}

static uint8_t fdLenToDlc(uint16_t len)
{
    if (len <= 8) return (uint8_t)len;
    if (len <= 12) return 9;
    if (len <= 16) return 10;
    if (len <= 20) return 11;
    if (len <= 24) return 12;
    if (len <= 32) return 13;
    if (len <= 48) return 14;
    return 15;
}

static bool IRAM_ATTR onRxDone(twai_node_handle_t node, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    (void)edata;
    (void)user_ctx;
    BaseType_t hpTaskWoken = pdFALSE;
    RawFrame raw;
    twai_frame_t frame = {};
    frame.buffer = raw.data;
    frame.buffer_len = sizeof(raw.data);
    if (twai_node_receive_from_isr(node, &frame) == ESP_OK)
    {
        raw.id = frame.header.id;
        uint16_t len = fdDlcToLen(frame.header.dlc);
        raw.len = (len > sizeof(raw.data)) ? sizeof(raw.data) : (uint8_t)len;
        s_rxCount = s_rxCount + 1;
        if (s_rxQueue)
        {
            xQueueSendFromISR(s_rxQueue, &raw, &hpTaskWoken);
        }
    }
    return hpTaskWoken == pdTRUE;
}

bool initCAN()
{
    s_rxQueue = xQueueCreate(16, sizeof(RawFrame));
    if (!s_rxQueue)
    {
        handleError(ERROR_CAN_INIT, "Failed to create CAN RX queue");
        return false;
    }

    twai_onchip_node_config_t cfg = {};
    cfg.io_cfg.tx = (gpio_num_t)CAN_TX_PIN;
    cfg.io_cfg.rx = (gpio_num_t)CAN_RX_PIN;
    cfg.io_cfg.quanta_clk_out = GPIO_NUM_NC;
    cfg.io_cfg.bus_off_indicator = GPIO_NUM_NC;
    // moteus timing is fixed: 1 Mbps arbitration / 5 Mbps data, and it
    // requires a 0.666 sample point in both phases. The secondary sample
    // point enables transmitter delay compensation for the 5 M data phase.
    cfg.bit_timing.bitrate = 1000000;
    cfg.bit_timing.sp_permill = 666;
    cfg.data_timing.bitrate = 5000000;
    cfg.data_timing.sp_permill = 666;
    cfg.data_timing.ssp_permill = 666;
    cfg.fail_retry_cnt = -1; // retransmit on error like classic CAN
    cfg.tx_queue_depth = TX_RING;

    if (twai_new_node_onchip(&cfg, &s_node) != ESP_OK)
    {
        handleError(ERROR_CAN_INIT, "Failed to create TWAI-FD node");
        s_node = nullptr;
        return false;
    }

    twai_event_callbacks_t cbs = {};
    cbs.on_rx_done = onRxDone;
    if (twai_node_register_event_callbacks(s_node, &cbs, nullptr) != ESP_OK)
    {
        handleError(ERROR_CAN_INIT, "Failed to register TWAI callbacks");
        return false;
    }
    if (twai_node_enable(s_node) != ESP_OK)
    {
        handleError(ERROR_CAN_INIT, "Failed to enable TWAI node");
        return false;
    }
    logMessage(LOG_INFO, "TWAI-FD node started: 1M/5M, sample point 0.666.");
    if (s_quiet)
    {
        logMessage(LOG_WARN, "CAN boots MUTED (bring-up default) — send cal {\"can_quiet\":false} to go live");
    }
    return true;
}

void setCANUseBRS(bool on)
{
    s_useBRS = on;
    logMessagef(LOG_INFO, "CAN-FD bit rate switch %s (%s data phase)",
                on ? "ON" : "OFF", on ? "5 Mbps" : "1 Mbps");
}

bool getCANUseBRS()
{
    return s_useBRS;
}

uint32_t getCANTxOkCount()
{
    return s_txOk;
}

uint32_t getCANTxDropCount()
{
    return s_txDrops;
}

uint32_t getCANRxCount()
{
    return s_rxCount;
}

void setCANQuiet(bool on)
{
    s_quiet = on;
    logMessagef(LOG_WARN, "CAN maintenance mute %s%s", on ? "ON" : "OFF",
                on ? " — no frames will be sent until can_quiet:false or reboot" : "");
}

bool getCANQuiet()
{
    return s_quiet;
}

static size_t putVaruint(uint8_t *p, uint32_t v)
{
    size_t n = 0;
    do
    {
        uint8_t b = v & 0x7F;
        v >>= 7;
        if (v)
        {
            b |= 0x80;
        }
        p[n++] = b;
    } while (v);
    return n;
}

static size_t putFloat(uint8_t *p, float v)
{
    memcpy(p, &v, sizeof(float));
    return sizeof(float);
}

// Cap blocking at ~2 ms; if the queue stays full, drop and log once per
// second (same policy as the old ODrive transmit path).
static bool transmitFrame(uint32_t id, const uint8_t *payload, uint8_t len)
{
    if (!s_node)
    {
        return false;
    }
    if (s_quiet)
    {
        // Maintenance mute: single choke point for ALL outbound frames.
        // Silent no-op — no drop counting or logging while deliberately off.
        return false;
    }
    uint8_t idx = s_txIdx;
    s_txIdx = (s_txIdx + 1) % TX_RING;

    uint8_t dlc = fdLenToDlc(len);
    uint16_t padded = fdDlcToLen(dlc);
    memcpy(s_txBuf[idx], payload, len);
    // moteus wants trailing padding to be NOP subframes, not zeros
    memset(s_txBuf[idx] + len, MP_NOP, padded - len);

    twai_frame_t *f = &s_txFrames[idx];
    memset(f, 0, sizeof(*f));
    f->header.id = id;
    f->header.ide = true;
    f->header.fdf = true;
    f->header.brs = s_useBRS;
    f->header.dlc = dlc;
    f->buffer = s_txBuf[idx];
    f->buffer_len = padded;

    if (twai_node_transmit(s_node, f, 2) != ESP_OK)
    {
        s_txDrops = s_txDrops + 1;
        static unsigned long lastLogMs = 0;
        unsigned long now = millis();
        if (now - lastLogMs > 1000)
        {
            handleError(ERROR_CAN_TRANSMIT, "CAN TX timeout/drop (queue congested or bus down)");
            lastLogMs = now;
        }
        return false;
    }
    s_txOk = s_txOk + 1;
    return true;
}

static uint32_t frameId(uint8_t servoId, bool query)
{
    uint32_t id = ((uint32_t)MOTEUS_HOST_ID << 8) | servoId;
    if (query)
    {
        id |= 0x8000;
    }
    return id;
}

bool sendMoteusPositionCommand(uint8_t servoId, float velocityRevS, float maxTorqueNm, bool query)
{
    uint8_t buf[24];
    size_t n = 0;
    // Mode = position. Writing mode first each frame is the documented
    // pattern; it is a no-op when the axis is already in position mode.
    buf[n++] = MP_WRITE_INT8 | 0x01;
    n += putVaruint(&buf[n], REG_MODE);
    buf[n++] = MOTEUS_MODE_POSITION;
    // position = NaN ("start from wherever the output is right now"),
    // velocity = the chase setpoint. The target re-anchors every frame, so
    // the axis pursues velocity forever without position-error windup.
    buf[n++] = MP_WRITE_FLOAT | 0x02;
    n += putVaruint(&buf[n], REG_CMD_POSITION);
    n += putFloat(&buf[n], NAN);
    n += putFloat(&buf[n], velocityRevS);
    // maximum torque = the rendered force (kp/kd scales stay at default 1.0)
    buf[n++] = MP_WRITE_FLOAT | 0x01;
    n += putVaruint(&buf[n], REG_CMD_MAX_TORQUE);
    n += putFloat(&buf[n], maxTorqueNm);
    if (query)
    {
        buf[n++] = MP_READ_FLOAT | 0x03;
        n += putVaruint(&buf[n], REG_POSITION); // position, velocity, torque
    }
    return transmitFrame(frameId(servoId, query), buf, n);
}

bool sendMoteusStop(uint8_t servoId)
{
    uint8_t buf[8];
    size_t n = 0;
    buf[n++] = MP_WRITE_INT8 | 0x01;
    n += putVaruint(&buf[n], REG_MODE);
    buf[n++] = MOTEUS_MODE_STOPPED;
    // Keep the feedback stream alive across the transition
    buf[n++] = MP_READ_FLOAT | 0x03;
    n += putVaruint(&buf[n], REG_POSITION);
    return transmitFrame(frameId(servoId, true), buf, n);
}

bool sendMoteusQuery(uint8_t servoId)
{
    uint8_t buf[4];
    size_t n = 0;
    buf[n++] = MP_READ_FLOAT | 0x03;
    n += putVaruint(&buf[n], REG_POSITION); // position, velocity, torque
    return transmitFrame(frameId(servoId, true), buf, n);
}

bool sendMoteusTelemetryQuery(uint8_t servoId)
{
    uint8_t buf[12];
    size_t n = 0;
    buf[n++] = MP_READ_FLOAT | 0x02;
    n += putVaruint(&buf[n], REG_VOLTAGE); // voltage, temperature
    buf[n++] = MP_READ_FLOAT | 0x01;
    n += putVaruint(&buf[n], REG_Q_CURRENT);
    buf[n++] = MP_READ_INT8 | 0x01;
    n += putVaruint(&buf[n], REG_MODE);
    buf[n++] = MP_READ_INT8 | 0x01;
    n += putVaruint(&buf[n], REG_FAULT);
    return transmitFrame(frameId(servoId, true), buf, n);
}

static bool getVaruint(const uint8_t *data, uint8_t len, size_t &i, uint32_t &out)
{
    out = 0;
    int shift = 0;
    while (i < len)
    {
        uint8_t b = data[i++];
        out |= (uint32_t)(b & 0x7F) << shift;
        if (!(b & 0x80))
        {
            return true;
        }
        shift += 7;
        if (shift > 28)
        {
            return false;
        }
    }
    return false;
}

// Assign one decoded reply value into the MoteusReply. Only registers we
// actually query are mapped; anything else is parsed and dropped.
static void assignValue(MoteusReply &out, uint32_t reg, float fval, int32_t ival, bool isFloat)
{
    switch (reg)
    {
    case REG_POSITION:
        out.position = fval;
        out.has_feedback = true;
        break;
    case REG_VELOCITY:
        out.velocity = fval;
        break;
    case REG_TORQUE:
        out.torque = fval;
        break;
    case REG_Q_CURRENT:
        out.q_current = fval;
        out.has_telemetry = true;
        break;
    case REG_VOLTAGE:
        out.voltage = fval;
        out.has_telemetry = true;
        break;
    case REG_TEMPERATURE:
        out.temperature = fval;
        break;
    case REG_MODE:
        out.mode = (int8_t)(isFloat ? (int32_t)fval : ival);
        out.has_telemetry = true;
        break;
    case REG_FAULT:
        out.fault = (int8_t)(isFloat ? (int32_t)fval : ival);
        out.has_telemetry = true;
        break;
    default:
        break;
    }
}

static bool parseReply(const RawFrame &raw, MoteusReply &out)
{
    // Accept only frames addressed from our servo to us
    if ((raw.id & 0xFFFF) != (((uint32_t)MOTEUS_ID_M1 << 8) | MOTEUS_HOST_ID))
    {
        return false;
    }
    out = MoteusReply{};
    out.servo_id = MOTEUS_ID_M1;
    out.voltage = NAN;
    out.temperature = NAN;
    out.q_current = NAN;
    out.mode = -1;

    size_t i = 0;
    while (i < raw.len)
    {
        uint8_t type = raw.data[i++];
        if (type == MP_NOP)
        {
            continue;
        }
        uint8_t base = type & ~0x03;
        uint32_t count = type & 0x03;
        if (base == MP_REPLY_INT8 || base == MP_REPLY_INT16 ||
            base == MP_REPLY_INT32 || base == MP_REPLY_FLOAT)
        {
            if (count == 0 && !getVaruint(raw.data, raw.len, i, count))
            {
                return false;
            }
            uint32_t reg = 0;
            if (!getVaruint(raw.data, raw.len, i, reg))
            {
                return false;
            }
            size_t itemSize = (base == MP_REPLY_INT8) ? 1 : (base == MP_REPLY_INT16) ? 2 : 4;
            for (uint32_t k = 0; k < count; k++)
            {
                if (i + itemSize > raw.len)
                {
                    return false;
                }
                if (base == MP_REPLY_FLOAT)
                {
                    float v;
                    memcpy(&v, &raw.data[i], sizeof(float));
                    assignValue(out, reg + k, v, 0, true);
                }
                else
                {
                    int32_t v = 0;
                    if (base == MP_REPLY_INT8)
                    {
                        v = (int8_t)raw.data[i];
                    }
                    else if (base == MP_REPLY_INT16)
                    {
                        int16_t tmp;
                        memcpy(&tmp, &raw.data[i], sizeof(tmp));
                        v = tmp;
                    }
                    else
                    {
                        memcpy(&v, &raw.data[i], sizeof(v));
                    }
                    assignValue(out, reg + k, 0.0f, v, false);
                }
                i += itemSize;
            }
        }
        else if (type == MP_WRITE_ERROR || type == MP_READ_ERROR)
        {
            uint32_t reg = 0, err = 0;
            if (!getVaruint(raw.data, raw.len, i, reg) || !getVaruint(raw.data, raw.len, i, err))
            {
                return false;
            }
            logMessagef(LOG_WARN, "moteus %s error: reg=0x%03X err=%u",
                        (type == MP_WRITE_ERROR) ? "write" : "read", (unsigned)reg, (unsigned)err);
        }
        else
        {
            // Unknown subframe: bail rather than misparse the remainder
            return out.has_feedback || out.has_telemetry;
        }
    }
    return out.has_feedback || out.has_telemetry;
}

bool receiveMoteusReply(MoteusReply &out)
{
    if (!s_rxQueue)
    {
        return false;
    }
    RawFrame raw;
    while (xQueueReceive(s_rxQueue, &raw, 0) == pdTRUE)
    {
        if (parseReply(raw, out))
        {
            return true;
        }
    }
    return false;
}
