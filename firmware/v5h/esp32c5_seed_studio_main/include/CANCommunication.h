// include/CANCommunication.h
//
// moteus-x1 register protocol over the ESP32-C5's native TWAI-FD controller.
// Replaces the v5h ODrive CANSimple layer: the legacy driver/twai.h API does
// not exist on the C5 (compiled out on FD-capable chips) — this uses the
// ESP-IDF 5.5 node API (esp_twai_onchip.h) that ships inside Arduino 3.3.x.
//
// Bus: 1 Mbps arbitration / 5 Mbps data, sample point 0.666 in both phases
// (moteus requires 0.666; its rates are fixed in firmware). If 5 M proves
// marginal, clear BRS via setCANUseBRS(false) — moteus mirrors the query's
// BRS flag, so the whole bus drops to 1 Mbps while keeping FD frames.
#ifndef CAN_COMMUNICATION_H
#define CAN_COMMUNICATION_H

#include <Arduino.h>

// XIAO ESP32-C5: pad D8 = GPIO8 -> CAN Pal TX, pad D3 = GPIO7 -> CAN Pal RX
// (same physical pads as the old S3 harness). GPIO7 is a strapping pin, but
// the TJA1051's push-pull RXD holds it at a firm recessive-high through
// reset, and stock eFuses ignore that strap (JTAG source select).
#define CAN_TX_PIN 8
#define CAN_RX_PIN 7

// moteus servo ids are 1..126. A fresh moteus boots as id 1; ours is left
// there — set via moteus_tool + `conf write`.
// The host (source) id is NOT 0 on purpose: tview/moteus_tool/fdcanusb use
// source 0, and two hosts sharing a source id transmit identical arbitration
// IDs — which CAN cannot arbitrate — and cross-read each other's replies.
// With a distinct id the C5 and the fdcanusb coexist on the bus, so tview
// can watch live while the machine runs.
#define MOTEUS_ID_M1 1
#define MOTEUS_HOST_ID 4

// moteus mode register (0x000) values used here
#define MOTEUS_MODE_STOPPED 0
#define MOTEUS_MODE_FAULT 1
#define MOTEUS_MODE_POSITION 10
#define MOTEUS_MODE_TIMEOUT 11

// One parsed reply frame from the moteus. Position/velocity/torque arrive on
// every control-cycle query; the telemetry fields only on the 1 Hz query
// (NaN / has_telemetry=false otherwise).
struct MoteusReply
{
    uint8_t servo_id;
    float position;    // output revolutions (== ODrive turns on this drivetrain)
    float velocity;    // rev/s
    float torque;      // Nm
    bool has_feedback; // position/velocity/torque present in this frame
    float voltage;     // bus V
    float temperature; // board degC
    float q_current;   // phase Q current, A (screen's "current" readout)
    int8_t mode;       // moteus mode register
    int8_t fault;      // fault code (0 = none)
    bool has_telemetry;
};

bool initCAN();

// The per-control-tick frame: position mode with position=NaN so the axis
// chases `velocityRevS` forever, saturated at maxTorqueNm — the cap IS the
// rendered force (same trick as the ODrive velocity-mode + torque-limit
// setup). kp/kd scales are left at their default 1.0 (registers not sent).
// query=true solicits a position/velocity/torque reply.
bool sendMoteusPositionCommand(uint8_t servoId, float velocityRevS, float maxTorqueNm, bool query);

// Mode 0 = stopped: axis freewheels (zero current); also clears latched faults.
bool sendMoteusStop(uint8_t servoId);

// Query-only frame (no mode write): position/velocity/torque while the axis
// stays stopped — used for wake-from-sleep movement detection.
bool sendMoteusQuery(uint8_t servoId);

// Low-rate telemetry query: voltage, board temp, Q current, mode, fault.
bool sendMoteusTelemetryQuery(uint8_t servoId);

// Drain one parsed reply (non-blocking); false if none pending.
bool receiveMoteusReply(MoteusReply &out);

// Per-frame bit-rate-switch control. Default true (5 Mbps data phase).
void setCANUseBRS(bool on);
bool getCANUseBRS();

// Maintenance mute: while quiet, the C5 transmits NOTHING on the bus (all
// sends no-op silently). Toggled via the WiFi/HTTP cal command
// {"can_quiet":true/false}. BRING-UP PHASE: the boot default is MUTED (see
// CANCommunication.cpp) so calibration/tview own the bus until the C5 is
// explicitly unmuted each boot; flip that default for production. RAM only.
void setCANQuiet(bool on);
bool getCANQuiet();

// Cumulative counters for the heartbeat line: frames accepted for transmit,
// TX failures (queue full / bus errors), and frames received off the bus.
uint32_t getCANTxOkCount();
uint32_t getCANTxDropCount();
uint32_t getCANRxCount();

#endif // CAN_COMMUNICATION_H
