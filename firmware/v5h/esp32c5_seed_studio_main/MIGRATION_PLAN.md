# v5h Main Board Migration Plan — XIAO ESP32-S3 → XIAO ESP32-C5, ODrive S1 → moteus-x1

Date: 2026-07-12. Status as of 2026-07-17: **BRING-UP COMPLETE — working
machine.** Strength and row modes live on the new architecture; see the
session log at the bottom of this file for the full state of the world.
Build from native PowerShell/cmd — NOT Git Bash: ESP-IDF's tool installer
aborts in MSys environments ("MSys/Mingw is not supported").

Why: the ESP32-C5 has native CAN-FD (2× TWAI-FD controllers, CTU CAN FD IP), which the
mjbots moteus-x1 requires (CAN-FD only, 1 Mbps arbitration / 5 Mbps data, not configurable
on the controller). Transceiver: Adafruit CAN Pal (TJA1051T/3) — rated for 5 Mbit/s CAN-FD
data phase per the current NXP datasheet (Rev. 8/9, 2016+; the "2 Mbit/s" figure is from
pre-2016 datasheet revisions of the same silicon). mjbots themselves recommend TJA1051T/3.

---

## 1. Pin map (Seeed XIAO ESP32-C5)

Physical pads are kept identical to the current XIAO ESP32-S3 harness wherever possible.

| XIAO pad | C5 GPIO | Function | Connects to | Old S3 GPIO (same pad) |
|---|---|---|---|---|
| **D3**  | GPIO7  | TWAI-FD RX | CAN Pal `RX` | GPIO4 (CAN RX) |
| **D8**  | GPIO8  | TWAI-FD TX | CAN Pal `TX` | GPIO7 (CAN TX) |
| **D4**  | GPIO23 | I2C0 SDA — **slave @ addr 8** | Display board SDA (LilyGo GPIO43) | GPIO5 (SDA) |
| **D5**  | GPIO24 | I2C0 SCL — **slave @ addr 8** | Display board SCL (LilyGo GPIO44) | GPIO6 (SCL) |
| **rear pad MTMS** | GPIO2 | LP-I2C SDA — master | BMI270 Qwiic SDA (blue) | (new) |
| **rear pad MTDI** | GPIO3 | LP-I2C SCL — master | BMI270 Qwiic SCL (yellow) | (new) |
| rear pad 3V3 | — | 3.3 V | BMI270 Qwiic 3V3 (red) | |
| rear pad GND | — | GND | BMI270 Qwiic GND (black) | |
| 3V3-OUT (front) | — | 3.3 V | CAN Pal `Vcc` | |
| GND (front) | — | GND | CAN Pal `GND` + display GND | |

Free for future use: D0 (GPIO1, ADC1_CH0), D1 (GPIO0), D2 (GPIO25), D6 (GPIO11/U0TX),
D7 (GPIO12/U0RX), D9 (GPIO9), D10 (GPIO10). Board-reserved: GPIO27 user LED (active-low),
GPIO28 BOOT button, GPIO6 battery ADC + GPIO26 battery-sense enable, GPIO13/14 USB.

### Why these pins

- **CAN on D3/D8 (GPIO7/GPIO8):** same pads as the old harness. TWAI-FD routes to any GPIO
  via the GPIO matrix — no dedicated pins. GPIO8 is *not* a strapping pin on the C5.
  GPIO7 *is* a strapping pin (JTAG signal source, no internal pulls), but: (a) with stock
  eFuses the strap is ignored (USB-Serial-JTAG stays default), and (b) the TJA1051's RXD
  output is push-pull and idles high (bus recessive), which gives the pin a firm, defined
  level through reset — it satisfies the datasheet's "must not float" requirement. Do not
  ever move CAN RX to GPIO28 (BOOT): a dominant bit during reset would enter download mode.
- **Display link on D4/D5 (GPIO23/24):** the C5's only high-performance I2C controller
  (I2C0) stays in slave mode at address 8, so the display board firmware, protocol, and
  harness are 100 % unchanged. These are also the C5's default `Wire` pins.
- **IMU on GPIO2/GPIO3 (rear MTMS/MTDI pads):** the C5 has **only one HP I2C controller**
  (datasheet §4.2.1.3) — the S3's "slave-to-display + master-to-IMU on two hardware buses"
  design cannot map 1:1. The LP (low-power) I2C controller is a hardware master fixed to
  LP_GPIO2/3 = GPIO2/3, which on the XIAO exist only as the rear JTAG through-hole pads.
  Conveniently, 3V3 and GND pads sit in the same rear cluster, so the whole Qwiic pigtail
  lands in one place. GPIO2/3 are strapping pins, but the BMI270's 2.2 kΩ pull-ups holding
  them high at boot is harmless (GPIO2: JTAG-related, GPIO3: SDIO clock-edge — neither used).
  - Software path A (try first): `Wire1` — on the C5 Arduino core it maps to the LP-I2C.
  - Software path B: ESP-IDF `i2c_new_master_bus()` with `.i2c_port = LP_I2C_NUM_0`,
    `.clk_source = LP_I2C_SCLK_DEFAULT`, wired into the Bosch bmi2 read/write fptrs.
  - Software path C (last resort, **same wiring**): bit-bang I2C on GPIO2/3 — they are
    normal GPIOs after boot, so the harness is robust to any software outcome.
  - **No-rear-solder alternative (decided against, but valid):** bit-bang I2C on front
    pads D9/GPIO9 (SCL) + D10/GPIO10 (SDA) — the same pads the S3 harness used for the
    IMU. Master-side bit-bang tolerates preemption (transactions stretch, never corrupt),
    so at low task priority it can't disturb the control loop; ~4 % CPU at 400 kHz /
    100 Hz reads. Requires driving the Bosch bmi2 API with custom read/write fptrs
    (SparkFun's class wants a `TwoWire&`). Rear pads were chosen because the hardware
    LP-I2C costs zero CPU and the soldered wiring covers every software fallback.
  - End-state option: when the display-link protocol rework happens (planned), flip the
    C5 to bus master and put display + BMI270 on the one HP I2C0 bus; the LP bus then
    goes away. Not done now to avoid changing two firmwares mid-migration.
  - Note: the SparkFun **Micro** BMI270 (SEN-22398) is effectively I2C-only (CS is not
    broken out). SPI would require swapping to the 1"×1" SEN-22397. Not needed.

---

## 2. Wiring details

### CAN bus (XIAO ↔ CAN Pal ↔ moteus-x1)

```
XIAO 3V3-OUT ── CAN Pal Vcc     (VIO is tied to Vcc → RX/TX are 3.3 V logic;
XIAO GND     ── CAN Pal GND      onboard AP3602A charge pump makes the 5 V internally)
XIAO D8 (GPIO8, TWAI TX) ── CAN Pal TX      (straight through, not crossed)
XIAO D3 (GPIO7, TWAI RX) ── CAN Pal RX
CAN Pal SLNT ── leave floating (internal pull-down = normal mode; high = listen-only)

CAN Pal H  ── twisted pair ── moteus-x1 CAN JST PH-3 pin 1 (CAN_H)
CAN Pal L  ── twisted pair ── moteus-x1 CAN JST PH-3 pin 2 (CAN_L)
CAN Pal terminal GND ─────── moteus-x1 CAN JST PH-3 pin 3 (GND)   ← common reference
```

- **Termination:** CAN Pal slide switch **ON** (it is a proper 2×60 Ω split termination
  with 4.7 nF center cap). moteus has **no onboard termination** → put the mjbots
  JST-PH3 CAN-FD terminator in the x1's second CAN jack (or solder 120 Ω). Both ends
  terminated; keep the bus short and stub-free (this is what makes 5 Mbps work).
- Verify the PH-3 pin order against the x1 silkscreen before crimping (mjbots pinout doc:
  1=CAN_H, 2=CAN_L, 3=GND).
- No power flows over the moteus CAN connector. Keep powering the XIAO from USB-C / the
  existing 5 V into VBUS. The x1's 5 V aux out is only 200 mA — too marginal for dual-band
  Wi-Fi TX bursts; don't use it for the XIAO.

### Bus timing (firmware-side, both phases)

- Arbitration 1 Mbps / data 5 Mbps, **sample point 0.666 in both phases** (moteus
  requires 0.666), SJW/DSJW as large as possible. moteus bit rates are fixed in its
  firmware — nothing to configure on the motor side.
- **Fallback if 5 M is marginal** (Espressif has formally tested the C5 at 4 M; the CTU
  core goes higher, and short buses generally run 5 M fine): clear the per-frame **BRS**
  flag on our side — moteus mirrors the query's BRS, so the whole bus drops to 1 Mbps
  while keeping FD frames. This is the officially documented fallback
  (`--can-disable-brs` in mjbots tooling). Still ample bandwidth: a 19-byte cmd+query
  exchange ≈ 0.5 ms round trip at 1 M.

### IMU (BMI270 Micro, Qwiic pigtail soldered to rear pads)

- SDA→GPIO2 (MTMS), SCL→GPIO3 (MTDI), 3V3, GND — see pin map. Pull-ups (2.2 kΩ) are on
  the breakout; chip supports up to 1 MHz I2C, target 400 kHz. Address 0x68.
- BMI270 init uploads a mandatory 8 KB Bosch config blob every power-on: `beginI2C()`
  takes ~200–300 ms at 400 kHz. Init inside the IMU task (as today), never blocking boot.

### Display link — ROLES FLIPPED (2026-07-13)

Original plan (C5 = slave @ 8, display = master) died on hardware: the
Arduino I2C slave driver never ACKs on ESP32-C5 silicon — the prebuilt IDF
libs only compile the legacy slave driver (CONFIG_I2C_ENABLE_SLAVE_DRIVER_
VERSION_2 is not set), and `Wire.begin(addr,...)` returns success while the
peripheral stays deaf. The C5's I2C *master* path works (BMI270 on LP-I2C
proved the silicon; HP I2C0 master is the standard driver).

Now: **C5 = bus master** on the same wires (D4/D5), **display = slave @ 8**
(the S3 slave role the old main board ran for months). C5 pushes the
unchanged 32-byte status frame at 10 Hz and polls a 32-byte command
mailbox (`I2C_CMD_MAILBOX`, byte-identical structs both sides, seq-deduped,
checksummed); screen commands are staged in the mailbox and collected
within ~100 ms. All command handlers, status struct, and ack/resend logic
survived the flip.

---

## 3. Toolchain

- Official `platformio/espressif32` is stuck on Arduino core 2.0.17 — **no C5, dead end**.
- Use the community **pioarduino** platform (Arduino 3.3.x / ESP-IDF 5.5.x), board id
  `seeed_xiao_esp32c5` (shipped since release 55.03.38):

```ini
[env:seeed_xiao_esp32c5]
platform = https://github.com/pioarduino/platform-espressif32/releases/download/55.03.39/platform-espressif32.zip
board = seeed_xiao_esp32c5
framework = arduino
monitor_speed = 115200
lib_deps =
    bblanchon/ArduinoJson @ ^6.18.5
    links2004/WebSockets @ ^2.7.1
```

- Arduino core 2.x→3.3.x is a breaking upgrade, but this codebase uses none of the broken
  APIs (no LEDC/analogWrite/old-timer/ADC-driver usage found). WiFi/WebSockets/ArduinoJson/
  Wire (incl. slave mode, `SOC_I2C_SUPPORT_SLAVE=1` on C5) all fine.
- **`driver/twai.h` does not exist on the C5 at all** (legacy driver is compiled out on
  FD-capable chips). CAN code must be rewritten against ESP-IDF 5.5's node API:
  `esp_twai.h` / `esp_twai_onchip.h` (`twai_new_node_onchip`, dual `bit_timing` +
  `data_timing`, per-frame `fdf`/`brs` flags, callback-driven RX). The precompiled
  Arduino libs include this component — usable directly from the sketch.

---

## 4. Software migration (by module)

| Module | Change |
|---|---|
| `CANCommunication` | **Full rewrite.** ODrive CANSimple (11-bit `node<<5\|cmd`) → moteus register protocol (16-bit source/dest ID, 0x8000 reply bit ⇒ extended IDs; little-endian multiplex subframes). Options: (a) mjbots `moteus-arduino` lib + ~100-line C5 backend (it's templated on a simple CanBus concept: `poll/available/receive/tryToSend` over its own `CANFDMessage`), or (b) mjbots `moteus_protocol.h` frame builders directly over the node API. RX callback → FreeRTOS queue consumed by MotorControlTask (keeps current architecture). |
| `MotorControl` | Control mapping is nearly 1:1 (below). Loop becomes query-driven instead of broadcast-driven: each cycle sends ONE position-mode command frame with the query bit; the reply carries position/velocity/torque. Replaces ODrive's 100 Hz pos/vel broadcast + 3–4 TX frames per cycle. |
| `Accelerometer` | Rewrite MPU6050 register code → BMI270 (SparkFun BMI270 Arduino Library if `Wire1` works; else Bosch bmi2 API over IDF LP-I2C). Keep the 50-sample averaging + sharedData path. |
| `I2CCommunication` | Pin numbers only (SDA 5→23, SCL 6→24). Protocol unchanged. |
| `WiFiCommunication`, `SerialCommunication`, `SharedData`, `Logging`, `ErrorHandling` | Port as-is; re-check task pinning (below). |
| Tasks | **C5 is single-core** (one 240 MHz RISC-V) vs S3's dual Xtensa. All `xTaskCreatePinnedToCore(..., 1)` must move to core 0 / no-affinity, and priorities re-tuned so the control task outranks Wi-Fi work. Watch the 500 Hz loop under Wi-Fi bursts; 8 MB PSRAM is available if RAM gets tight. |

### Control mapping (ODrive → moteus)

Force rendering today = velocity mode chasing an unreachable reel-in setpoint with a live
torque cap (the cap IS the rendered force). moteus equivalent, one frame per cycle:

| Concept | ODrive (today) | moteus (new) |
|---|---|---|
| Mode: render force | `AxisState=CLOSED_LOOP` + velocity control | Mode register (0x000) = **10** (position) |
| Mode: idle/sleep | `AxisState=IDLE` | Mode = **0** (stopped; also clears faults) |
| Unreachable chase | `Set_Input_Vel = -20 turns/s` | position (0x020) = **NaN**, velocity (0x021) = −20 rev/s |
| Force = live cap | `torque_soft_min` endpoint write (or Set_Limits current) | **maximum_torque (0x025) = \|sentTq\|** |
| Zone/recoil speed cap | `Set_Limits velocity_limit` | **velocity_limit (0x028)** |
| Feedback | 100 Hz pos/vel broadcast | query bit ⇒ reply: position 0x001, velocity 0x002, torque 0x003 |
| VBUS/temp telemetry | GET_VBUS 1 Hz | query 0x00d (voltage), 0x00e (temp) at low rate |
| Watchdog | none | `0x027` per-frame or `servo.default_timeout_s`; timeout ⇒ stopped = safe release |
| Units | motor turns, turns/s, Nm | output revolutions, rev/s, Nm — **1:1 carryover** |
| Node ID | 0 | **1** (moteus valid IDs 1–126; source/host = 0) |

Torque matrix / calibration anchors (0.5 Nm ≈ 1 lbf etc.) carry over as starting values if
the motor+spool stay the same; re-anchor with the existing `test_torque` cal probe.

### moteus bench prep (before any firmware work — needs fdcanusb/mjcanfd-usb-1x)

1. `moteus_tool --target 1 --calibrate` with the motor mounted free-spinning.
2. Set `id.id = 1`, verify `servo.default_timeout_s` + `servo.timeout_mode` (stopped),
   sane `servo.max_current_A` / velocity limits; `conf write`.
3. In tview, hand-verify the render recipe: `d pos nan -20 <maxT>` style commands produce
   the expected constant pull with a hard cap. Dump config into `moteus_x1/` (like
   `odrive_s1/my_config.json`).

---

## 5. Bring-up order (each step independently testable)

1. **Toolchain smoke test:** blink + Wi-Fi + WebSocket echo on the XIAO C5 (pioarduino env).
2. **IMU:** Qwiic on rear pads; try `Wire1` → IDF LP-I2C → bit-bang, in that order. WHO_AM_I
   (0x00 = 0x24), then streaming.
3. **CAN loopback:** TWAI-FD node in self-test/loopback at 1 M/5 M, verify FD frames.
4. **CAN ↔ moteus:** two-node bus, both terminations on. Send stop (Mode 0) + query,
   confirm reply. Then at 5 M under load; if errors, clear BRS and re-test at 1 M.
5. **Force rendering on the bench:** replicate today's feel (recoil, zone taper, weight
   gate, drop-catch) — the state machines are unchanged, only the transport swapped.
6. **Display link + full integration**, then re-anchor force calibration.

## 6. Risks / open items

- **5 Mbps data phase on the C5** — Espressif's formally tested figure is 4 M; short
  terminated bus expected to run 5 M; BRS-off @ 1 M is the sanctioned fallback. (Latency
  impact of fallback: negligible at our rates.)
- **LP-I2C under Arduino (`Wire1`) unproven** — mitigated by same-wiring fallbacks (IDF
  driver, bit-bang). Verify in step 2 before building anything on it.
- **I2C slave quirks on the new core** (v2 slave driver era) — existing checksum/seq
  protocol already defends; re-run the display-poll soak test.
- **Single-core scheduling** — 500 Hz control + Wi-Fi + I2C-slave ISR on one core; may
  need control-loop rate or priority adjustments.
- **Boot ROM prints on GPIO11 (D6)** at every reset — keep D6 for UART logging or leave
  free; don't hang reset-sensitive hardware on it.
- Included FPC antenna is the 2.4 GHz XIAO part — fine for current 2.4 GHz use; swap if
  5 GHz Wi-Fi 6 is ever wanted.
- Shopping: mjbots JST-PH3 CAN-FD terminator (+ crimped PH-3 leads), fdcanusb if not
  already owned (required — moteus cannot be calibrated from the MCU).

## 7. Row mode (added 2026-07-12, after the C5 port)

Row is its own control mode: a virtual Concept2-style flywheel in
MotorControl.cpp (one-way clutch with hysteresis, stiff coupling for the
catch, quadratic air-drag decay), rendered through the same torque-cap
pipeline — drop-catch and auto-sleep stay in force; only the near-home taper
shrinks to a pinch-only `row_zone`. Screen exposes gear/drag (1-10) via the
shared numpad; entering/leaving the row screen enables/disables the mode
(entry clears any latched weight — row and strength never stack).

Protocol changes (BOTH firmwares must be flashed together):
- `SharedStateData` grew 24 -> 32 bytes: + row_spm/row_drag/row_gear (u8,
  gear/drag echo 0 while row is off — the screen's ack signal) + row_watts
  (float). Checksum covers the new bytes automatically.
- New I2C command `SET_ROW` (3): {mask, gear, drag, row_enable} — mask bits
  0x1 gear / 0x2 drag / 0x4 enable-state.
- WS/HTTP "row" command now takes {"type":"on|off","gear":N,"drag":N}
  (legacy gear_ratio/damping accepted); metrics gained row_watts/row_spm.
- cal gained row tuning keys: row_kc, row_inertia, row_drag_base,
  row_return, row_max, row_zone (see setRowTuning) — tune on the bench over
  WS before trusting the screen defaults (gear 5 / drag 5).

## 8. Session log — bring-up 2026-07-12 → 2026-07-17 (state of the world)

**THE MACHINE WORKS.** Full chain live: screen ↔ C5 ↔ CAN-FD ↔ moteus-x1 ↔
2× RI115-PH KV40 (parallel, one shaft). Strength + row + force profiles +
web API all functional. What follows is everything a fresh session needs.

### Hardware truths (measured, not assumed)
- Motor pair calibrated as ONE motor: 40 poles, 0.038 Ω, 54 µH, Kv 40
  forced, `motor_position.output.sign = -1` (reel-in direction fix).
- Drivetrain: **~5 lbf per N·m** at the handle, **~5 revs per stroke**,
  capstan friction **~25% of load** (up/down asymmetry measured 2026-07-14:
  set 10 → 11/6 lb, set 20 → 20/12.5 before compensation).
- Bus: 48 V; CAN topology CAN Pal (term ON) → moteus (no term) → fdcanusb
  (term ON) daisy chain; C5 = CAN host id **4** (0 = tview/fdcanusb).
- Force ceiling **~48 lbf — deliberate** (`servo.max_current_A 40` = pair
  continuous). Headroom plan: 80 A + 1400 W → ~95 lb; ~104 A → ~120 lb but
  wire the RI115 thermistor to x1 AUX2 first; PSU must source ~20 A @ 48 V.

### Architecture decisions that were forced by reality
- **Display link roles FLIPPED** (§ Display link): C5 Arduino I2C slave
  never ACKs on this silicon (v2 slave driver not compiled into prebuilt
  libs) → C5 is bus MASTER (10 Hz status push + mailbox poll), display is
  slave @8. Structs byte-identical both firmwares → **always flash C5 +
  display as a pair** after protocol changes.
- **Canonical weight unit is POUNDS** everywhere (CommandMsg.weight_lb;
  /metrics active/pending_weight in lb). The old `weight_kg` name caused a
  real bug: web-set weights rendered at 45% force. Fixed 2026-07-14.
- BMI270 lives on LP-I2C (rear GPIO2/3 pads) and works.
- PlatformIO: C5 project has isolated `packages_dir` (packages-pioarduino)
  — official + pioarduino platforms clobber each other's riscv toolchain
  in the shared dir otherwise.
- USB CDC is non-blocking (`setTxTimeoutMs(0)`) — a half-attached monitor
  used to stall the 500 Hz control loop.

### Control design (strength)
500 Hz moteus position-mode stream: position=NaN, velocity = zone-clamped
reel-in chase, maximum_torque = rendered force. Pipeline order: weight gate
(pull-to-confirm) → matrix (force_scale 0.40, midpoint-anchored) → zone
taper → concentric-only unloading (con_pct 0-100, screen % label) →
directional friction comp (fric_k 0.17, boosts lowering/trims pulling) →
drop-catch shed → symmetric cap to moteus. Idle = 50 Hz query-only stream
(wake detection); 1 Hz telemetry (vbus/iq/temp/mode/fault + stop-recovery).

### Row mode
Virtual flywheel (one-way clutch + quadratic decay), gear/drag 1-10 from
the screen (defaults 1/1), watts + SPM computed and shipped. Anti-chatter:
high inertia (5) + wide hysteresis + fast-attack/slow-release shaping.
Enter/leave = row screen enter/leave; row and strength never stack.

### Force profiles (all dHome-anchored = revs from dock, repeatable)
constant / linear (chains + resistance-band recipes in WEB_API.md) /
detents (raised-cosine notches, re-enabled) / pulse. Compose in that order.

### Baked cal defaults (current compiled truth — see CAL_REFERENCE.md)
force_scale 0.40 · fric_k 0.17 band 0.6 · con 0 (0.3/1.5) · recoil_tq 0.3
recoil_vel 8 (dock 2) · zone 1.0 · gate 1.1/0.3/0.15/0.4 · wake 0.25 ·
drop 6/15/1.5 · row_kc 2.5 inertia 5 drag_base 0.15 return 0.6 max 10
zone 0.5. Cal is RAM-only — bake keepers here + reflash (NVS persistence
was offered, declined for now).

### Operational gotchas (hard-won)
- **Mute the C5 before tview/moteus_tool** (`cal {"can_quiet":true}`, boots
  live) and close tview before the calibration script (COM port exclusive).
  Power-cycle the moteus after any killed diagnostic session.
- moteus calibration: config-before-calibrate (servopos nan clears fault
  39); calibrate_moteus.py owns the port handoff; report JSON is ground
  truth if post-cal readback wedges.
- WiFi: KNOWN_NETWORKS list; hotspot currently impersonates home SSID
  (WPA2, 2.4 GHz compat mode). Boot is offline-tolerant; mDNS can go stale
  after network switch — the [hb] heartbeat IP is ground truth.
- Serial: [boot] banner + 5 s [hb] (wifi/can/tx/drop/rx/i2c push+cmd/vbus)
  + [wg]/[row] 4 Hz (cal debug toggles). Display -fast env has NO USB
  serial — flash plain T-Display-AMOLED env to hear it.
- Display upload_port pinned COM3 in its platformio.ini.
- Web app: HTTPS pages can't call the http device (mixed content) — utility
  must run from an http origin; device-served control page offered/pending.

### Open items (ordered, none blocking)
1. Bake session-tuned cal values as they settle (user reports numbers).
2. Force ceiling raise (deferred; plan above).
3. Device-served phone control page (solves mobile HTTPS wall + no-internet).
4. Friction comp acceptance at rep speed; fric_k → 0.20 candidate.
5. Bench-validate detents/chains/band profiles; SPM label on row screen
   (data already transmitted); concentric label ↔ app two-writer sync.
6. Strip bring-up scaffolding (wiring probe, extra prints) when stable.
7. NVS cal persistence if reflash-to-bake gets old.

## 9. Key sources

- ESP32-C5 datasheet v1.3 (I2C count §4.2.1.3; TWAI-FD §4.2.1.6; strapping ch. 3)
- ESP-IDF v5.5 TWAI (esp32c5) + I2C docs; esp-idf issues #17453 (4 M tested), #17461
- Seeed XIAO ESP32-C5 wiki (pinout, Arduino ≥3.3.5, PlatformIO)
- pioarduino platform-espressif32 (board `seeed_xiao_esp32c5`, ≥55.03.38)
- mjbots: moteus reference/protocol docs, fw/moteus.cc (fixed 1M/5M, BRS mirroring),
  moteus-x1 product page (2× JST PH-3, no termination, 5V/200mA aux), moteus-arduino
- NXP TJA1051 datasheet Rev. 9 ("timing guaranteed … up to 5 Mbit/s in the CAN FD fast
  phase"); Adafruit CAN Pal schematic (VIO=Vcc, AP3602A charge pump, switched split term.)
- SparkFun BMI270 hookup guide + Eagle files (Micro = I2C-only, 2.2 kΩ pull-ups, 0x68)
