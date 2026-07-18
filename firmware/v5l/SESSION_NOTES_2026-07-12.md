# Session Notes — Weight System Overhaul (2026-07-11 → 2026-07-12)

Restore-context document for the v5l firmware work session (Claude Code).
Covers both boards: `esp32s3_lillygo_amoled` (screen) and `esp32s3_main`
(motor controller). Everything below is BUILT and bench-verified except
where marked TODO.

---

## System architecture (as of end of session)

```
Screen (T-Display S3 AMOLED 1.91")          Main (Seeed XIAO ESP32S3)         ODrive S1 (fw 0.6.12)
┌─────────────────────────────┐   I2C    ┌──────────────────────────┐   CAN   ┌──────────────────┐
│ touch numpad sets weight    │ ───────► │ cmd queue → weight gate  │ ──────► │ M1, velocity mode│
│ red ZERO zone (left 30%)    │  0x08    │ → zone taper → drop-catch│  250k   │ setpoint -20 t/s │
│ 1 Hz status poll (checksum) │ ◄─────── │ → torque_soft_min writes │         │ torque_soft_min  │
│ ack/retry on commands       │          │ states: ACTIVE/RECOIL/   │         │ = the force      │
└─────────────────────────────┘          │         SLEEP            │         └──────────────────┘
```

**Force rendering ("the way"):** ODrive runs velocity control toward a
permanently unreachable reel-in setpoint (−20 turns/s), so it sits in
torque saturation and `axis0.config.torque_soft_min` IS the rendered
force. Written over CAN RxSdo (cmd 0x04) at **endpoint 304**;
`torque_soft_max` = **305**, streamed constant −/+0.5 Nm (motor must never
push toward the user). Endpoint IDs verified against
`firmware/v5l/odrive_s1/flat_endpoints.json` (fw 0.6.12) — **IDs are
firmware-version-specific and fail SILENTLY if stale**; re-check that file
after any ODrive fw update (`cal {"ep_min":<id>}` retargets live; the 1 Hz
TxSdo readback probe prints `[wg] ODrive readback ep=... val=...` when the
ID is right). Fallback path: Set_Limits (0x00F) current_limit with
`amps_per_nm` (cal `use_ep:false`).

**Calibration anchor (bench-measured):** 0.5 Nm ≈ 1 lbf at the handle
(≈2 lbf/Nm, near-direct drive, large spool). Stroke is multiple turns
(span telemetry in `[wg]` line measures it). Torque matrix is a pure
LINEAR law: torque = 0.5 Nm × screen units, NO plateau — upper rows exceed
motor ability on purpose (torque_soft_min is a bound, not a setpoint; the
ODrive saturates at max_current × Kt and force tops out gracefully).
Ceiling chain: `max_current` (20 A) × Kt → ODrive motor config → thermals.

## Safety / behavior layers (main board, MotorControl.cpp)

Applied in order: weight gate → RECOIL override → zone taper → drop-catch.
All share the 0.5 Nm (~1 lbf) floor and one philosophy: **force is only
ever restored by pulling into it.**

1. **Weight gate (pull-to-confirm increases):** new weight arms; force
   ramps old→new over `delta_x` (0.22) of pull after `deadband` (0.03),
   latches after `delta_y` (0.06) more; backing out > `cancel` (0.08)
   makes the ramp descend-only and CANCELS (screen snaps back via sync).
   Progress <10% just re-arms (jitter tolerance). Decreases apply
   IMMEDIATELY with a 75 units/s time slew — never gated. 0 = immediate.
2. **Machine states:** ACTIVE (weight set) / RECOIL (0 lb = 0.5 Nm
   self-stow at `recoil_vel` 2 turns/s) / SLEEP (axis idle). Sleep entry:
   parked within the home zone AND position still (within `wake_d` 0.05
   turns — position-window, NOT velocity: encoder velocity noise is ±0.2
   at rest) for `sleep_s` 10 s. Wake: movement > `wake_d` → closed loop in
   low/slow profile within ms; the zone taper re-delivers the latched
   weight with travel. Boot state = SLEEP. Latched weight survives sleep.
3. **Home + zone taper:** home = running min of pull position
   (bounce-filtered: only updates when |v| < 0.5). Within `zone` (0.75
   turns) of home, the force's excess above the floor scales linearly with
   distance — pinch safety at the dock, per-rep soft pickup, wake guard.
   Scales the REQUEST (no fixed ceiling → no step at the boundary).
4. **Drop-catch (anti-runaway):** reel-in faster than `drop_vmin` (2
   turns/s) sheds excess force linearly, floor-only at `drop_vmax` (8);
   one-way ratchet (no resurge on slow-down); restored linearly over
   `drop_restore` (0.3) turns of outward travel. Pull-out never limited.
   Does not touch the gate/screen — output-only suppression.

## Screen (apps/ows/)

- **Numpad** (`weight_numpad.cpp/h` — OUTSIDE `src/` so SquareLine
  re-exports can't clobber it): tap the weight number → modal 0–150 pad.
- **Red ZERO zone**: left 160 px of StrengthScreen, fires on PRESS →
  `handleMainScreenEvent(0)` → recoil. Created in code post-`ui_init()`.
- **Fonts**: weight value = RowFont2 (140 px, was 230 and ran under the
  X); "lb" = montserrat 32; static "43%" placeholder hidden.
- **Status link**: `SharedStateData` 24 bytes, seq + additive checksum
  (seed 0xA5) in old tail padding. Screen polls 1 Hz, drains stale bytes
  first, REJECTS bad checksum (counted on serial). Main always answers
  (last-good snapshot on mutex miss — empty responses were the NaN
  voltage bug: master clocked 0xFF filler = NaN float).
- **Command ack/retry**: weight sends tracked until main reports them
  (pending or active); unacked after 1 s → resend, ×3, then adopt main's
  truth. Distinguishes lost-command (resend) from user-cancel (snap back).
- **⚠ struct lockstep**: `SharedStateData` must stay byte-identical in
  `ows.ino` and `I2CCommunication.h`; flash BOTH boards when it changes.
- Splash: full-screen logo (240×536, byte-swapped RGB565 via
  `convert_logo.py`), drawn post-DISPON, redrawn after deferred init
  (cold-boot fix), dim boot + ramp. LilyGo SPI has NO lock — pixel writes
  from the LVGL/main task only.

## Live tuning — WiFi `cal` command (WS + HTTP, RAM-only, resets on boot)

```json
{"command":"cal",
 "force_scale":1.0, "zero_m1":0.1,
 "row":3, "values":[200,8,-10.2,0,-10.2,0],
 "delta_x":0.22, "delta_y":0.06, "deadband":0.03, "cancel":0.08,
 "pull_sign":1, "amps_per_nm":12, "vel_limit":25, "max_current":20,
 "ep_min":304, "use_ep":true,
 "zone":0.75, "recoil_tq":0.5, "recoil_vel":2, "soft_max":0.5,
 "sleep_s":10, "wake_d":0.05,
 "drop_vmin":2, "drop_vmax":8, "drop_restore":0.3,
 "test_torque":0.5, "debug":true}
```
Omitted keys unchanged. `test_torque`: constant reel-in Nm for 10 s
(clamp ±3), for Nm→felt-lbf mapping. Bake tuned values into the source
defaults once bench-final.

## Telemetry (`[wg]` serial line, 4 Hz, `debug:false` to silence)

`st` gate state (0 inactive/1 armed/2 ramping/3 cancelling) · `ms` machine
state (0 sleep/1 recoil/2 active) · `act/pend` weights · `tgt` N ·
`Tq` sent Nm · `d` dist from home · `drop` shed fraction · `pos/v` ·
`mode` (1 = streaming) · `span` min/max pos since boot (stroke measure).
Plus 1 Hz `ODrive readback ep=0x0130 val=...` (write verification).

## Hard-won debugging lessons (do not relearn)

- ODrive CAN endpoint IDs are fw-specific; wrong IDs are silent no-ops —
  weeks of "constants change nothing" came from 0x0101 vs 304. Verify via
  readback, never assume.
- `createCANMessage` zeroes payloads now — RxSdo byte 3 (reserved, must
  be 0) was uninitialized stack garbage.
- Empty I2C slave responses → master reads 0xFF → NaN floats. Always
  answer; always checksum.
- ESP32 encoder velocity jitters ±0.2 turns/s at rest — never threshold
  raw velocity for stillness; use position windows.
- Fire-and-forget commands + truth-sync = phantom reverts; ack everything.
- RM67162: RAM writes before DISPON are dropped; panel needs ~100 ms after
  cold power-on; brightness 0x51 latches only after DISPON.

## Open TODOs

- **Units quirk (deliberate, unreconciled):** screen "lb" number is
  treated as kg internally (×9.81 → N); WiFi converts lb→kg first. Same
  number, different force per path. Reconcile in a units pass someday.
- Bake bench-tuned cal values into source defaults (they reset on boot).
- Product max-weight intake clamp once the motor ceiling is measured
  (screen WEIGHT_MAX=150 + mirror on WiFi paths).
- Eccentric bias: matrix pull-in vs pull-out columns are symmetric now;
  add via row edits when feel work resumes.
- `processI2CData()` legacy SET_DEBUG mode-toggle path is dead code.
- Hardware: bulk cap on 5V rail still the real fix for cold-boot margin.
- `weight_max`, damping, dynamic feedback fields in I2C cmd: sent but
  unused.

## Build / flash

- Screen: `esp32s3_lillygo_amoled/` → `pio run -e T-Display-AMOLED -t upload` (COM3)
- Main: `esp32s3_main/` → `pio run -e seeed_xiao_esp32s3 -t upload`
- PlatformIO at `%USERPROFILE%\.platformio\penv\Scripts\platformio.exe`
