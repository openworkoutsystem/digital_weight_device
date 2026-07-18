#include "MotorControl.h"
#include "ErrorHandling.h"
#include "CANCommunication.h"
#include "I2CCommunication.h"
#include "SharedData.h" // Include the shared data header
#include <math.h>

ControlFeedback controlFeedback; // Define the controlFeedback structure

// declare functions
float applyForceControl(float target_force, float position);
float applyDetentControl(float target_force, float position);
float applyPulseControl(float target_force);
float applyRowModeControl(float target_force, float velocity, float &row_force_strength);

// ============================ Weight-update gate =============================
// SAFETY: a weight INCREASE is never applied instantly. The user must "pull
// into it": force ramps linearly from the old weight to the new one over
// g_wgDeltaX of travel, and only latches after holding a further g_wgDeltaY
// beyond the ramp. Backing out before latching reverses the force along the
// same ramp and CANCELS the update (the old weight is restored and reported
// back to the screen).
// A weight DECREASE applies immediately with no movement required — the
// output slews down smoothly over time (WG_DOWN_SLEW_PER_SEC), never as a
// step. Idle (0) always applies instantly — it is the off switch.
//
// Position units are M1 MOTOR turns (pre-gearbox). The drivetrain gain
// (~120 lbf/Nm) implies heavy reduction: expect roughly ~1 cm of strap per
// motor turn, so ramp distances need to be TENS of turns. Measure exactly:
// with [wg] telemetry on, note `pos` at strap rest vs pulled a known
// distance, then set these live via the cal command (delta_x/delta_y/
// deadband/cancel) — no reflash needed.
#define WG_MIN_PROGRESS_REARM (0.10f) // below this ramp progress a backtrack re-arms instead of cancelling
// Bench evidence (2.0-turn deadband unreachable by a full pull; 0.1 turns
// took a noticeable pull) puts the full stroke at only ~1-2 position units,
// so the gate distances default to fractions of a unit. Calibrate with the
// span=[min,max] readout in the [wg] telemetry after one full pull, then
// set exact values live via cal (delta_x/delta_y/deadband/cancel).
// Stroke measurement 2026-07-11 (second bench log): pos reached 0.63+ and
// was still climbing mid-pull — real stroke is at least ~0.8 units (the
// earlier 0.19 "span" was only a partial tug). Distances sized to that;
// confirm with one deliberate full pull via the span=[..] readout.
static float g_wgPullSign = +1.0f;       // +1 if position increases as the user pulls out; cal "pull_sign" flips it
static float g_wgDeltaX = 0.22f;         // ramp ~1/4 of stroke
static float g_wgDeltaY = 0.06f;         // latch hold
static float g_wgDeadband = 0.03f;       // engage deadband
static float g_wgCancelRetract = 0.08f;  // cancel backtrack

void setWeightGateDistances(float deltaX, float deltaY, float deadband, float cancelRetract)
{
    if (deltaX > 0.0f) g_wgDeltaX = deltaX;
    if (deltaY >= 0.0f) g_wgDeltaY = deltaY;
    if (deadband >= 0.0f) g_wgDeadband = deadband;
    if (cancelRetract > 0.0f) g_wgCancelRetract = cancelRetract;
}

void setWeightGatePullSign(float sign)
{
    if (sign == 1.0f || sign == -1.0f)
    {
        g_wgPullSign = sign;
    }
}

// Stroke-span tracker for calibration: min/max raw position seen since boot
static float g_posSpanMin = 1e9f;
static float g_posSpanMax = -1e9f;

// Direct torque probe for Nm -> felt-lbf calibration: overrides the force
// pipeline with a constant torque for 10 s, then reverts. Clamped small.
// Only streams while in strength mode (set a weight first).
static float g_testTorque = 0.0f;
static unsigned long g_testTorqueUntilMs = 0;

void setTestTorque(float torqueNm)
{
    // Clamp at ~6 lbf equivalent (0.5 Nm ~= 1 lbf)
    if (torqueNm > 3.0f) torqueNm = 3.0f;
    if (torqueNm < -3.0f) torqueNm = -3.0f;
    // Convention: pass magnitude or signed; rendering always pulls in
    g_testTorque = -fabsf(torqueNm);
    g_testTorqueUntilMs = (torqueNm != 0.0f) ? millis() + 10000UL : 0;
}
#define WG_DOWN_SLEW_PER_SEC (75.0f)  // max downward output change (weight units/s); ~2s for a full-scale drop

// Calibration anchor (Aaron, bench-measured via torque_soft_min in
// odrivetool): 0.5 Nm ~= 1 lbf at the handle -> drivetrain gain ~2 lbf/Nm,
// i.e. near-direct drive on a large spool (r ~= 0.11 m). Consistent with
// the ~1-turn full stroke. The matrix rows below are expressed in real Nm
// on that basis, so the lookup runs unscaled.
// Tunable live (no reflash) via the WiFi "cal" command — see setForceCal().
#define FORCE_CAL_SCALE (1.0f)
static float g_forceCalScale = FORCE_CAL_SCALE;

// Torque matrix at file scope so the live-cal setter can adjust the zero
// row. Single-motor machine (M1 only; the high-range M2 motor was removed).
// 6 rows x 6 columns:
// force, velocity_delta, -M1_Fn, -M1_Fm, M1_Fn, M1_Fm
// minus sign = motion towards the device, positive = towards the user.
// Fn is static torque, Fm the velocity coefficient: F = Fn + Fm * v.
// Row 0 is the ZERO row: near-zero torque so light forces are actually
// renderable (the old matrix had a hard floor at its first row); it also
// sets the strap keep-taut tension now that M2 is gone. The top rows
// plateau at the row-3 torques — the old values there backed M1 off while
// M2 supplied the heavy force, which no longer exists. TUNE ON HARDWARE.
// Rows in REAL Nm from the 0.5 Nm ~= 1 lbf anchor: LINEAR law, torque =
// 0.5 * screen units (screen unit -> 9.81 N internally), all the way up —
// the matrix shapes feel (velocity blend, keep-taut floor), it does NOT
// clamp. The upper rows exceed what the motor can physically deliver on
// purpose: torque_soft_min is a BOUND, not a setpoint, so the ODrive just
// saturates at its current limit (max_current * Kt) and force tops out
// gracefully. The ceiling lives in the physical limit chain, not here.
// Symmetric pull-in/pull-out to start; add eccentric bias via row edits.
// velocity_delta (col 1) is 8 turns/s so the direction blend transitions
// smoothly instead of grabbing on hesitation.
static float torque_matrix[6][6] = {
    {0, 8, -0.1, 0, -0.1, 0},        // zero row (keep-taut, ~0.2 lbf)
    {9.8, 8, -0.5, 0, -0.5, 0},      // screen 1 ~= 1 lbf (bench anchor)
    {50, 8, -2.55, 0, -2.55, 0},     // screen ~5
    {200, 8, -10.2, 0, -10.2, 0},    // screen ~20
    {640, 8, -32.6, 0, -32.6, 0},    // screen ~65
    {2000, 8, -101.9, 0, -101.9, 0}  // screen ~204 (beyond motor: saturates)
};

static bool g_wgDebug = true; // 4 Hz [wg] serial telemetry; toggle via cal {"debug":false}

// Force rendering: VELOCITY MODE with a live TORQUE LIMIT ("the way"). The
// axis runs velocity control toward a constantly unreachable reel-in
// setpoint, so it sits in torque saturation and the torque limit IS the
// rendered force. The limit is set via Set_Limits (0x00F) current_limit —
// a first-class CANSimple command — NOT via the torque_soft_* endpoint
// writes the original code used: those went through the arbitrary-endpoint
// interface (RxSdo) which this ODrive provably ignores (the 1 Hz TxSdo
// readback never got a reply on the bench; no cap value ever changed feel).
// torque[Nm] -> current[A] via g_ampsPerNm (= 1 / torque_constant), live
// tunable ("amps_per_nm") since the motor's torque constant is unverified.
static float g_ampsPerNm = 12.0f;   // A per Nm; used only by the Set_Limits fallback path
static float g_velLimit = 25.0f;    // turns/s limit sent with Set_Limits; > |reel-in setpoint|
static float g_reelInVel = -20.0f;  // velocity setpoint (unreachable by design)
static float g_maxCurrentA = 20.0f; // generous current ceiling streamed in endpoint mode
                                    // (force is governed by torque_soft_min; this just
                                    // guarantees vel_limit and undoes any stale low clamp)

// Force rendering DEFAULT: write torque_soft_min directly (the original
// architecture) at the endpoint ID verified against this ODrive firmware's
// flat_endpoints.json (304). The 1 Hz readback probe reads the same ID, so
// correct addressing shows up as "[wg] ODrive readback ..." lines echoing
// our written values. cal {"ep_min":<id>} retargets after a future ODrive
// fw update; cal {"use_ep":false} falls back to Set_Limits current-based
// limiting.
static uint16_t g_epTorqueSoftMin = (uint16_t)TORQUE_SOFT_MIN_M1;
static bool g_useEpWrites = true;

void setSoftMinEndpoint(int endpointId, int useEpWrites)
{
    if (endpointId > 0 && endpointId <= 0xFFFF)
    {
        g_epTorqueSoftMin = (uint16_t)endpointId;
    }
    if (useEpWrites == 0 || useEpWrites == 1)
    {
        g_useEpWrites = (useEpWrites == 1);
    }
}

// =================== Home zone / recoil / auto-sleep ========================
// - Home is the running minimum of pull position (init at boot, follows any
//   lower position, filtered so recoil bounce can't ratchet it down).
// - Within g_homeZone of home the force cap tapers from the recoil floor
//   (~1 lbf) up to uncapped at the zone edge, and the velocity limit lerps
//   from recoil-slow to normal. This is simultaneously: pinch safety at the
//   dock, the soft "pick the weight up" on every rep, and wake-from-sleep
//   jolt protection.
// - Weight 0 = RECOIL: ~1 lbf at recoil velocity so the strap self-stows.
// - Parked inside the zone and still for g_sleepTimeoutMs -> SLEEP (axis
//   idle, zero current). Any movement past g_wakeThresh wakes into the
//   low/slow profile immediately; the zone taper then restores the latched
//   weight as the user pulls out. The latched weight NEVER changes across
//   sleep, and force is never re-applied without user travel.
enum MachineState : uint8_t
{
    MS_SLEEP = 0,  // axis idle, watching for movement (also the boot state)
    MS_RECOIL = 1, // weight 0: gentle self-stow
    MS_ACTIVE = 2  // weight set: normal force pipeline
};
static MachineState g_machineState = MS_SLEEP;
static float g_homePos = 1e9f;         // running min of pull position
static float g_homeZone = 0.75f;       // turns from home (Aaron 2026-07-12)
static float g_recoilTorqueNm = 0.5f;  // ~1 lbf recoil force / cap floor
static float g_recoilVel = 2.0f;       // turns/s near home / during recoil
static float g_softMaxNm = 0.5f;       // push-toward-user bound (~1 lbf)
static float g_sleepTimeoutMs = 10000.0f;
static float g_wakeThresh = 0.05f;     // turns of movement that wake from sleep
                                       // (also the stillness window: less
                                       // movement than would wake = still)
static unsigned long g_stillSinceMs = 0;
static float g_stillAnchorPos = 0.0f;  // stillness reference position
static float g_sleepPos = 0.0f;        // position captured at sleep entry
static float g_effectiveTq = 0.0f;     // state-aware torque actually rendered

void setHomeBehavior(float zone, float recoilTq, float recoilVel,
                     float softMax, float sleepS, float wakeThresh)
{
    if (zone > 0.0f && zone <= 10.0f) g_homeZone = zone;
    if (recoilTq >= 0.0f && recoilTq <= 2.0f) g_recoilTorqueNm = recoilTq;
    if (recoilVel > 0.0f && recoilVel <= 50.0f) g_recoilVel = recoilVel;
    if (softMax >= 0.0f && softMax <= 2.0f) g_softMaxNm = softMax;
    if (sleepS > 0.0f && sleepS <= 3600.0f) g_sleepTimeoutMs = sleepS * 1000.0f;
    if (wakeThresh > 0.0f && wakeThresh <= 1.0f) g_wakeThresh = wakeThresh;
}

// ========================== Drop-catch (anti-runaway) =======================
// If the strap reels IN faster than g_dropVMin (lost grip / dropped handle),
// the force's excess above the recoil floor is shed linearly with speed,
// reaching floor-only at g_dropVMax — and the shed level LATCHES (one-way
// ratchet) so force cannot resurge onto a runaway or unheld strap when it
// slows. Pulling OUT releases the latch linearly over g_dropRestoreDist of
// outward travel ("the weight is re-earned by pulling into it", same rule
// as wake and the home zone). Pull-out speed is never limited. Controlled
// eccentrics stay below g_dropVMin and never trigger it.
static float g_dropVMin = 2.0f;        // turns/s reel-in where shedding starts
static float g_dropVMax = 8.0f;        // turns/s reel-in where only the floor remains
static float g_dropRestoreDist = 0.3f; // turns of outward travel for full restore
static float g_dropFrac = 1.0f;        // live shed fraction: 1 = full force, 0 = floor only
static float g_dropLastPullPos = 0.0f;

void setDropCatch(float vMin, float vMax, float restoreDist)
{
    if (vMin > 0.0f && vMin <= 50.0f) g_dropVMin = vMin;
    if (vMax > 0.0f && vMax <= 50.0f && vMax > g_dropVMin) g_dropVMax = vMax;
    if (restoreDist > 0.0f && restoreDist <= 10.0f) g_dropRestoreDist = restoreDist;
}

// Near-home taper: scale the excess force above the recoil floor by the
// distance from home — floor-only at the dock, the full requested force at
// the zone edge. Because it scales the REQUEST rather than capping to a
// fixed ceiling, there is no step at the boundary for any magnitude; the
// physical ceiling remains the current-limit chain, never this taper.
static float zoneTaperTorque(float dHome, float magNm)
{
    if (dHome >= g_homeZone) return magNm;
    if (dHome < 0.0f) dHome = 0.0f;
    float excess = magNm - g_recoilTorqueNm;
    if (excess <= 0.0f) return magNm;
    return g_recoilTorqueNm + (dHome / g_homeZone) * excess;
}

// Velocity limit lerped from recoil-slow at home to normal at the zone edge
static float zoneVelLimit(float dHome)
{
    if (dHome >= g_homeZone) return g_velLimit;
    if (dHome < 0.0f) dHome = 0.0f;
    return g_recoilVel + (dHome / g_homeZone) * (g_velLimit - g_recoilVel);
}

void setDriveParams(float ampsPerNm, float velLimit, float maxCurrentA)
{
    if (ampsPerNm > 0.0f && ampsPerNm <= 100.0f) g_ampsPerNm = ampsPerNm;
    if (velLimit > 0.0f && velLimit <= 100.0f) g_velLimit = velLimit;
    if (maxCurrentA > 0.0f && maxCurrentA <= 100.0f) g_maxCurrentA = maxCurrentA;
}

void setForceCalDebug(bool on)
{
    g_wgDebug = on;
}

void setForceCalRow(int row, const float *values, int count)
{
    if (row < 0 || row >= 6 || values == nullptr)
    {
        return;
    }
    for (int i = 0; i < count && i < 6; i++)
    {
        torque_matrix[row][i] = values[i];
    }
}

void setForceCal(float lookupScale, float zeroM1)
{
    if (lookupScale > 0.0f && lookupScale <= 1.0f)
    {
        g_forceCalScale = lookupScale;
    }
    if (zeroM1 >= 0.0f && zeroM1 <= 1.0f)
    {
        torque_matrix[0][2] = -zeroM1;
        torque_matrix[0][4] = -zeroM1;
    }
}

void getForceCal(float *lookupScale, float *zeroM1)
{
    if (lookupScale) *lookupScale = g_forceCalScale;
    if (zeroM1) *zeroM1 = -torque_matrix[0][2];
}

enum WgState : uint8_t
{
    WG_INACTIVE = 0,
    WG_ARMED_UP,   // pending increase, waiting for a forward pull to start
    WG_ENGAGED_UP, // ramping up with forward travel
    WG_CANCEL_UP   // backing out of an increase: force may only descend
};

static WgState wgState = WG_INACTIVE;
static float wgActive = 0.0f;    // latched weight value (units as commanded)
static float wgPending = 0.0f;   // requested weight value awaiting confirmation
static float wgAnchor = 0.0f;    // trough while armed
static float wgRampStart = 0.0f; // position where the ramp begins
static float wgExtreme = 0.0f;   // best position reached while engaged
static float wgCeil = 0.0f;      // progress ratchet while cancelling
static float wgRendered = 0.0f;  // slew-limited output actually applied
static unsigned long wgLastTickMs = 0;

static inline float wgPullPos()
{
    return g_wgPullSign * controlFeedback.m1_position;
}

// Report gate state to the screen via the I2C status struct. Rate-limited
// to 10 Hz — plenty for the screen's 1 Hz poll, and it keeps this path from
// hammering the mutex the I2C request handler needs. A missed take retries
// on the next control tick.
static void wgPublish()
{
    static unsigned long wgPubMs = 0;
    if (millis() - wgPubMs < 100)
    {
        return;
    }
    if (xSemaphoreTake(mutex, 0) == pdTRUE)
    {
        sharedStateData.active_weight = wgActive;
        sharedStateData.pending_weight = (wgState != WG_INACTIVE) ? wgPending : wgActive;
        sharedStateData.weight_pending = (wgState != WG_INACTIVE) ? 1 : 0;
        xSemaphoreGive(mutex);
        wgPubMs = millis();
    }
}

// A new weight was commanded (screen or WiFi). Increases arm the
// pull-to-confirm gate; decreases take effect at once (output slew smooths
// the drop, no movement required).
static void wgCommand(float value)
{
    if (value > wgActive)
    {
        wgPending = value;
        wgAnchor = wgPullPos();
        wgExtreme = wgAnchor;
        wgState = WG_ARMED_UP;
    }
    else
    {
        wgActive = value;
        wgPending = 0.0f;
        wgState = WG_INACTIVE;
    }
}

// Idle is immediate and clears any pending confirmation.
static void wgIdle()
{
    wgActive = 0.0f;
    wgPending = 0.0f;
    wgRendered = 0.0f; // bypass the slew: idle is the off switch
    wgState = WG_INACTIVE;
}

// Advance the gate with the latest position; returns the weight value the
// machine should render right now.
static float wgTick()
{
    float pos = wgPullPos();
    float desired = wgActive;

    switch (wgState)
    {
    case WG_INACTIVE:
        break;

    case WG_ARMED_UP:
        if (pos < wgAnchor) wgAnchor = pos; // follow the trough down
        if (pos > wgAnchor + g_wgDeadband)
        {
            wgRampStart = wgAnchor + g_wgDeadband;
            wgExtreme = pos;
            wgState = WG_ENGAGED_UP;
        }
        break;

    case WG_ENGAGED_UP:
    {
        if (pos > wgExtreme) wgExtreme = pos;
        if (pos >= wgRampStart + g_wgDeltaX + g_wgDeltaY)
        {
            wgActive = wgPending; // confirmed: latch
            wgState = WG_INACTIVE;
            desired = wgActive;
            break;
        }
        if (wgExtreme - pos > g_wgCancelRetract)
        {
            float peakProgress = (wgExtreme - wgRampStart) / g_wgDeltaX;
            if (peakProgress < WG_MIN_PROGRESS_REARM)
            {
                // Barely engaged: treat as jitter, re-arm instead of cancel
                wgAnchor = pos;
                wgState = WG_ARMED_UP;
                break;
            }
            wgCeil = constrain((pos - wgRampStart) / g_wgDeltaX, 0.0f, 1.0f);
            wgState = WG_CANCEL_UP;
            desired = wgActive + (wgPending - wgActive) * wgCeil;
            break;
        }
        float progress = constrain((pos - wgRampStart) / g_wgDeltaX, 0.0f, 1.0f);
        desired = wgActive + (wgPending - wgActive) * progress;
        break;
    }

    case WG_CANCEL_UP:
    {
        // Descend-only: force follows the ramp back down as they retract and
        // can never rise again; at zero the update is fully cancelled.
        float progress = constrain((pos - wgRampStart) / g_wgDeltaX, 0.0f, 1.0f);
        if (progress < wgCeil) wgCeil = progress;
        if (wgCeil <= 0.0f)
        {
            wgState = WG_INACTIVE; // cancelled: wgActive stands, pending dropped
            if (wgActive <= 0.0f)
            {
                // The cancelled increase started from 0: back to recoil
                // (auto-sleep will idle the axis once the strap parks)
                g_machineState = MS_RECOIL;
            }
            break;
        }
        desired = wgActive + (wgPending - wgActive) * wgCeil;
        break;
    }
    }

    // Smooth every downward output change over time: heavy -> light applies
    // with no movement required, but always as a ramp, never a step. Upward
    // changes are already position-ramped by the gate above, so they pass
    // through directly. Idle bypasses this entirely via wgIdle().
    unsigned long nowMs = millis();
    float dt = (wgLastTickMs == 0) ? 0.0f : (nowMs - wgLastTickMs) / 1000.0f;
    if (dt > 0.1f) dt = 0.1f; // guard against long gaps (boot, CAN stall)
    wgLastTickMs = nowMs;
    if (desired < wgRendered)
    {
        float maxStep = WG_DOWN_SLEW_PER_SEC * dt;
        wgRendered = (wgRendered - desired > maxStep) ? (wgRendered - maxStep) : desired;
    }
    else
    {
        wgRendered = desired;
    }
    return wgRendered;
}
// ========================== end weight-update gate ===========================

void initMotorControl()
{
    xTaskCreatePinnedToCore(
        MotorControlTask,   // Task function
        "MotorControlTask", // Name of the task
        10000,              // Stack size in words
        NULL,               // Task input parameter
        2,                  // Priority of the task (equal to WS for round-robin)
        NULL,               // Task handle
        0);                 // Core where the task should run
}

void MotorControlTask(void *parameter)
{
    int counter = 0;
    int lastID = 0;
    static uint8_t lastAppliedMode = 255; // Track last mode to avoid redundant CAN sends

    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    int64_t prev_time_us = time_us;
    int64_t runInterval_us = 2000; // 500Hz loop time.

    float m1_torque_soft = 0;
    float min_torque_m1 = -0.4;
    float m1_torque_soft_max = 0;
    float m1_torque_soft_min = 0;

    int torque_matrix_a = 0;
    int torque_matrix_b = 1;
    float torque_interpolation = 0;
    float m1_minus_torque_soft_fn = 0;
    float m1_minus_torque_soft_fm = 0;
    float m1_plus_torque_soft_fn = 0;
    float m1_plus_torque_soft_fm = 0;
    float velocity_delta = 0;

    float vel_factor = 0.5;
    float vel_minus_factor = 0.5;
    float vel_plus_factor = 0.5;

    float row_force_strength = 0;
    static bool modesConfigured = false; // Configure controller/input modes once

    // torque_matrix lives at file scope (above) so live calibration can
    // adjust it — see setForceCal().

    // SAFETY: bound the torque before the axis can ever enter closed loop,
    // so the first entry can't run on a stale/stored limit and snatch the
    // strap. In endpoint mode the force bound is torque_soft_min; the
    // current limit stays GENEROUS (a low current clamp would silently
    // ceiling the whole machine, since nothing else raises it later).
    if (g_useEpWrites)
    {
        sendMotorRX(CAN_NODEID_M1, (ODriveEndpointID)g_epTorqueSoftMin, -0.1f);
        sendMotorRX(CAN_NODEID_M1, TORQUE_SOFT_MAX_M1, g_softMaxNm);
        sendMotorLimits(CAN_NODEID_M1, g_recoilVel, g_maxCurrentA);
    }
    else
    {
        sendMotorLimits(CAN_NODEID_M1, g_recoilVel, 0.2f);
    }

    // init force modulation params
    updateForceData("constant", 100, 30, 0.5, 2); // off, constant, linear
    updateDetentData("off", 50, .2, .3, 10);
    updatePulseData("off", 0, 100, 16);
    updateRowData("off", 20, 3, 0);

    while (true)
    {
        gettimeofday(&tv_now, NULL);
        time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
        int64_t elapsed_us = time_us - prev_time_us;
        if (elapsed_us < runInterval_us)
        {
            // Yield to avoid spin-wait starvation; sleep fractional ms if possible
            int64_t remaining_us = runInterval_us - elapsed_us;
            if (remaining_us > 1000) {
                vTaskDelay(pdMS_TO_TICKS(1)); // yield ~1ms
            } else {
                taskYIELD(); // yield immediately for sub-ms waits
            }
            continue;
        }
        prev_time_us = time_us;

        // Drain command queue from WS/HTTP without blocking (before timing gate)
        // Coalesce to the latest requested state to avoid rapid back-to-back toggles causing traffic bursts
        CommandMsg msg;
        CommandType desiredType = CMD_NONE;
        float desiredWeightKg = 0.0f;
        unsigned long desiredCmdMs = 0;
        while (g_cmdQueue && xQueueReceive(g_cmdQueue, &msg, 0) == pdTRUE) {
            desiredType = msg.type;
            desiredWeightKg = msg.weight_kg;
            desiredCmdMs = millis();
        }
        if (desiredType != CMD_NONE) {
            bool applied = false;
            // Small timeout (not 0): a drained command that failed to apply
            // would otherwise be silently lost.
            if (xSemaphoreTake(mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                applied = true;
                if (desiredType == CMD_STRENGTH) {
                    // SAFETY: never apply a new weight instantly. Arm the
                    // gate; force follows the user's travel (see wgTick) and
                    // only latches after a confirming pull.
                    wgCommand(desiredWeightKg);
                    g_machineState = (desiredWeightKg > 0.0f) ? MS_ACTIVE : MS_RECOIL;
                    // Enter closed loop right away — the pre-staged torque
                    // bound is the gate's current output (~zero row while
                    // armed), so the motor engages holding essentially no
                    // force and the ramp adds it with travel.
                    if (controlState.mode != 1) {
                        controlState.mode = 1;
                        pendingApplyStrength = true;
                    }
                } else if (desiredType == CMD_IDLE) {
                    // Weight 0 = RECOIL: gentle ~1 lbf self-stow at low
                    // velocity (auto-sleep idles the axis once parked)
                    wgIdle();
                    sharedCfgData.target_force = 0.0f;
                    g_machineState = MS_RECOIL;
                    if (controlState.mode != 1) {
                        controlState.mode = 1;
                        pendingApplyStrength = true;
                    }
                }
                xSemaphoreGive(mutex);
            }
            if (!applied)
            {
                // Mutex contention: DO NOT lose the command — push it back
                // to the front of the queue and retry next control cycle
                CommandMsg retry;
                retry.type = desiredType;
                retry.weight_kg = desiredWeightKg;
                if (g_cmdQueue)
                {
                    xQueueSendToFront(g_cmdQueue, &retry, 0);
                }
            }
            sharedData.t_cmd_set_ms = desiredCmdMs;
        }

        // Handle any pending mode changes requested by other threads (e.g., WiFi) here
        // to centralize CAN traffic and avoid cross-task contention.
        if (xSemaphoreTake(mutex, 0) == pdTRUE)
        {
            if (pendingApplyStrength && lastAppliedMode != 1)
            {
                uint32_t velocity_control_mode = 0x00000002;
                uint32_t passthrough_input_mode = 0x00000001;
                uint32_t closed_loop_mode = 0x00000008;
                // SAFETY: pre-stage the torque bound and velocity setpoint
                // BEFORE commanding closed loop so the axis engages holding
                // the gate's current output (~zero row at a ramp start),
                // never the ODrive's stored limit.
                if (g_useEpWrites)
                {
                    sendMotorRX(CAN_NODEID_M1, (ODriveEndpointID)g_epTorqueSoftMin, -fabsf(g_effectiveTq));
                    sendMotorRX(CAN_NODEID_M1, TORQUE_SOFT_MAX_M1, g_softMaxNm);
                    // Conservative recoil-slow velocity limit for the entry
                    // instant; the per-frame stream corrects it immediately
                    sendMotorLimits(CAN_NODEID_M1, g_recoilVel, g_maxCurrentA);
                }
                else
                {
                    sendMotorLimits(CAN_NODEID_M1, g_recoilVel, fabsf(g_effectiveTq) * g_ampsPerNm);
                }
                sendMotorVelocity(CAN_NODEID_M1, g_reelInVel);
                if (!modesConfigured) {
                    // First time entering strength: configure controller/input modes
                    sendMotorMode(CAN_NODEID_M1, velocity_control_mode, passthrough_input_mode, closed_loop_mode);
                    modesConfigured = true;
                } else {
                    // Modes already configured: only command axis state to closed loop
                    twai_message_t msg1 = createCANMessage(CAN_NODEID_M1, SET_AXIS_STATE, 4);
                    memcpy(&msg1.data[0], &closed_loop_mode, sizeof(uint32_t));
                    transmitCANMessage(msg1);
                }
                lastAppliedMode = 1;
                pendingApplyStrength = false;
                unsigned long applyMs = millis();
                sharedData.t_ctrl_apply_ms = applyMs;
                sharedData.mode_latency_ms = applyMs - sharedData.t_cmd_set_ms;
            }
            else if (pendingApplyStrength)
            {
                // Already in strength mode, just clear flag
                pendingApplyStrength = false;
                unsigned long applyMs = millis();
                sharedData.t_ctrl_apply_ms = applyMs;
                sharedData.mode_latency_ms = applyMs - sharedData.t_cmd_set_ms;
            }
            if (pendingApplyIdle && lastAppliedMode != 0)
            {
                uint32_t idle_mode = 0x00000001;
                // Only command axis idle; controller/input modes remain configured
                twai_message_t msg1 = createCANMessage(CAN_NODEID_M1, SET_AXIS_STATE, 4);
                memcpy(&msg1.data[0], &idle_mode, sizeof(uint32_t));
                transmitCANMessage(msg1);
                // Drop the torque bound to the floor while idle so the next
                // closed-loop entry can never start from a stale limit
                if (g_useEpWrites)
                {
                    sendMotorRX(CAN_NODEID_M1, (ODriveEndpointID)g_epTorqueSoftMin, -0.1f);
                }
                else
                {
                    sendMotorLimits(CAN_NODEID_M1, g_velLimit, 0.2f);
                }
                lastAppliedMode = 0;
                pendingApplyIdle = false;
                unsigned long applyMs = millis();
                sharedData.t_ctrl_apply_ms = applyMs;
                sharedData.mode_latency_ms = applyMs - sharedData.t_cmd_set_ms;
            }
            else if (pendingApplyIdle)
            {
                // Already in idle mode, just clear flag
                pendingApplyIdle = false;
                unsigned long applyMs = millis();
                sharedData.t_ctrl_apply_ms = applyMs;
                sharedData.mode_latency_ms = applyMs - sharedData.t_cmd_set_ms;
            }
            xSemaphoreGive(mutex);
        }

        processI2CData();

        // Debug: once a second ask the ODrive what torque bound it ACTUALLY
        // holds; the TxSdo reply is printed next to what we last sent. A
        // mismatch means the endpoint ID is wrong for this ODrive firmware
        // and every cap write has been a silent no-op.
        if (g_wgDebug)
        {
            static unsigned long rbMs = 0;
            if (millis() - rbMs > 1000)
            {
                rbMs = millis();
                sendMotorRXRead(CAN_NODEID_M1, (ODriveEndpointID)g_epTorqueSoftMin);
            }
        }

        twai_message_t message;                     // Define the message structure
        if (twai_receive(&message, 0) == ESP_OK)    // Non-blocking receive from the CAN bus
        {
            uint16_t nodeID = (message.identifier >> 5) & 0x003F;
            uint16_t cmdID = message.identifier & 0x001F;

            float position = 0;
            float velocity = 0;
            float voltage = 0;
            float current = 0;
            uint32_t error = 0;

            switch (cmdID) // Check the command ID of the received message
            {
            case GET_POSVEL:
                memcpy(&position, &message.data[0], sizeof(float));
                memcpy(&velocity, &message.data[4], sizeof(float));
                if (nodeID == CAN_NODEID_M1)
                {
                    controlFeedback.m1_position = position;
                    controlFeedback.m1_velocity = velocity;
                    if (position < g_posSpanMin) g_posSpanMin = position;
                    if (position > g_posSpanMax) g_posSpanMax = position;

                    // ---- Home tracking / auto-sleep / wake ----
                    float pullPosNow = g_wgPullSign * position;
                    if (g_homePos > 1e8f)
                    {
                        g_homePos = pullPosNow; // first sample after boot
                        g_sleepPos = pullPosNow;
                        g_stillAnchorPos = pullPosNow;
                        g_stillSinceMs = millis();
                        g_dropLastPullPos = pullPosNow;
                    }
                    else if (pullPosNow < g_homePos && fabsf(velocity) < 0.5f)
                    {
                        // Follow the lowest position, but only when moving
                        // slowly — recoil bounce can transiently undershoot
                        g_homePos = pullPosNow;
                    }
                    float dHome = pullPosNow - g_homePos;

                    if (g_machineState == MS_SLEEP)
                    {
                        if (fabsf(pullPosNow - g_sleepPos) > g_wakeThresh)
                        {
                            // Wake FAST, but always into the low/slow
                            // profile — the zone taper restores the latched
                            // weight only as the user pulls out of the zone
                            g_machineState = (wgActive > 0.0f || wgState != WG_INACTIVE) ? MS_ACTIVE : MS_RECOIL;
                            controlState.mode = 1;
                            pendingApplyStrength = true;
                        }
                    }
                    else if (fabsf(pullPosNow - g_stillAnchorPos) > g_wakeThresh)
                    {
                        // Strap moved: re-anchor and restart the stillness
                        // timer. Position-window detection, NOT velocity —
                        // the encoder velocity estimate jitters past any
                        // sane threshold at rest (±0.2 turns/s on the
                        // bench), which kept resetting the timer forever.
                        g_stillAnchorPos = pullPosNow;
                        g_stillSinceMs = millis();
                    }
                    else if (dHome < g_homeZone && millis() >= g_testTorqueUntilMs &&
                             millis() - g_stillSinceMs > (unsigned long)g_sleepTimeoutMs)
                    {
                        // Parked inside the home zone and still: idle the
                        // axis to save power. Never sleeps mid-stroke — a
                        // static hold under load is outside the zone by
                        // definition, and holding still there just keeps
                        // the timer running without ever passing this gate.
                        g_machineState = MS_SLEEP;
                        g_sleepPos = pullPosNow;
                        g_stillSinceMs = millis();
                        controlState.mode = 0;
                        pendingApplyIdle = true;
                    }

                    // ---- Drop-catch state update ----
                    float pullVel = g_wgPullSign * velocity; // >0 pulling out
                    if (pullVel < -g_dropVMin)
                    {
                        // Fast reel-in: ratchet the shed fraction down —
                        // instantly, no slew; shedding fast is the feature
                        float sev = ((-pullVel) - g_dropVMin) / (g_dropVMax - g_dropVMin);
                        if (sev > 1.0f) sev = 1.0f;
                        float allowed = 1.0f - sev;
                        if (allowed < g_dropFrac) g_dropFrac = allowed;
                    }
                    else if (g_dropFrac < 1.0f)
                    {
                        // Latched: only outward travel releases it
                        float dOut = pullPosNow - g_dropLastPullPos;
                        if (dOut > 0.0f)
                        {
                            g_dropFrac += dOut / g_dropRestoreDist;
                            if (g_dropFrac > 1.0f) g_dropFrac = 1.0f;
                        }
                    }
                    g_dropLastPullPos = pullPosNow;

                    float desiredPosition = 0;

                    // Weight-update gate: compute the base force allowed at
                    // the current position (ramp / latch / cancel logic) and
                    // report gate state for the screen to poll.
                    sharedCfgData.target_force = wgTick() * 9.81f;
                    wgPublish();

                    // Update shared data in sharedData.h
                    updateMotorData(sharedCfgData.target_force, controlFeedback.m1_position, controlFeedback.m1_velocity);

                    // Apply force modulations
                    // float modulated_force = sharedCfgData.target_force;
                    float modulated_force = applyForceControl(sharedCfgData.target_force, controlFeedback.m1_position);
                    // modulated_force = applyDetentControl(modulated_force, controlFeedback.m1_position);
                    modulated_force = applyPulseControl(modulated_force);
                    modulated_force = applyRowModeControl(modulated_force, controlFeedback.m1_velocity, row_force_strength);

                    // set the torque values for m1 and m2 based on the target weight and velocity and torque_matrix
                    // interpolate the torque values from the matrix based on the target weight and velocity.
                    // g_forceCalScale calibrates commanded force to felt force at the handle.
                    float lookup_force = modulated_force * g_forceCalScale;
                    if (lookup_force <= torque_matrix[1][0])
                    {
                        torque_matrix_a = 0;
                        torque_matrix_b = 1;
                        torque_interpolation = (lookup_force - torque_matrix[0][0]) / (torque_matrix[1][0] - torque_matrix[0][0]);
                    }
                    else if (lookup_force <= torque_matrix[2][0])
                    {
                        torque_matrix_a = 1;
                        torque_matrix_b = 2;
                        torque_interpolation = (lookup_force - torque_matrix[1][0]) / (torque_matrix[2][0] - torque_matrix[1][0]);
                    }
                    else if (lookup_force <= torque_matrix[3][0])
                    {
                        torque_matrix_a = 2;
                        torque_matrix_b = 3;
                        torque_interpolation = (lookup_force - torque_matrix[2][0]) / (torque_matrix[3][0] - torque_matrix[2][0]);
                    }
                    else if (lookup_force <= torque_matrix[4][0])
                    {
                        torque_matrix_a = 3;
                        torque_matrix_b = 4;
                        torque_interpolation = (lookup_force - torque_matrix[3][0]) / (torque_matrix[4][0] - torque_matrix[3][0]);
                    }
                    else if (lookup_force <= torque_matrix[5][0])
                    {
                        torque_matrix_a = 4;
                        torque_matrix_b = 5;
                        torque_interpolation = (lookup_force - torque_matrix[4][0]) / (torque_matrix[5][0] - torque_matrix[4][0]);
                    }
                    else
                    {
                        Serial.printf("Target weight out of range: %f\n", lookup_force);
                        // set torque matrix to min
                        torque_matrix_a = 0;
                        torque_matrix_b = 1;
                        torque_interpolation = (lookup_force - torque_matrix[0][0]) / (torque_matrix[1][0] - torque_matrix[0][0]);
                    }

                    m1_minus_torque_soft_fn = torque_matrix[torque_matrix_a][2] + (torque_matrix[torque_matrix_b][2] - torque_matrix[torque_matrix_a][2]) * torque_interpolation;
                    m1_minus_torque_soft_fm = torque_matrix[torque_matrix_a][3] + (torque_matrix[torque_matrix_b][3] - torque_matrix[torque_matrix_a][3]) * torque_interpolation;
                    m1_plus_torque_soft_fn = torque_matrix[torque_matrix_a][4] + (torque_matrix[torque_matrix_b][4] - torque_matrix[torque_matrix_a][4]) * torque_interpolation;
                    m1_plus_torque_soft_fm = torque_matrix[torque_matrix_a][5] + (torque_matrix[torque_matrix_b][5] - torque_matrix[torque_matrix_a][5]) * torque_interpolation;
                    velocity_delta = torque_matrix[torque_matrix_a][1] + (torque_matrix[torque_matrix_b][1] - torque_matrix[torque_matrix_a][1]) * torque_interpolation;
                    vel_factor = ((velocity + velocity_delta) / velocity_delta) * 0.5;
                    vel_factor = (vel_factor > 0.5) ? 0.5 : vel_factor;
                    vel_factor = (vel_factor < -0.5) ? -0.5 : vel_factor;

                    vel_minus_factor = 0.5 - vel_factor;
                    vel_plus_factor = 0.5 + vel_factor;

                    float vel_positive = (velocity > 0) ? abs(velocity) : 0;
                    float vel_negative = (velocity < 0) ? abs(velocity) : 0;

                    m1_torque_soft = (m1_minus_torque_soft_fn + m1_minus_torque_soft_fm * vel_negative) * vel_minus_factor + (m1_plus_torque_soft_fn + m1_plus_torque_soft_fm * vel_positive) * vel_plus_factor;

                    m1_torque_soft_max = (m1_torque_soft > 0) ? m1_torque_soft : 0;
                    m1_torque_soft_min = (m1_torque_soft <= 0) ? m1_torque_soft : 0;

                    if (g_wgDebug)
                    {
                        static unsigned long wgDbgMs = 0;
                        if (millis() - wgDbgMs > 250)
                        {
                            wgDbgMs = millis();
                            Serial.printf("[wg] st=%d ms=%d act=%.1f pend=%.1f tgt=%.1fN Tq=%.3f d=%.2f drop=%.2f pos=%.3f v=%.2f mode=%d span=[%.3f..%.3f]\n",
                                          (int)wgState, (int)g_machineState, wgActive, wgPending,
                                          sharedCfgData.target_force, m1_torque_soft, dHome,
                                          g_dropFrac, controlFeedback.m1_position, velocity,
                                          (int)controlState.mode, g_posSpanMin, g_posSpanMax);
                        }
                    }

                    if (controlState.mode == 1) // update odrive motor controller over CAN
                    {
                        // Velocity mode with live torque limit: the axis
                        // chases an unreachable reel-in setpoint, saturated
                        // at the torque bound — so the bound IS the force.
                        // test_torque (cal probe, 10 s expiry) overrides for
                        // direct Nm -> felt-force calibration.
                        float sentTq = m1_torque_soft;
                        float velLim = zoneVelLimit(dHome);
                        if (millis() < g_testTorqueUntilMs)
                        {
                            sentTq = g_testTorque;
                            velLim = g_velLimit;
                        }
                        else
                        {
                            if (g_machineState == MS_RECOIL)
                            {
                                // Weight 0: constant gentle self-stow pull
                                sentTq = -g_recoilTorqueNm;
                                velLim = g_recoilVel;
                            }
                            // Near-home taper: pinch safety at the dock,
                            // per-rep soft pickup, wake-from-sleep guard
                            float mag = zoneTaperTorque(dHome, fabsf(sentTq));
                            // Drop-catch: shed the excess above the floor on
                            // a fast recoil; restored only by pulling out
                            if (g_dropFrac < 1.0f)
                            {
                                float excess = mag - g_recoilTorqueNm;
                                if (excess > 0.0f)
                                {
                                    mag = g_recoilTorqueNm + g_dropFrac * excess;
                                }
                            }
                            sentTq = (sentTq < 0.0f) ? -mag : mag;
                        }
                        g_effectiveTq = sentTq;
                        sendMotorVelocity(CAN_NODEID_M1, g_reelInVel);
                        if (g_useEpWrites)
                        {
                            // torque_soft_min (negative bound) IS the force;
                            // the current limit stays generous alongside.
                            sendMotorRX(CAN_NODEID_M1, (ODriveEndpointID)g_epTorqueSoftMin, -fabsf(sentTq));
                            // Push-toward-user bound: small and constant;
                            // refreshed at 1 Hz
                            static unsigned long softMaxMs = 0;
                            if (millis() - softMaxMs > 1000)
                            {
                                softMaxMs = millis();
                                sendMotorRX(CAN_NODEID_M1, TORQUE_SOFT_MAX_M1, g_softMaxNm);
                            }
                            sendMotorLimits(CAN_NODEID_M1, velLim, g_maxCurrentA);
                        }
                        else
                        {
                            sendMotorLimits(CAN_NODEID_M1, velLim, fabsf(sentTq) * g_ampsPerNm);
                        }
                        m1_torque_soft = sentTq; // reflect in telemetry
                    }
                }
                break;
            case GET_VBUS:
                memcpy(&voltage, &message.data[0], sizeof(float));
                memcpy(&current, &message.data[4], sizeof(float));
                controlFeedback.voltage = voltage;
                controlFeedback.current = current;
                // Serial.printf("Voltage: %f, Current: %f\n", voltage, current);

                if (xSemaphoreTake(mutex, 0) == pdTRUE)
                {
                    sharedStateData.voltage = voltage;
                    sharedStateData.current = current;
                    sharedStateData.newData = true;
                    xSemaphoreGive(mutex);
                }
                else
                {
                    handleError(ERROR_I2C_RECEIVE, "Failed to take mutex for processing CAN VBUS data");
                }
                break;
            case GET_MOTORERR:
                memcpy(&error, &message.data[0], sizeof(uint32_t));
                if (nodeID == CAN_NODEID_M1)
                {
                    controlState.error_state = error;
                }
                break;
            case GET_TXSDO:
            {
                uint16_t ep = (uint16_t)message.data[1] | ((uint16_t)message.data[2] << 8);
                float val = 0;
                memcpy(&val, &message.data[4], sizeof(float));
                Serial.printf("[wg] ODrive readback ep=0x%04X val=%.4f\n", ep, val);
                break;
            }
            default:
                break;
            }
        }

        // Minimal yield at end of cycle; main yield is in timing loop above
        taskYIELD();
    }
}

float applyForceControl(float target_force, float position)
{
    float force = target_force;
    noInterrupts(); // Begin critical section
    if (forceData.type == "off")
    {
        force = 0;
    }
    else if (forceData.type == "constant")
    {
        force = (forceData.strength / 100.0) * target_force;
    }
    else if (forceData.type == "linear")
    {
        if (position < forceData.start_position)
        {
            force = 0;
        }
        else if (position > forceData.saturation_position)
        {
            force = (forceData.strength / 100.0) * target_force;
        }
        else
        {
            float scale = (position - forceData.start_position) / (forceData.saturation_position - forceData.start_position);
            // if scale is less than 0, set it to 0
            scale = (scale < 0) ? 0 : scale;
            // if scale is greater than 1, set it to 1
            scale = (scale > 1) ? 1 : scale;
            float strength = forceData.start_strength + scale * (forceData.strength - forceData.start_strength);
            force = (strength / 100.0) * target_force;
        }
    }
    // forceData.updated = false;
    interrupts(); // End critical section
    return force;
}

float applyDetentControl(float target_force, float position)
{
    float force = target_force;
    noInterrupts(); // Begin critical section
    if (detentData.type != "on")
    {
        force = target_force;
    }
    else
    {
        if (position >= detentData.start_position)
        {
            int step = (position - detentData.start_position) / detentData.step_position;
            if (step < detentData.total_steps)
            {
                force = target_force * (1.0 - (detentData.strength / 100.0));
            }
        }
    }
    // detentData.updated = false;
    interrupts(); // End critical section
    return force;
}

float applyPulseControl(float target_force)
{
    noInterrupts(); // Begin critical section
    static float adder_force = 0;
    float force = target_force;
    float max_force = 0.4 * target_force;

    int frequency = pulseData.frequency;
    if (frequency < 3)
    {
        frequency = 3;
    }

    if (pulseData.type == "on")
    {
        unsigned long currentTime = millis();
        float period = 1000.0 / pulseData.frequency; // Period in milliseconds

        // Calculate the time fraction within the period
        float timeFraction = fmod(static_cast<float>(currentTime), period) / period;

        // Calculate sine wave modulation
        adder_force = sin(2 * PI * timeFraction) * (pulseData.strength / 100.0) * max_force;

        // Calculate final force
        force = target_force + adder_force;
    }
    interrupts(); // End critical section

    return force;
}

float applyRowModeControl(float target_force, float velocity, float &row_force_strength)
{
    float force = target_force;
    noInterrupts(); // Begin critical section
    // float delta_velocity_threshold = (0.5 + rowData.gear_ratio / 10);
    float delta_velocity_threshold = 0.5;
    float velocity_slow_rate = 0.02;
    float velocity_gain_rate = 0.005;
    float force_change_rate = 7;

    if (rowData.type != "off")
    {
        float virtual_velocity = sharedData.virtual_velocity;
        float damping_loss = (velocity_slow_rate * (rowData.damping / 100.0)) * (virtual_velocity);
        float velocity_delta = velocity - virtual_velocity;

        if (velocity_delta > 0)
        {
            float force = (velocity_delta / delta_velocity_threshold) * (target_force);
            virtual_velocity += (velocity_delta * velocity_gain_rate);
            row_force_strength += force_change_rate;
            row_force_strength = (row_force_strength > 100) ? 100 : row_force_strength;
        }
        else
        {
            row_force_strength -= force_change_rate;
        }

        // Min force
        row_force_strength = (row_force_strength < 0) ? 0 : row_force_strength;
        force = row_force_strength / 100.0 * force;

        // Min virtual velocity is 0
        virtual_velocity -= damping_loss;
        virtual_velocity = (virtual_velocity < 0) ? 0 : virtual_velocity;
        sharedData.virtual_velocity = virtual_velocity;
    }
    interrupts(); // End critical section
    return force;
}
