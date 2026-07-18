#include "MotorControl.h"
#include "ErrorHandling.h"
#include "CANCommunication.h"
#include "I2CCommunication.h"
#include "Logging.h"
#include "SharedData.h" // Include the shared data header
#include <math.h>

ControlFeedback controlFeedback; // Define the controlFeedback structure

// declare functions — force-profile positions are REVS FROM THE DOCK
// (dHome), not raw encoder position, so profiles repeat identically every
// rep and survive home re-establishment across sessions
float applyForceControl(float target_force, float dHome);
float applyDetentControl(float target_force, float dHome);
float applyPulseControl(float target_force);

// ========================= moteus transport notes ===========================
// Force rendering keeps the ODrive-era architecture: the axis chases an
// unreachable reel-in velocity while a live torque cap IS the rendered
// force. On moteus that is ONE frame per control tick — position mode with
// position=NaN (target re-anchors at the current output every frame),
// velocity = the zone-limited reel-in setpoint, maximum_torque = the cap —
// plus a query bit so the reply carries position/velocity/torque back.
// There is no pre-staging choreography like the ODrive needed: the same
// frame that (re)enters position mode carries the torque bound, so the axis
// can never engage on a stale limit.
//
// Differences vs the ODrive setup, called out for bring-up:
// - The moteus per-command torque cap is SYMMETRIC (±). The ODrive's tiny
//   separate push-toward-user bound (torque_soft_max ~1 lbf) has no direct
//   equivalent; braking of a fast inrushing strap is instead bounded by the
//   drop-catch shed, which collapses the cap to the recoil floor within one
//   control cycle of a fast reel-in. Watch this on the bench.
// - Position units are output REVOLUTIONS (== ODrive motor turns on this
//   near-direct drivetrain), velocity rev/s, torque Nm — values carry over.
// - Feedback is query-driven at the control tick rate instead of the
//   ODrive's 100 Hz broadcast, so the force pipeline now runs every tick.
// - servo.default_timeout_s is left at the moteus config default: if this
//   stream ever stalls, the axis drops to its timeout mode (configure
//   "stopped" via moteus_tool) — strap goes slack, which is the safe way
//   for this machine to fail.

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
// Position units are output revolutions (moteus reports the output shaft;
// with the near-direct spool drive this matches the old ODrive turns).
// Measure exactly: with [wg] telemetry on, note `pos` at strap rest vs
// pulled a known distance, then set these live via the cal command
// (delta_x/delta_y/deadband/cancel) — no reflash needed.
#define WG_MIN_PROGRESS_REARM (0.10f) // below this ramp progress a backtrack re-arms instead of cancelling
// Bench evidence (2.0-turn deadband unreachable by a full pull; 0.1 turns
// took a noticeable pull) puts the full stroke at only ~1-2 position units,
// so the gate distances default to fractions of a unit. Calibrate with the
// span=[min,max] readout in the [wg] telemetry after one full pull, then
// set exact values live via cal (delta_x/delta_y/deadband/cancel).
// Stroke measurement 2026-07-11 (second bench log): pos reached 0.63+ and
// was still climbing mid-pull — real stroke is at least ~0.8 units. NOTE:
// re-verify stroke and pull_sign on the moteus encoder before trusting
// these; the x1's zero/direction will differ from the ODrive's.
// Distances rescaled 2026-07-13 for the new drivetrain's ~5-rev stroke
// (old spool was ~0.8 rev full stroke; these were fractions of that).
static float g_wgPullSign = +1.0f;       // +1 if position increases as the user pulls out; cal "pull_sign" flips it
static float g_wgDeltaX = 1.1f;          // ramp ~1/4 of stroke
static float g_wgDeltaY = 0.3f;          // latch hold
static float g_wgDeadband = 0.15f;       // engage deadband
static float g_wgCancelRetract = 0.4f;   // cancel backtrack

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

// Calibration anchor: the ODrive-era drivetrain measured 0.5 Nm ~= 1 lbf
// (2 lbf/Nm), and the torque matrix rows are built on that. The NEW
// RI115-pair drivetrain: eyeball 0.4, scale-corrected x0.8 => 0.32 with
// screen 20 measuring 20 while PULLING (i.e. 0.32 mapped set weight to the
// pulling direction, which runs (1+K_TRUE) heavy from capstan friction).
// With directional friction comp, the anchor moves to the MIDPOINT so that
// holds and slow motion land ON the set weight: scale = 0.32 * (1+K_TRUE)
// = 0.32 * 1.25 = 0.40. This is independent of fric_k (the comp fraction);
// do NOT re-anchor when tuning fric_k. First anchor attempt used (1+fric_k)
// = 0.375 — wrong: it made every slow movement read 17% heavy (2026-07-14).
#define FORCE_CAL_SCALE (0.40f)
static float g_forceCalScale = FORCE_CAL_SCALE;

// Torque matrix at file scope so the live-cal setter can adjust the zero
// row. Single-motor machine (M1 only; the high-range M2 motor was removed).
// 6 rows x 6 columns:
// force, velocity_delta, -M1_Fn, -M1_Fm, M1_Fn, M1_Fm
// minus sign = motion towards the device, positive = towards the user.
// Fn is static torque, Fm the velocity coefficient: F = Fn + Fm * v.
// Row 0 is the ZERO row: near-zero torque so light forces are actually
// renderable; it also sets the strap keep-taut tension. Rows in REAL Nm
// from the 0.5 Nm ~= 1 lbf anchor: LINEAR law, torque = 0.5 * screen units
// (screen unit -> 9.81 N internally), all the way up — the matrix shapes
// feel (velocity blend, keep-taut floor), it does NOT clamp. The upper rows
// exceed what the motor can physically deliver on purpose: maximum_torque
// is a BOUND, not a setpoint, so the moteus just saturates at its
// configured current/torque limits and force tops out gracefully. The
// ceiling lives in the moteus config (servo.max_current_A), not here.
// Symmetric pull-in/pull-out to start; add eccentric bias via row edits.
// velocity_delta (col 1) is 8 revs/s so the direction blend transitions
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

// Reel-in chase setpoint and the away-from-home velocity limit. The
// commanded moteus velocity each cycle is -min(|g_reelInVel|, zone limit),
// which reproduces the ODrive's setpoint+vel_limit behavior in one value.
static float g_velLimit = 25.0f;    // rev/s limit away from home; > |reel-in setpoint|
static float g_reelInVel = -20.0f;  // velocity setpoint (unreachable by design)

void setDriveParams(float reelInVel, float velLimit)
{
    if (reelInVel > 0.0f && reelInVel <= 100.0f) g_reelInVel = -reelInVel;
    if (velLimit > 0.0f && velLimit <= 100.0f) g_velLimit = velLimit;
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
//   stopped, zero current). Any movement past g_wakeThresh wakes into the
//   low/slow profile immediately; the zone taper then restores the latched
//   weight as the user pulls out. The latched weight NEVER changes across
//   sleep, and force is never re-applied without user travel.
enum MachineState : uint8_t
{
    MS_SLEEP = 0,  // axis stopped, watching for movement (also the boot state)
    MS_RECOIL = 1, // weight 0: gentle self-stow
    MS_ACTIVE = 2  // weight set: normal force pipeline
};
static MachineState g_machineState = MS_SLEEP;
static float g_homePos = 1e9f;         // running min of pull position
static float g_homeZone = 1.0f;        // revs from home (~20% of the new
                                       // ~5-rev stroke; was 0.75 of the old
                                       // 0.8-rev stroke)
static float g_recoilTorqueNm = 0.3f;  // ~1.5 lbf recoil force / cap floor
                                       // (new drivetrain: ~5 lbf/Nm)
static float g_recoilVel = 8.0f;       // rev/s stow speed at weight 0 —
                                       // sized to the ~5-rev stroke so the
                                       // strap keeps up with a returning hand
static float g_dockVel = 2.0f;         // rev/s ceiling right AT the dock
                                       // (pinch/jolt guard; recoil and the
                                       // active zone both lerp down to this)
static float g_softMaxNm = 0.5f;       // retained knob: the old ODrive push-out
                                       // bound; no moteus per-command equivalent
                                       // (see transport notes above)
static float g_sleepTimeoutMs = 10000.0f;
static float g_wakeThresh = 0.25f;     // revs of movement that wake from sleep
                                       // (scaled to the ~5-rev stroke; 0.05
                                       // woke on vibration)
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
// On moteus this shed matters MORE than it did on the ODrive: it is also
// what bounds the symmetric cap's braking of a flying strap.
// Thresholds rescaled for the new drivetrain: controlled eccentrics now run
// 3-5 rev/s (5-rev stroke), so shedding must start well above that or the
// anti-runaway shaves force off every normal lower.
static float g_dropVMin = 6.0f;        // rev/s reel-in where shedding starts
static float g_dropVMax = 15.0f;       // rev/s reel-in where only the floor remains
static float g_dropRestoreDist = 1.5f; // revs of outward travel for full restore
static float g_dropFrac = 1.0f;        // live shed fraction: 1 = full force, 0 = floor only
static float g_dropLastPullPos = 0.0f;

void setDropCatch(float vMin, float vMax, float restoreDist)
{
    if (vMin > 0.0f && vMin <= 50.0f) g_dropVMin = vMin;
    if (vMax > 0.0f && vMax <= 50.0f && vMax > g_dropVMin) g_dropVMax = vMax;
    if (restoreDist > 0.0f && restoreDist <= 10.0f) g_dropRestoreDist = restoreDist;
}

// ============================== Row mode ====================================
// Virtual Concept2-style flywheel. One state variable — the wheel speed, in
// strap rev/s so it compares directly with measured pull velocity — plus a
// one-way-clutch rule:
// - DRIVE (pulling faster than the wheel spins): force = coupling stiffness
//   x overspeed, capped; the same coupling torque spins the wheel up. A
//   stiff-but-not-rigid coupling gives a crisp catch without differentiating
//   noisy velocity into acceleration.
// - RECOVERY (slower than the wheel / returning): the clutch freewheels —
//   force drops to a light return tension and the strap follows the hands.
// - The wheel always decays QUADRATICALLY (drag x speed^2): that v^2 air-drag
//   decay is most of what makes it feel like a rowing flywheel rather than
//   molasses. The user's "drag" 1-10 scales the coefficient (C2 damper);
//   "gear" scales force-per-overspeed only, so a heavier gear reads as a
//   bigger oar without changing the wheel's glide.
// Stroke metrics fall out of the clutch state: drive-start to drive-start is
// a stroke (SPM), and integrating force x speed over the drive gives watts.
// Rendered through the same torque-cap pipeline as strength mode, so the
// drop-catch shed and sleep machinery stay in force underneath; only the
// near-home taper shrinks to a pinch-only zone (a full-size zone would kill
// the catch, which happens right beside the dock).
static bool g_rowEnabled = false;
static uint8_t g_rowGear = 1; // 1-10, force scaling (5 = 1.0x); start gentle
static uint8_t g_rowDrag = 1; // 1-10, wheel decay (C2 damper); start gentle
// Tunables — live via cal {"row_kc":..} etc.; bake in values once found.
// Scaled to the NEW drivetrain (bench 2026-07-13: ~5 lbf/Nm, ~5 revs per
// stroke): kc=2.5 at gear 5 puts a 1 rev/s overspeed catch around 12 lbf.
// Inertia is deliberately high — a wheel that matches hand speed too fast
// collapses the overspeed mid-drive and the force flaps (felt as chatter);
// with inertia 5 the wheel stays behind the hands through a whole drive,
// and the drag term sets the sustained equilibrium force.
static float g_rowKc = 2.5f;        // Nm per rev/s of overspeed at gear 5
static float g_rowInertia = 5.0f;   // wheel inertia: dw/dt = Nm / inertia
static float g_rowDragBase = 0.15f; // decay coefficient per drag unit
static float g_rowReturnTq = 0.6f;  // Nm strap tension during recovery
                                    // (~3 lbf, top of the real-erg range —
                                    // in saturated-chase rendering the
                                    // recovery tension IS the chase torque,
                                    // so this is also the catch-up muscle)
static float g_rowMaxNm = 10.0f;    // ceiling on rendered row force (~50 lbf)
static float g_rowZone = 0.5f;      // pinch-only taper zone, revs from home
                                    // (scaled to the ~5-rev stroke)
static float g_rowFly = 0.0f;       // wheel speed state (strap rev/s)
static float g_rowFOut = 0.0f;      // smoothed rendered force (anti-chatter)
static bool g_rowDriving = false;
static unsigned long g_rowDriveStartMs = 0; // start of the current/last drive
static float g_rowStrokeJoules = 0.0f;      // work accumulated this stroke
static float g_rowWatts = 0.0f;             // smoothed per-stroke power
static float g_rowSpm = 0.0f;               // smoothed strokes per minute

void setRowTuning(float kc, float inertia, float dragBase, float returnTq,
                  float maxNm, float zone)
{
    if (kc > 0.0f && kc <= 100.0f) g_rowKc = kc;
    if (inertia > 0.0f && inertia <= 100.0f) g_rowInertia = inertia;
    if (dragBase > 0.0f && dragBase <= 10.0f) g_rowDragBase = dragBase;
    if (returnTq >= 0.0f && returnTq <= 2.0f) g_rowReturnTq = returnTq;
    if (maxNm > 0.0f && maxNm <= 100.0f) g_rowMaxNm = maxNm;
    if (zone > 0.0f && zone <= 10.0f) g_rowZone = zone;
}

// Reset wheel and metrics on every row-mode entry
static void rowReset()
{
    g_rowFly = 0.0f;
    g_rowFOut = 0.0f;
    g_rowDriving = false;
    g_rowDriveStartMs = 0;
    g_rowStrokeJoules = 0.0f;
    g_rowWatts = 0.0f;
    g_rowSpm = 0.0f;
}

// Advance the flywheel with the latest pull velocity; returns the force (Nm)
// to render. vPull > 0 = pulling out.
static float rowTick(float vPull, float dt)
{
    float gearScale = 0.4f + 0.12f * (float)g_rowGear; // gear 5 = 1.0
    float dq = g_rowDragBase * (float)g_rowDrag;

    // One-way clutch with WIDE hysteresis (rev/s bands sized to the new
    // drivetrain's stroke speeds) so crossover jitter can't machine-gun
    // the drive/recovery transitions
    bool engaged = g_rowDriving ? (vPull > g_rowFly - 0.15f)
                                : (vPull > g_rowFly + 0.25f);
    float force;
    if (engaged && vPull > 0.0f)
    {
        float over = vPull - g_rowFly;
        if (over < 0.0f) over = 0.0f;
        force = gearScale * g_rowKc * over + g_rowReturnTq;
        if (force > g_rowMaxNm) force = g_rowMaxNm;
        // The coupling torque (gear-independent) spins the wheel up
        g_rowFly += ((g_rowKc * over) / g_rowInertia) * dt;
    }
    else
    {
        engaged = false;
        force = g_rowReturnTq;
    }
    // Quadratic decay runs in both phases (air drag never sleeps)
    g_rowFly -= ((dq * g_rowFly * g_rowFly) / g_rowInertia) * dt;
    if (g_rowFly < 0.0f) g_rowFly = 0.0f;

    // Anti-chatter shaping: instant attack (the catch stays crisp), ~150 ms
    // release — brief clutch dropouts mid-drive get bridged instead of
    // rendered as force slams
    if (force >= g_rowFOut)
    {
        g_rowFOut = force;
    }
    else
    {
        g_rowFOut += (force - g_rowFOut) * fminf(dt / 0.15f, 1.0f);
    }
    force = g_rowFOut;

    // ---- Stroke bookkeeping ----
    unsigned long now = millis();
    if (engaged && !g_rowDriving)
    {
        // Drive start: close out the previous stroke (drive-start to
        // drive-start) and report its power and rate
        if (g_rowDriveStartMs != 0)
        {
            float periodS = (now - g_rowDriveStartMs) / 1000.0f;
            if (periodS > 0.8f && periodS < 20.0f)
            {
                float watts = g_rowStrokeJoules / periodS;
                g_rowWatts = (g_rowWatts <= 0.0f) ? watts : (0.6f * g_rowWatts + 0.4f * watts);
                float spm = 60.0f / periodS;
                g_rowSpm = (g_rowSpm <= 0.0f) ? spm : (0.6f * g_rowSpm + 0.4f * spm);
            }
        }
        g_rowDriveStartMs = now;
        g_rowStrokeJoules = 0.0f;
        g_rowDriving = true;
    }
    else if (!engaged && g_rowDriving)
    {
        g_rowDriving = false;
    }
    if (g_rowDriving && vPull > 0.0f)
    {
        // Power at the spool: torque x angular velocity (rev/s -> rad/s)
        g_rowStrokeJoules += force * vPull * (2.0f * PI) * dt;
    }

    // The wheel speed IS the virtual velocity the app already charts
    updateVirtualVelocity(g_rowFly);
    return force;
}
// ============================ end row mode ==================================

// ================== Directional friction compensation =======================
// Capstan-style drivetrain friction eats a load-PROPORTIONAL bite (~25%,
// scale-measured 2026-07-14: set 10 -> 11 up / 6 down; set 20 -> 20 / 12.5):
// the user feels cmd*(1+k) pulling out and cmd*(1-k) lowering. Compensate by
// commanding excess/(1 + k*s), where s is the shaped pull direction in
// [-1..+1] — lowering gets boosted, pulling trimmed, symmetric about the
// calibrated midpoint, and a hold (s~0) lands on the set weight.
// g_fricK is deliberately ~70% OF THE MEASURED 0.25: overcompensation is
// negative damping (the machine would feed energy into motion) and friction
// varies with wrap/temperature, so the margin is not optional. FORCE_CAL_
// SCALE is re-anchored *(1+k) so the pulling direction still measures true.
static float g_fricK = 0.17f;    // effective comp fraction (cal "fric_k")
static float g_fricBand = 0.6f;  // rev/s at which s saturates (cal "fric_band")
                                 // — low enough that deliberate slow reps
                                 // still earn most of the compensation
static float g_fricS = 0.0f;     // shaped direction state [-1..+1]

void setFrictionComp(float k, float band)
{
    if (k >= 0.0f && k <= 0.5f) g_fricK = k;
    if (band > 0.05f && band <= 10.0f) g_fricBand = band;
}

// Returns the multiplier for the force excess (1 = no change)
static float frictionCompFactor(float pullVel, float dt)
{
    if (g_fricK <= 0.001f)
    {
        g_fricS = 0.0f;
        return 1.0f;
    }
    float target = pullVel / g_fricBand;
    if (target > 1.0f) target = 1.0f;
    if (target < -1.0f) target = -1.0f;
    // Symmetric ~120 ms shaping: tracks rep direction changes without
    // stepping at the v=0 crossings (top/bottom of every rep)
    g_fricS += (target - g_fricS) * fminf(dt / 0.12f, 1.0f);
    return 1.0f / (1.0f + g_fricK * g_fricS);
}
// ================ end directional friction compensation =====================

// ====================== Concentric-only unloading ==========================
// Safety feature for exercises where the eccentric/hold must be light: the
// set percentage (0-100, from the strength screen or cal "con_pct") scales
// away the force whenever the user is NOT actively pulling out. 0 = normal
// (full force both directions), 100 = force only during the concentric pull
// — stop or lower and it fades to the keep-taut floor (recoil is preserved;
// the strap always stows). Direction comes from pull velocity blended over
// [g_conVLo..g_conVHi] rev/s, shaped fast-attack / slow-release so a brief
// hesitation mid-pull doesn't dump the weight (the row-chatter lesson).
static float g_conPct = 0.0f;    // 0-100
static float g_conVLo = 0.3f;    // rev/s: at/below this you are "not pulling"
static float g_conVHi = 1.5f;    // rev/s: at/above this you get full force
static float g_conFactor = 1.0f; // shaped live factor (1 = full force)

void setConcentric(float pct, float vLo, float vHi)
{
    if (pct >= 0.0f && pct <= 100.0f) g_conPct = pct;
    if (vLo >= 0.0f && vLo < 10.0f) g_conVLo = vLo;
    if (vHi > g_conVLo && vHi <= 20.0f) g_conVHi = vHi;
}

// Returns the live force factor for the current pull velocity (1 = full).
static float concentricFactor(float pullVel, float dt)
{
    if (g_conPct <= 0.0f)
    {
        g_conFactor = 1.0f;
        return 1.0f;
    }
    float t = (pullVel - g_conVLo) / (g_conVHi - g_conVLo);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    float target = (1.0f - g_conPct / 100.0f) + (g_conPct / 100.0f) * t;
    // Fast attack (force is there the instant they pull), ~300 ms release
    // (stopping fades the weight rather than dropping it off a cliff)
    if (target >= g_conFactor)
    {
        g_conFactor += (target - g_conFactor) * fminf(dt / 0.06f, 1.0f);
    }
    else
    {
        g_conFactor += (target - g_conFactor) * fminf(dt / 0.30f, 1.0f);
    }
    return g_conFactor;
}
// ==================== end concentric-only unloading =========================

// Near-home taper: scale the excess force above the recoil floor by the
// distance from home — floor-only at the dock, the full requested force at
// the zone edge. Because it scales the REQUEST rather than capping to a
// fixed ceiling, there is no step at the boundary for any magnitude; the
// physical ceiling remains the moteus current-limit chain, never this taper.
static float zoneTaperTorque(float dHome, float magNm)
{
    if (dHome >= g_homeZone) return magNm;
    if (dHome < 0.0f) dHome = 0.0f;
    float excess = magNm - g_recoilTorqueNm;
    if (excess <= 0.0f) return magNm;
    return g_recoilTorqueNm + (dHome / g_homeZone) * excess;
}

// Velocity limit lerped from dock-slow at home to normal at the zone edge
static float zoneVelLimit(float dHome)
{
    if (dHome >= g_homeZone) return g_velLimit;
    if (dHome < 0.0f) dHome = 0.0f;
    return g_dockVel + (dHome / g_homeZone) * (g_velLimit - g_dockVel);
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
        // Row status: param echoes double as the enable indicator (0 = off),
        // so the screen can detect and resend a dropped enable/disable.
        sharedStateData.row_gear = g_rowEnabled ? g_rowGear : 0;
        sharedStateData.row_drag = g_rowEnabled ? g_rowDrag : 0;
        sharedStateData.row_spm = (uint8_t)constrain((int)lroundf(g_rowSpm), 0, 255);
        sharedStateData.row_watts = g_rowWatts;
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
                // (auto-sleep will stop the axis once the strap parks)
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

// Latest command values the tick sender streams to the moteus. Written by
// processMoteusFeedback(), read by the sender. Boot defaults are the safe
// profile: recoil-floor cap at recoil speed until real feedback arrives.
static float g_cmdMaxTorqueNm = 0.1f;
static float g_cmdVelocity = -2.0f;

// The old GET_POSVEL handler body: home tracking, sleep/wake, drop-catch,
// the weight gate, and the force pipeline. Runs once per feedback reply and
// leaves the values to stream in g_cmdMaxTorqueNm / g_cmdVelocity.
static void processMoteusFeedback(float position, float velocity)
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
            // weight only as the user pulls out of the zone.
            // Row mode survives sleep: it resumes with a
            // stopped wheel, so the first catch is gentle.
            g_machineState = (g_rowEnabled || wgActive > 0.0f || wgState != WG_INACTIVE) ? MS_ACTIVE : MS_RECOIL;
            controlState.mode = 1;
            pendingApplyStrength = true;
        }
    }
    else if (fabsf(pullPosNow - g_stillAnchorPos) > g_wakeThresh)
    {
        // Strap moved: re-anchor and restart the stillness
        // timer. Position-window detection, NOT velocity —
        // the encoder velocity estimate jitters past any
        // sane threshold at rest, which kept resetting the
        // timer forever on the bench.
        g_stillAnchorPos = pullPosNow;
        g_stillSinceMs = millis();
    }
    else if (dHome < g_homeZone && millis() >= g_testTorqueUntilMs &&
             millis() - g_stillSinceMs > (unsigned long)g_sleepTimeoutMs)
    {
        // Parked inside the home zone and still: stop the
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

    if (g_rowEnabled)
    {
        // ---- Row pipeline: the flywheel model replaces the weight gate ----
        static unsigned long rowMs = 0;
        unsigned long nowMs = millis();
        float rowDt = (rowMs == 0) ? 0.0f : (nowMs - rowMs) / 1000.0f;
        if (rowDt > 0.05f) rowDt = 0.05f; // guard long gaps (re-entry, CAN stall)
        rowMs = nowMs;

        float rowF = rowTick(pullVel, rowDt);
        sharedCfgData.target_force = rowF;
        wgPublish();
        updateMotorData(rowF, controlFeedback.m1_position, controlFeedback.m1_velocity);

        // Pinch-only zone at the dock: the catch happens just outside it and
        // must stay crisp, so this is much smaller than the strength zone
        float mag = rowF;
        float velLim = g_velLimit;
        if (dHome < g_rowZone)
        {
            float dH = (dHome < 0.0f) ? 0.0f : dHome;
            float excess = mag - g_rowReturnTq;
            if (excess > 0.0f)
            {
                mag = g_rowReturnTq + (dH / g_rowZone) * excess;
            }
            velLim = g_dockVel + (dH / g_rowZone) * (g_velLimit - g_dockVel);
        }
        // Drop-catch shed still applies: with moteus's symmetric torque cap
        // this is also what bounds braking of an unheld flying strap
        if (g_dropFrac < 1.0f)
        {
            float excess = mag - g_rowReturnTq;
            if (excess > 0.0f)
            {
                mag = g_rowReturnTq + g_dropFrac * excess;
            }
        }
        float sentTq = -mag;
        if (millis() < g_testTorqueUntilMs)
        {
            sentTq = g_testTorque;
            velLim = g_velLimit;
        }
        g_effectiveTq = sentTq;
        g_cmdMaxTorqueNm = fabsf(sentTq);
        g_cmdVelocity = -fminf(fabsf(g_reelInVel), velLim);

        if (g_wgDebug)
        {
            static unsigned long rowDbgMs = 0;
            if (millis() - rowDbgMs > 250)
            {
                rowDbgMs = millis();
                Serial.printf("[row] fly=%.2f v=%.2f F=%.2fNm W=%.0f spm=%.0f g=%u d=%u drv=%d drop=%.2f pos=%.3f\n",
                              g_rowFly, pullVel, mag, g_rowWatts, g_rowSpm,
                              (unsigned)g_rowGear, (unsigned)g_rowDrag,
                              (int)g_rowDriving, g_dropFrac, controlFeedback.m1_position);
            }
        }
        return;
    }

    // Weight-update gate: compute the base force allowed at
    // the current position (ramp / latch / cancel logic) and
    // report gate state for the screen to poll.
    sharedCfgData.target_force = wgTick() * 9.81f;
    wgPublish();

    // Update shared data in sharedData.h
    updateMotorData(sharedCfgData.target_force, controlFeedback.m1_position, controlFeedback.m1_velocity);

    // Apply force modulations (all position-based profiles run on dHome —
    // revs of pull from the dock — so they repeat per rep)
    float modulated_force = applyForceControl(sharedCfgData.target_force, dHome);
    modulated_force = applyDetentControl(modulated_force, dHome);
    modulated_force = applyPulseControl(modulated_force);

    // Interpolate the torque values from the matrix based on the target
    // weight and velocity. g_forceCalScale calibrates commanded force to
    // felt force at the handle.
    int torque_matrix_a = 0;
    int torque_matrix_b = 1;
    float torque_interpolation = 0;
    float lookup_force = modulated_force * g_forceCalScale;
    if (lookup_force <= torque_matrix[1][0])
    {
        torque_matrix_a = 0;
        torque_matrix_b = 1;
    }
    else if (lookup_force <= torque_matrix[2][0])
    {
        torque_matrix_a = 1;
        torque_matrix_b = 2;
    }
    else if (lookup_force <= torque_matrix[3][0])
    {
        torque_matrix_a = 2;
        torque_matrix_b = 3;
    }
    else if (lookup_force <= torque_matrix[4][0])
    {
        torque_matrix_a = 3;
        torque_matrix_b = 4;
    }
    else if (lookup_force <= torque_matrix[5][0])
    {
        torque_matrix_a = 4;
        torque_matrix_b = 5;
    }
    else
    {
        Serial.printf("Target weight out of range: %f\n", lookup_force);
        // set torque matrix to min
        torque_matrix_a = 0;
        torque_matrix_b = 1;
    }
    torque_interpolation = (lookup_force - torque_matrix[torque_matrix_a][0]) /
                           (torque_matrix[torque_matrix_b][0] - torque_matrix[torque_matrix_a][0]);

    float m1_minus_torque_soft_fn = torque_matrix[torque_matrix_a][2] + (torque_matrix[torque_matrix_b][2] - torque_matrix[torque_matrix_a][2]) * torque_interpolation;
    float m1_minus_torque_soft_fm = torque_matrix[torque_matrix_a][3] + (torque_matrix[torque_matrix_b][3] - torque_matrix[torque_matrix_a][3]) * torque_interpolation;
    float m1_plus_torque_soft_fn = torque_matrix[torque_matrix_a][4] + (torque_matrix[torque_matrix_b][4] - torque_matrix[torque_matrix_a][4]) * torque_interpolation;
    float m1_plus_torque_soft_fm = torque_matrix[torque_matrix_a][5] + (torque_matrix[torque_matrix_b][5] - torque_matrix[torque_matrix_a][5]) * torque_interpolation;
    float velocity_delta = torque_matrix[torque_matrix_a][1] + (torque_matrix[torque_matrix_b][1] - torque_matrix[torque_matrix_a][1]) * torque_interpolation;
    float vel_factor = ((velocity + velocity_delta) / velocity_delta) * 0.5;
    vel_factor = (vel_factor > 0.5) ? 0.5 : vel_factor;
    vel_factor = (vel_factor < -0.5) ? -0.5 : vel_factor;

    float vel_minus_factor = 0.5 - vel_factor;
    float vel_plus_factor = 0.5 + vel_factor;

    float vel_positive = (velocity > 0) ? fabsf(velocity) : 0;
    float vel_negative = (velocity < 0) ? fabsf(velocity) : 0;

    float m1_torque_soft = (m1_minus_torque_soft_fn + m1_minus_torque_soft_fm * vel_negative) * vel_minus_factor + (m1_plus_torque_soft_fn + m1_plus_torque_soft_fm * vel_positive) * vel_plus_factor;

    // State-aware final torque and velocity, streamed by the tick sender.
    // test_torque (cal probe, 10 s expiry) overrides for direct Nm ->
    // felt-force calibration.
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
            // Weight 0: gentle self-stow at full recoil speed, slowing to
            // dock speed over the home zone so the strap parks softly
            sentTq = -g_recoilTorqueNm;
            float dH = (dHome < 0.0f) ? 0.0f : fminf(dHome, g_homeZone);
            velLim = g_dockVel + (dH / g_homeZone) * (g_recoilVel - g_dockVel);
        }
        // Near-home taper: pinch safety at the dock,
        // per-rep soft pickup, wake-from-sleep guard
        float mag = zoneTaperTorque(dHome, fabsf(sentTq));
        // Shared dt for the velocity-shaped stages (concentric, friction)
        static unsigned long shapeMs = 0;
        unsigned long shapeNow = millis();
        float shapeDt = (shapeMs == 0) ? 0.0f : (shapeNow - shapeMs) / 1000.0f;
        if (shapeDt > 0.05f) shapeDt = 0.05f;
        shapeMs = shapeNow;
        // Concentric-only unloading: scale the excess by the shaped
        // pull-direction factor (no-op at con_pct 0)
        {
            float cf = concentricFactor(pullVel, shapeDt);
            if (cf < 1.0f)
            {
                float excess = mag - g_recoilTorqueNm;
                if (excess > 0.0f)
                {
                    mag = g_recoilTorqueNm + cf * excess;
                }
            }
        }
        // Directional friction compensation on whatever force survived the
        // unloading (boosts lowering, trims pulling; runs BEFORE drop-catch
        // so a real runaway still sheds the boosted force)
        {
            float ff = frictionCompFactor(pullVel, shapeDt);
            if (ff != 1.0f)
            {
                float excess = mag - g_recoilTorqueNm;
                if (excess > 0.0f)
                {
                    mag = g_recoilTorqueNm + ff * excess;
                }
            }
        }
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
    // The chase velocity is the reel-in setpoint clamped by the zone/recoil
    // limit — one value replaces the ODrive's setpoint + Set_Limits pair.
    g_cmdMaxTorqueNm = fabsf(sentTq);
    g_cmdVelocity = -fminf(fabsf(g_reelInVel), velLim);

    if (g_wgDebug)
    {
        static unsigned long wgDbgMs = 0;
        if (millis() - wgDbgMs > 250)
        {
            wgDbgMs = millis();
            Serial.printf("[wg] st=%d ms=%d act=%.1f pend=%.1f tgt=%.1fN Tq=%.3f v_cmd=%.2f d=%.2f drop=%.2f con=%.2f fr=%.2f pos=%.3f v=%.2f mode=%d span=[%.3f..%.3f]\n",
                          (int)wgState, (int)g_machineState, wgActive, wgPending,
                          sharedCfgData.target_force, g_cmdMaxTorqueNm, g_cmdVelocity, dHome,
                          g_dropFrac, g_conFactor, g_fricS, controlFeedback.m1_position, velocity,
                          (int)controlState.mode, g_posSpanMin, g_posSpanMax);
        }
    }
}

// Telemetry replies (1 Hz query): bus voltage, Q-phase current, board temp,
// mode and fault. Mirrors the old ODrive GET_VBUS handling for the screen,
// and adds fault/timeout recovery: stopped mode clears a latched fault, and
// the next streamed frame re-enters position control.
static void processMoteusTelemetry(const MoteusReply &r)
{
    if (!isnan(r.voltage))
    {
        controlFeedback.voltage = r.voltage;
        controlFeedback.current = isnan(r.q_current) ? 0.0f : r.q_current;
        if (xSemaphoreTake(mutex, 0) == pdTRUE)
        {
            sharedStateData.voltage = r.voltage;
            sharedStateData.current = controlFeedback.current;
            sharedStateData.newData = true;
            xSemaphoreGive(mutex);
        }
        else
        {
            handleError(ERROR_I2C_RECEIVE, "Failed to take mutex for processing CAN telemetry");
        }
    }
    if (r.mode >= 0)
    {
        controlState.error_state = (uint32_t)r.fault;
        if (r.mode == MOTEUS_MODE_FAULT || r.mode == MOTEUS_MODE_TIMEOUT)
        {
            static unsigned long lastRecoverMs = 0;
            if (millis() - lastRecoverMs > 1000)
            {
                lastRecoverMs = millis();
                logMessagef(LOG_WARN, "moteus in mode %d (fault=%d) — sending stop to clear",
                            r.mode, r.fault);
                sendMoteusStop(MOTEUS_ID_M1);
            }
        }
        if (g_wgDebug && r.fault != 0)
        {
            logMessagef(LOG_DEBUG, "[wg] moteus mode=%d fault=%d vbus=%.1fV iq=%.1fA temp=%.0fC",
                        r.mode, r.fault, r.voltage, r.q_current, r.temperature);
        }
    }
}

void initMotorControl()
{
    // Single-core C5: no core pinning. Priority 3 keeps the 500 Hz control
    // tick ahead of the WiFi tasks (WS p2 / HTTP p1), which the dual-core
    // S3 handled by putting them on separate cores.
    xTaskCreate(
        MotorControlTask,   // Task function
        "MotorControlTask", // Name of the task
        10000,              // Stack size in words
        NULL,               // Task input parameter
        3,                  // Priority of the task
        NULL);              // Task handle
}

void MotorControlTask(void *parameter)
{
    static uint8_t lastAppliedMode = 255; // Track last mode to avoid redundant CAN sends

    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    int64_t prev_time_us = time_us;
    int64_t runInterval_us = 2000; // 500Hz loop time.

    // Known state at boot: stopped (freewheel) — this also clears any fault
    // latched on the moteus from a previous run. The boot machine state is
    // MS_SLEEP; the 50 Hz idle query stream below watches for wake movement.
    sendMoteusStop(MOTEUS_ID_M1);

    // init force modulation params
    updateForceData("constant", 100, 30, 0.5, 2); // off, constant, linear
    updateDetentData("off", 50, .2, .3, 10);
    updatePulseData("off", 0, 100, 16);
    updateRowData("off", 20, 3, 0);

    uint32_t tick = 0;

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
        tick++;

        // Drain command queue from WS/HTTP without blocking (before timing gate)
        // Coalesce to the latest requested state to avoid rapid back-to-back toggles causing traffic bursts
        CommandMsg msg;
        CommandMsg desired = {};
        CommandType desiredType = CMD_NONE;
        unsigned long desiredCmdMs = 0;
        while (g_cmdQueue && xQueueReceive(g_cmdQueue, &msg, 0) == pdTRUE) {
            desired = msg;
            desiredType = msg.type;
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
                    // only latches after a confirming pull. A weight command
                    // also exits row mode — the two never stack.
                    g_rowEnabled = false;
                    wgCommand(desired.weight_lb);
                    g_machineState = (desired.weight_lb > 0.0f) ? MS_ACTIVE : MS_RECOIL;
                    // Enter streaming right away — the first frame carries
                    // the gate's current output (~zero row while armed), so
                    // the motor engages holding essentially no force and the
                    // ramp adds it with travel.
                    if (controlState.mode != 1) {
                        controlState.mode = 1;
                        pendingApplyStrength = true;
                    }
                } else if (desiredType == CMD_IDLE) {
                    // Weight 0 = RECOIL: gentle ~1 lbf self-stow at low
                    // velocity (auto-sleep stops the axis once parked).
                    // Idle is the off switch for row mode too.
                    g_rowEnabled = false;
                    wgIdle();
                    sharedCfgData.target_force = 0.0f;
                    g_machineState = MS_RECOIL;
                    if (controlState.mode != 1) {
                        controlState.mode = 1;
                        pendingApplyStrength = true;
                    }
                } else if (desiredType == CMD_ROW) {
                    if (desired.row_gear >= 1 && desired.row_gear <= 10) {
                        g_rowGear = desired.row_gear;
                    }
                    if (desired.row_drag >= 1 && desired.row_drag <= 10) {
                        g_rowDrag = desired.row_drag;
                    }
                    if (desired.row_enable == 1 && !g_rowEnabled) {
                        // SAFETY: entering row clears any latched weight —
                        // row force and strength force never stack. The
                        // wheel starts stopped, so the first catch renders
                        // from the return-tension floor.
                        wgIdle();
                        sharedCfgData.target_force = 0.0f;
                        rowReset();
                        g_rowEnabled = true;
                        g_machineState = MS_ACTIVE;
                        updateModeData("row");
                        if (controlState.mode != 1) {
                            controlState.mode = 1;
                            pendingApplyStrength = true;
                        }
                    } else if (desired.row_enable == 0 && g_rowEnabled) {
                        g_rowEnabled = false;
                        wgIdle();
                        g_machineState = MS_RECOIL; // self-stow, then auto-sleep
                        updateModeData("off");
                        if (controlState.mode != 1) {
                            controlState.mode = 1;
                            pendingApplyStrength = true;
                        }
                    }
                }
                xSemaphoreGive(mutex);
            }
            if (!applied)
            {
                // Mutex contention: DO NOT lose the command — push it back
                // to the front of the queue and retry next control cycle
                if (g_cmdQueue)
                {
                    xQueueSendToFront(g_cmdQueue, &desired, 0);
                }
            }
            sharedData.t_cmd_set_ms = desiredCmdMs;
        }

        // Handle any pending mode changes requested by other threads (e.g., WiFi) here
        // to centralize CAN traffic and avoid cross-task contention.
        if (xSemaphoreTake(mutex, 0) == pdTRUE)
        {
            if (pendingApplyStrength)
            {
                if (lastAppliedMode != 1)
                {
                    // Nothing to pre-stage on the moteus: the first streamed
                    // frame below carries mode 10 + the gate's current torque
                    // cap together. Boot defaults are the recoil-safe profile.
                    lastAppliedMode = 1;
                }
                pendingApplyStrength = false;
                unsigned long applyMs = millis();
                sharedData.t_ctrl_apply_ms = applyMs;
                sharedData.mode_latency_ms = applyMs - sharedData.t_cmd_set_ms;
            }
            if (pendingApplyIdle)
            {
                if (lastAppliedMode != 0)
                {
                    // Stopped = freewheel + fault clear; the strap stays
                    // parked by the dock, watched by the idle query stream.
                    sendMoteusStop(MOTEUS_ID_M1);
                    lastAppliedMode = 0;
                }
                pendingApplyIdle = false;
                unsigned long applyMs = millis();
                sharedData.t_ctrl_apply_ms = applyMs;
                sharedData.mode_latency_ms = applyMs - sharedData.t_cmd_set_ms;
            }
            xSemaphoreGive(mutex);
        }

        processI2CData();

        // Drain every reply that arrived since the last tick; feedback
        // replies run the full force pipeline, telemetry replies refresh
        // the screen data and watch for faults.
        MoteusReply reply;
        while (receiveMoteusReply(reply))
        {
            if (reply.has_telemetry)
            {
                processMoteusTelemetry(reply);
            }
            if (reply.has_feedback)
            {
                processMoteusFeedback(reply.position, reply.velocity);
            }
        }

        // One frame per tick while streaming; a 50 Hz query-only stream in
        // idle keeps position feedback alive for wake detection without
        // energizing the axis. 1 Hz telemetry rides alongside either way.
        if (controlState.mode == 1)
        {
            sendMoteusPositionCommand(MOTEUS_ID_M1, g_cmdVelocity, g_cmdMaxTorqueNm, true);
        }
        else if ((tick % 10) == 0)
        {
            sendMoteusQuery(MOTEUS_ID_M1);
        }
        if ((tick % 500) == 0)
        {
            sendMoteusTelemetryQuery(MOTEUS_ID_M1);
        }

        // Minimal yield at end of cycle; main yield is in timing loop above
        taskYIELD();
    }
}

// Force-vs-position profile. dHome = revs of pull from the dock (stroke on
// this drivetrain ~5 revs). "constant" scales the set weight; "linear"
// ramps from start_strength% at start_position to strength% at
// saturation_position — strength above 100 is allowed, which is how the
// "resistance band" recipe overloads the top of the stroke; "chains" is the
// same profile starting partial and finishing at 100.
float applyForceControl(float target_force, float dHome)
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
        if (dHome <= forceData.start_position)
        {
            force = (forceData.start_strength / 100.0) * target_force;
        }
        else if (dHome >= forceData.saturation_position)
        {
            force = (forceData.strength / 100.0) * target_force;
        }
        else
        {
            float scale = (dHome - forceData.start_position) /
                          (forceData.saturation_position - forceData.start_position);
            float strength = forceData.start_strength + scale * (forceData.strength - forceData.start_strength);
            force = (strength / 100.0) * target_force;
        }
    }
    interrupts(); // End critical section
    return force;
}

// Detents: narrow raised-cosine force DIPS at every step_position revs past
// start_position — pulling through them clicks like a mechanical detent.
// strength = dip depth in percent of the current force; total_steps bounds
// how many notches exist. (The pre-moteus version was a flat reduction over
// the whole range and was disconnected from the pipeline.)
float applyDetentControl(float target_force, float dHome)
{
    float force = target_force;
    noInterrupts(); // Begin critical section
    if (detentData.type == "on" && detentData.step_position > 0.01f)
    {
        float rel = dHome - detentData.start_position;
        if (rel > 0.0f)
        {
            float stepF = rel / detentData.step_position;
            int step = (int)stepF;
            if (step < detentData.total_steps)
            {
                float phase = stepF - (float)step;                        // 0..1 within the interval
                float d = (phase < 0.5f) ? phase : (1.0f - phase);        // distance to nearest notch
                const float width = 0.25f;                                // notch width, fraction of interval
                if (d < width)
                {
                    float bump = 0.5f * (1.0f + cosf(PI * d / width));    // 1 at notch center -> 0 at edge
                    force = target_force * (1.0f - (detentData.strength / 100.0f) * bump);
                }
            }
        }
    }
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

// The old timer-ramped row modulation (applyRowModeControl) is gone: row is
// now its own mode with the flywheel model above, not a strength modifier.
