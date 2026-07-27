#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// Declare controlFeedback structure
struct ControlFeedback
{
    float m1_position;
    float m1_velocity;
    float voltage;
    float current;
};

extern ControlFeedback controlFeedback; // Declare as extern to be used in other files

void initMotorControl();
void MotorControlTask(void *parameter);

// Live force calibration (RAM only — resets to compile-time defaults on
// boot; bake tuned values into MotorControl.cpp once found).
// Pass a negative value to leave a parameter unchanged.
//   lookupScale: global scale applied to the torque-matrix force lookup
//   zeroM1: zero-row torque magnitude — the felt floor at ~0 target force,
//           which now also provides the strap keep-taut tension
void setForceCal(float lookupScale, float zeroM1);
void getForceCal(float *lookupScale, float *zeroM1);

// Hot-edit a full torque-matrix row (live tuning; RAM only). values holds up
// to 6 columns: force, velocity_delta, -M1_Fn, -M1_Fm, M1_Fn, M1_Fm.
void setForceCalRow(int row, const float *values, int count);
// Toggle 4 Hz serial telemetry of the weight gate / torque pipeline.
void setForceCalDebug(bool on);
// Live-tune the weight gate's travel distances (position units; negative =
// leave unchanged): ramp length, latch hold, engage deadband, cancel backtrack.
void setWeightGateDistances(float deltaX, float deltaY, float deadband, float cancelRetract);
// +1 if position increases as the user pulls the strap out, -1 otherwise.
void setWeightGatePullSign(float sign);
// Calibration probe: stream a constant reel-in torque (|Nm| clamped to 3.0)
// for 10 seconds, overriding the force pipeline. 0 cancels immediately.
void setTestTorque(float torqueNm);
// Drive parameters (negative = leave unchanged): reel-in chase speed
// magnitude (rev/s) and the away-from-home velocity limit (rev/s). The
// commanded moteus velocity each cycle is -min(reelInVel, zone limit).
void setDriveParams(float reelInVel, float velLimit);
// Home-zone / recoil / auto-sleep behavior (negative = leave unchanged):
// zone: revs from home over which force tapers up and velocity is limited
// recoilTq: Nm rendered at 0 lb (strap self-stow) and the cap floor at home
// recoilVel: velocity limit (rev/s) at home / during recoil
// softMax: retained for API compatibility — moteus's per-command torque cap
//          is symmetric, so the push-toward-user bound now equals the
//          rendered force; the drop-catch shed keeps that safe (see .cpp)
// sleepS: seconds parked-in-zone + still before the axis is idled
// wakeThresh: strap movement (revs) that wakes the axis from sleep
void setHomeBehavior(float zone, float recoilTq, float recoilVel,
                     float softMax, float sleepS, float wakeThresh);
// Sleep eligibility distance (revs from home): the axis may only auto-sleep
// with the strap inside this — i.e. docked, nobody holding it. Much tighter
// than the force-taper zone on purpose.
void setSleepZone(float revs);
// Drop-catch (anti-runaway) tuning (negative = leave unchanged):
// vMin/vMax: reel-in rev/s where force shedding starts / hits the floor
// restoreDist: revs of outward travel to fully restore the shed force
void setDropCatch(float vMin, float vMax, float restoreDist);
// Directional friction compensation (negative = leave unchanged):
// k: comp fraction (command divided by 1+k*s; keep ~70% of measured
//    friction — overcompensation is negative damping). band: rev/s at
//    which the direction blend saturates.
void setFrictionComp(float k, float band);
// Concentric-only unloading (negative = leave unchanged):
// pct 0-100: how much of the force disappears when not actively pulling
// out (0 = normal, 100 = concentric-only; keep-taut/recoil floor remains)
// vLo/vHi: pull rev/s band over which the force blends back in
void setConcentric(float pct, float vLo, float vHi);
// Row-mode flywheel tuning (negative = leave unchanged), live via cal:
// kc: clutch coupling stiffness, Nm per rev/s of overspeed
// inertia: virtual flywheel inertia (bigger = longer glide)
// dragBase: quadratic decay coefficient per drag unit (damper 1-10 scales it)
// returnTq: strap tension during recovery, Nm
// maxNm: hard ceiling on rendered row force
// zone: pinch-only taper zone near the dock, revs (replaces the strength
//       home zone in row mode — a full-size zone would strangle the catch)
void setRowTuning(float kc, float inertia, float dragBase, float returnTq,
                  float maxNm, float zone);

#endif // MOTOR_CONTROL_H
