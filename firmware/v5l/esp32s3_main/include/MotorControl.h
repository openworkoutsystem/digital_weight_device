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
// Calibration probe: stream a constant reel-in torque (|Nm| clamped to 0.3)
// for 10 seconds, overriding the force pipeline. 0 cancels immediately.
void setTestTorque(float torqueNm);
// Drive parameters (negative = leave unchanged): torque->current conversion
// (A per Nm, fallback path only), the Set_Limits velocity limit, and the
// generous current ceiling streamed in endpoint mode.
void setDriveParams(float ampsPerNm, float velLimit, float maxCurrentA);
// Configure the torque_soft_min endpoint ID for this ODrive firmware and
// optionally switch force rendering to endpoint writes (useEpWrites 1/0;
// pass -1 to leave either unchanged).
void setSoftMinEndpoint(int endpointId, int useEpWrites);
// Home-zone / recoil / auto-sleep behavior (negative = leave unchanged):
// zone: turns from home over which force tapers up and velocity is limited
// recoilTq: Nm rendered at 0 lb (strap self-stow) and the cap floor at home
// recoilVel: vel_limit (turns/s) at home / during recoil
// softMax: constant push-toward-user torque bound (should stay ~1 lbf)
// sleepS: seconds parked-in-zone + still before the axis is idled
// wakeThresh: strap movement (turns) that wakes the axis from sleep
void setHomeBehavior(float zone, float recoilTq, float recoilVel,
                     float softMax, float sleepS, float wakeThresh);
// Drop-catch (anti-runaway) tuning (negative = leave unchanged):
// vMin/vMax: reel-in turns/s where force shedding starts / hits the floor
// restoreDist: turns of outward travel to fully restore the shed force
void setDropCatch(float vMin, float vMax, float restoreDist);

#endif // MOTOR_CONTROL_H
