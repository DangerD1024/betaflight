/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <math.h>

#include "platform.h"

#include "build/debug.h"
#include "common/axis.h"
#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/time.h"

#include "fc/rc.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"

#include "flight/target_attitude.h"

// Drop the external (VOT_C) setpoint if nothing arrived within this window; the
// app streams at ~30 ms. When stale, the mode falls back to STANDALONE MANUAL
// (the pilot flies via the sticks) -- it does NOT need the app to be running.
// 350 ms (was 200): flight 2026-07-02 13:32 showed the FC flip-flopping to manual
// mid-chase from MSP delivery hiccups while VOT_C was streaming every frame --
// the wider window bridges link stalls; a real app death still falls back <0.4 s.
#define TARGET_STALE_US      350000

// All other tunables live in targetAttitudeConfig (CLI-settable: `set target_*`), so the
// rate law / autonomous loop can be tuned from real flights without a reflash. Defaults
// below match the original compile-time constants. Gains are stored x10.
//   AUTONOMOUS attitude P gain: body rate = 2 * (att_kp/10) * gain * q_err_vec.
//   MANUAL rate law: PITCH stick = body pitch rate (ACRO); the pan stick (ROLL or YAW,
//   `target_rotation`) = rotate about WORLD-DOWN (gravity) -> pure heading pan, no
//   bank/pitch change; the other of ROLL/YAW is ignored; gentle wings-level
//   (level_gain/10). No attitude setpoint -> no engage snap; no Euler yaw.
PG_REGISTER_WITH_RESET_TEMPLATE(targetAttitudeConfig_t, targetAttitudeConfig, PG_TARGET_ATTITUDE_CONFIG, 1);
PG_RESET_TEMPLATE(targetAttitudeConfig_t, targetAttitudeConfig,
    .att_kp        = 80,    // 8.0
    .max_accel     = 1500,  // deg/s^2
    .max_rate_rp   = 300,   // deg/s
    .max_rate_yaw  = 180,   // deg/s
    .level_gain    = 15,    // 1.5
    .rate_deadband = 6,     // deg/s
    .rotation      = TARGET_ROTATION_ROLL,
    .servo_correction = 0,   // us, off until the pilot sets it
    .servo_speed      = 500, // ms
    .servo_index      = 0,   // SERVO_GIMBAL_PITCH
);

// Set once if the retired 229 command is ever received; see the path marker below.
static bool retiredCmdSeen = false;

void targetAttitudeNoteRetiredCommand(void)
{
    retiredCmdSeen = true;
}

static quaternion qSp = QUATERNION_INITIALIZE;
static float targetGain = 0.0f;
static timeUs_t lastUpdateUs = 0;
static float rateSp[XYZ_AXIS_COUNT] = { 0.0f, 0.0f, 0.0f };

// accel-limit (slew) state, applied to rateSp once per loop on both paths
static float rateSpPrev[XYZ_AXIS_COUNT] = { 0.0f, 0.0f, 0.0f };
static timeUs_t slewLastUs = 0;

// The last MSP_SET_TARGET_CORRECTION command as received, in the wire's own
// fixed-point scale (q x16384, gain x10000), for blackbox logging. Kept separate
// from qSp/targetGain because those hold the COMPOSED absolute setpoint and the
// clamped gain, not the command the app sent.
static int16_t lastCmdQ[4] = { 0, 0, 0, 0 };
static int16_t lastCmdGain = 0;

void targetAttitudeInit(void)
{
    qSp.w = 1.0f;
    qSp.x = qSp.y = qSp.z = 0.0f;
    targetGain = 0.0f;
    lastUpdateUs = 0;
    rateSp[FD_ROLL] = rateSp[FD_PITCH] = rateSp[FD_YAW] = 0.0f;
    rateSpPrev[FD_ROLL] = rateSpPrev[FD_PITCH] = rateSpPrev[FD_YAW] = 0.0f;
    slewLastUs = 0;
}

void targetAttitudeSet(float w, float x, float y, float z, float gain)
{
    // Defensive normalize: the wire format is fixed-point, so the quaternion
    // arrives slightly off unit length.
    const float n2 = w * w + x * x + y * y + z * z;
    if (n2 < 1e-9f) {
        return; // ignore degenerate payloads, keep last good setpoint
    }
    const float inv = 1.0f / sqrtf(n2);
    qSp.w = w * inv;
    qSp.x = x * inv;
    qSp.y = y * inv;
    qSp.z = z * inv;
    targetGain = constrainf(gain, 0.0f, 1.0f);

    // Cooperative scheduler: this (MSP task) never interleaves with
    // targetAttitudeUpdate() (gyro/PID task), so no locking is required.
    lastUpdateUs = micros();
}

void targetAttitudeSetCorrection(float w, float x, float y, float z, float gain)
{
    // Defensive normalize (fixed-point wire format), as in targetAttitudeSet().
    const float n2 = w * w + x * x + y * y + z * z;
    if (n2 < 1e-9f) {
        return; // ignore degenerate payloads, keep last good setpoint
    }
    const float inv = 1.0f / sqrtf(n2);
    w *= inv; x *= inv; y *= inv; z *= inv;

    // Record the command as received (wire fixed-point scale) for the blackbox, so
    // a log shows exactly what the app commanded even when the composed setpoint or
    // the clamps downstream make the FC's response look unrelated to it.
    lastCmdQ[0] = (int16_t)lrintf(w * 16384.0f);
    lastCmdQ[1] = (int16_t)lrintf(x * 16384.0f);
    lastCmdQ[2] = (int16_t)lrintf(y * 16384.0f);
    lastCmdQ[3] = (int16_t)lrintf(z * 16384.0f);
    lastCmdGain = (int16_t)lrintf(constrainf(gain, 0.0f, 1.0f) * 10000.0f);

    // Compose the absolute setpoint against OUR current attitude: q_sp = q_cur (x) q_corr
    // (Hamilton product, q_corr in body axes). Between updates the stored q_sp still
    // holds an absolute attitude, so the hold-on-coast behavior matches the legacy
    // absolute-setpoint path. Cooperative scheduler: the MSP task never interleaves
    // with targetAttitudeUpdate() (gyro/PID task), so reading getQuaternion() and
    // writing qSp here needs no locking.
    quaternion qCur;
    getQuaternion(&qCur);
    qSp.w = qCur.w * w - qCur.x * x - qCur.y * y - qCur.z * z;
    qSp.x = qCur.w * x + qCur.x * w + qCur.y * z - qCur.z * y;
    qSp.y = qCur.w * y - qCur.x * z + qCur.y * w + qCur.z * x;
    qSp.z = qCur.w * z + qCur.x * y - qCur.y * x + qCur.z * w;

    targetGain = constrainf(gain, 0.0f, 1.0f);
    lastUpdateUs = micros();
}

bool targetAttitudeIsFresh(void)
{
    if (lastUpdateUs == 0) {
        return false;
    }
    return cmpTimeUs(micros(), lastUpdateUs) < (timeDelta_t)TARGET_STALE_US;
}

void targetAttitudeGetLastCommand(int16_t out[5])
{
    out[0] = lastCmdQ[0];
    out[1] = lastCmdQ[1];
    out[2] = lastCmdQ[2];
    out[3] = lastCmdQ[3];
    out[4] = lastCmdGain;
}

uint8_t targetAttitudeBlackboxState(void)
{
    uint8_t state = 0;
    if (FLIGHT_MODE(TARGET_MODE)) {
        state |= 1 << 0;
        if (targetAttitudeIsFresh()) {
            state |= 1 << 1;   // autonomous: following the streamed 231 setpoint
        }
    }
    if (IS_RC_MODE_ACTIVE(BOXMSPOVERRIDE)) {
        state |= 1 << 2;
    }
    return state;
}

// AUTONOMOUS path: q_err = conj(qCur) (x) qSp ; body-rate setpoint = 2*Kp*gain*vec(q_err),
// clamped. Used when VOT_C is streaming an attitude setpoint (the tracker).
static void targetComputeRate(const quaternion *qCur, const quaternion *qsp, float gain)
{
    const float cw =  qCur->w, cx = -qCur->x, cy = -qCur->y, cz = -qCur->z;
    const float sw = qsp->w, sx = qsp->x, sy = qsp->y, sz = qsp->z;

    float ew = cw * sw - cx * sx - cy * sy - cz * sz;
    float ex = cw * sx + cx * sw + cy * sz - cz * sy;
    float ey = cw * sy - cx * sz + cy * sw + cz * sx;
    float ez = cw * sz + cx * sy - cy * sx + cz * sw;

    if (ew < 0.0f) { ex = -ex; ey = -ey; ez = -ez; }   // shortest path

    const float maxRP = targetAttitudeConfig()->max_rate_rp, maxYaw = targetAttitudeConfig()->max_rate_yaw;
    const float k = 2.0f * (targetAttitudeConfig()->att_kp * 0.1f) * gain;
    rateSp[FD_ROLL]  = constrainf(RADIANS_TO_DEGREES(k * ex), -maxRP,  maxRP);
    rateSp[FD_PITCH] = constrainf(RADIANS_TO_DEGREES(k * ey), -maxRP,  maxRP);
    rateSp[FD_YAW]   = constrainf(RADIANS_TO_DEGREES(k * ez), -maxYaw, maxYaw);

    DEBUG_SET(DEBUG_ANGLE_TARGET, 3, lrintf(gain * 1000.0f));  // gain x1000 (== rc_log fc_tgt_gain)
    DEBUG_SET(DEBUG_ANGLE_TARGET, 4, 0);
    // Slot 5 belongs to the FC clock (written once per loop in targetAttitudeUpdate),
    // NOT to this path: it aligns the Pi's rc_log with the blackbox timeline.
    DEBUG_SET(DEBUG_ANGLE_TARGET, 7, 1000);                    // path marker: autonomous
}

// STANDALONE MANUAL rate law (see the header comment block above). Writes raw body-rate
// setpoints; targetAttitudeUpdate() then accel-limits them.
static void targetComputeManualRate(const quaternion *qCur)
{
    // world-down (gravity) expressed in the body frame == row 2 of the body->earth rMat.
    const float w = qCur->w, x = qCur->x, y = qCur->y, z = qCur->z;
    const float gx = 2.0f * (x * z - w * y);          // rMat[2][0]
    const float gy = 2.0f * (y * z + w * x);          // rMat[2][1]
    const float gz = 1.0f - 2.0f * (x * x + y * y);   // rMat[2][2]

    // CLI-settable tunables (see targetAttitudeConfig: `set target_*`).
    const float deadband  = targetAttitudeConfig()->rate_deadband;
    const float levelGain = targetAttitudeConfig()->level_gain * 0.1f;
    const float maxRP = targetAttitudeConfig()->max_rate_rp, maxYaw = targetAttitudeConfig()->max_rate_yaw;
    // `target_rotation` picks which stick pans the heading; the other is ignored.
    const int panAxis = (targetAttitudeConfig()->rotation == TARGET_ROTATION_YAW) ? FD_YAW : FD_ROLL;

    // pilot sticks -> rate (deg/s), deadbanded so a resting (~1501) stick truly holds.
    float pitchStick = getSetpointRate(FD_PITCH);
    float panStick   = getSetpointRate(panAxis);
    if (fabsf(pitchStick) < deadband) pitchStick = 0.0f;
    if (fabsf(panStick)   < deadband) panStick   = 0.0f;
    // pan stick RIGHT must rotate the craft RIGHT (CW seen from above). On the real
    // airframe the world-down pan came out mirrored, so the sign is negated here.
    // NOTE: the AUTONOMOUS/VOT_C path (targetComputeRate, q_err) does NOT use this,
    // so VOT_C control is unaffected by this sign.
    const float pan = -panStick;  // deg/s rotation about world-down (stick-right = right)

    // gentle wings-level: drive gravity's body-Y component (gy ~ sin bank) -> 0 about the
    // axis that changes bank with the LEAST heading change (g x ybody = (-gz,0,gx)), so it
    // does not fight the pan. Fades out toward 90deg bank (denominator). The slew below
    // ramps its onset, so engaging while banked eases level instead of jerking.
    float lvlRoll = 0.0f, lvlYaw = 0.0f;
    const float lvlDen = gx * gx + gz * gz;        // = 1 - gy^2 ; ->0 only at 90deg bank
    if (levelGain > 0.0f && lvlDen > 0.02f) {
        const float bankErrDeg = RADIANS_TO_DEGREES(asinf(constrainf(gy, -1.0f, 1.0f)));
        const float c = levelGain * bankErrDeg / lvlDen;
        lvlRoll = -c * gz;   // body-X component
        lvlYaw  =  c * gx;   // body-Z component
    }

    // body-rate command: pan about gravity (pan*g) + pitch on body-Y + wings-level.
    // The non-pan stick of ROLL/YAW contributes nothing.
    const float rRoll  = pan * gx + lvlRoll;
    const float rPitch = pan * gy + pitchStick;
    const float rYaw   = pan * gz + lvlYaw;

    rateSp[FD_ROLL]  = constrainf(rRoll,  -maxRP,  maxRP);
    rateSp[FD_PITCH] = constrainf(rPitch, -maxRP,  maxRP);
    rateSp[FD_YAW]   = constrainf(rYaw,   -maxYaw, maxYaw);

    DEBUG_SET(DEBUG_ANGLE_TARGET, 3, 1000);                      // "gain"=full (manual); == rc_log fc_tgt_gain
    DEBUG_SET(DEBUG_ANGLE_TARGET, 4, lrintf(pan));              // pan rate
    // Slot 5 used to carry the wings-level term (lvlRoll + lvlYaw). It now belongs to
    // the FC clock (written once per loop in targetAttitudeUpdate) — aligning the Pi's
    // rc_log with the blackbox proved worth more than this tuning visibility: the
    // 22 s manual-fallback event could not even be SEGMENTED between the two logs.
    // Path marker -- which manual-fallback story this is:
    //   2000  a setpoint DID arrive and has since gone stale (dropout, track released)
    //   2100  none has EVER arrived: nothing is talking to us on any port
    //   2200  none has arrived, but the RETIRED 229 command HAS been received, i.e.
    //         something IS talking and is speaking the dialect we no longer answer.
    //         That is version skew -- an old app against this firmware -- and it is
    //         otherwise indistinguishable from 2100 while having a completely
    //         different fix. Both 2026-09-07 bench runs were really this case.
    int16_t marker = 2000;
    if (lastUpdateUs == 0) {
        marker = retiredCmdSeen ? 2200 : 2100;
    }
    DEBUG_SET(DEBUG_ANGLE_TARGET, 7, marker);
}

// NOINLINE: this is called once per PID loop (not per gyro sample). Keeping it out
// of line stops LTO from pulling its body into the FAST_CODE pidController(), which
// otherwise overflows the small ITCM on F7 targets (e.g. SPEDIXF722).
NOINLINE void targetAttitudeUpdate(timeUs_t currentTimeUs)
{
    if (!FLIGHT_MODE(TARGET_MODE)) {
        slewLastUs = 0;
        rateSp[FD_ROLL] = rateSp[FD_PITCH] = rateSp[FD_YAW] = 0.0f;
        rateSpPrev[FD_ROLL] = rateSpPrev[FD_PITCH] = rateSpPrev[FD_YAW] = 0.0f;
        return;
    }

    quaternion qCur;
    getQuaternion(&qCur);

    if (targetAttitudeIsFresh()) {
        // External setpoint streamed by VOT_C (autonomous tracker): follow it.
        targetComputeRate(&qCur, &qSp, targetGain);
    } else {
        // STANDALONE MANUAL: the pilot flies via the sticks (ACRO-like rate law).
        targetComputeManualRate(&qCur);
    }

    const float preRoll = rateSp[FD_ROLL];  // pre-slew, for tuning visibility

    // accel-limit (slew) the commanded rates -- both paths. maxStep = 0 on the first loop
    // after engage (slewLastUs reset) -> output ramps up from rateSpPrev=0 (no snap); a
    // step in the setpoint is bounded to TARGET_MAX_ACCEL*dt per loop.
    float dt = (slewLastUs == 0) ? 0.0f : cmpTimeUs(currentTimeUs, slewLastUs) * 1e-6f;
    slewLastUs = currentTimeUs;
    dt = constrainf(dt, 0.0f, 0.05f);
    const uint16_t maxAccel = targetAttitudeConfig()->max_accel;
    if (maxAccel > 0) {
        const float maxStep = maxAccel * dt;
        for (int a = FD_ROLL; a <= FD_YAW; a++) {
            rateSp[a] = rateSpPrev[a] + constrainf(rateSp[a] - rateSpPrev[a], -maxStep, maxStep);
        }
    }
    rateSpPrev[FD_ROLL]  = rateSp[FD_ROLL];
    rateSpPrev[FD_PITCH] = rateSp[FD_PITCH];
    rateSpPrev[FD_YAW]   = rateSp[FD_YAW];

    DEBUG_SET(DEBUG_ANGLE_TARGET, 0, lrintf(rateSp[FD_ROLL]));   // commanded rate roll (post-slew)
    DEBUG_SET(DEBUG_ANGLE_TARGET, 1, lrintf(rateSp[FD_PITCH]));  // commanded rate pitch
    DEBUG_SET(DEBUG_ANGLE_TARGET, 2, lrintf(rateSp[FD_YAW]));    // commanded rate yaw
    // FC clock, for both paths: millis()/8 in the low 16 bits (8 ms tick, ~6 days
    // before wrap). The Pi reads it over MSP_DEBUG and x8s it back into milliseconds
    // (rc_log column fc_ms) — the only anchor between the Pi's clock and the FC's,
    // which blackbox_decode cannot provide (the two processes never shared a
    // timebase). Sent as int16, so values above 32767 arrive negative on the wire;
    // the Pi masks back to uint16 before scaling.
    DEBUG_SET(DEBUG_ANGLE_TARGET, 5, (millis() >> 3) & 0xFFFF);
    DEBUG_SET(DEBUG_ANGLE_TARGET, 6, lrintf(preRoll));           // pre-slew roll (see slew effect)
}

float targetAttitudeRateSetpoint(int axis)
{
    return rateSp[axis];
}

// Camera-servo nudge state. The offset eases in over target_servo_speed instead of
// stepping, so engaging targeting doesn't snap the gimbal (and with it the image, and on
// a heavy camera the airframe). Symmetric on the way out.
static float servoOffset = 0.0f;
static timeUs_t servoOffsetLastUs = 0;

// Hold-off before the gimbal is allowed to move at all.
//
// Two seconds from the moment BOTH conditions hold -- armed AND targeting engaged -- not
// from either one alone. So enabling TARGET before arming starts the clock at the arm
// instant, and arming first then enabling TARGET starts it at the enable instant; either
// way the servo stays put for two seconds after the last of the two arrives.
//
// Dropping either condition clears the latch, so a momentary TARGET_MODE flicker costs
// another full two seconds rather than resuming part-way. That is deliberate: the point
// of the delay is to let the airframe settle before the camera moves, and a part-elapsed
// timer would defeat it.
#define TARGET_SERVO_ARM_DELAY_US 2000000

static bool servoDelayLatched = false;
static timeUs_t servoArmEngageUs = 0;

// NOINLINE for the same reason as targetAttitudeUpdate above: the caller chain
// (writeServos <- subTaskMotorUpdate) ends in FAST_CODE, so without this LTO pulls the
// body into ITCM and overflows the F7's 16 KB. It runs once per loop, so the call
// overhead is irrelevant.
NOINLINE int16_t targetServoOffsetUpdate(void)
{
    const int16_t correction = targetAttitudeConfig()->servo_correction;
    // Both switches, matching how the pilot hands the interceptor to VOT_C. MSP OVERRIDE
    // has no FLIGHT_MODE bit, so it has to be read as an RC mode.
    //
    // Now ALSO gated on ARMED. This used to be deliberately ungated -- the old comment
    // said so explicitly, "No arming gate ... which also lets this be checked on the
    // bench" -- and the cost of that was that a bench or pre-arm bump of the TARGET
    // switch moved the gimbal, i.e. moved the camera the seeker looks through while
    // nothing was flying. Arming on the bench with the props off still exercises it;
    // you just have to wait the two seconds out.
    const bool engaged = ARMING_FLAG(ARMED)
                      && FLIGHT_MODE(TARGET_MODE)
                      && IS_RC_MODE_ACTIVE(BOXMSPOVERRIDE);

    const timeUs_t nowUs = micros();

    // Latch the instant both conditions first hold; clear the latch the moment either
    // drops so a re-engage waits the full delay again.
    if (!engaged) {
        servoDelayLatched = false;
    } else if (!servoDelayLatched) {
        servoDelayLatched = true;
        servoArmEngageUs = nowUs;
    }
    const bool delayElapsed = servoDelayLatched
                           && (cmpTimeUs(nowUs, servoArmEngageUs) >= TARGET_SERVO_ARM_DELAY_US);

    // Until the hold-off expires the goal stays 0, and because the ramp below is
    // symmetric the gimbal eases in over target_servo_speed from the moment the delay
    // ends rather than stepping. Disengaging is unchanged: it eases back to 0 with no
    // delay, since returning the camera to centre is the safe direction and should not
    // be held off.
    const float goal = (delayElapsed && correction != 0) ? (float)correction : 0.0f;

    // dt = 0 on the first call (no jump from a stale timestamp); clamped so a scheduler
    // stall can't turn into one big step either.
    float dt = (servoOffsetLastUs == 0) ? 0.0f : cmpTimeUs(nowUs, servoOffsetLastUs) * 1e-6f;
    servoOffsetLastUs = nowUs;
    dt = constrainf(dt, 0.0f, 0.05f);

    const uint16_t rampMs = targetAttitudeConfig()->servo_speed;
    if (rampMs == 0 || correction == 0) {
        servoOffset = goal;
    } else {
        // the FULL correction spans rampMs, so the rate doesn't depend on where we are now
        const float step = (ABS(correction) * 1000.0f / rampMs) * dt;
        servoOffset += constrainf(goal - servoOffset, -step, step);
    }

    return lrintf(servoOffset);
}
