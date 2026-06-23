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
#include "fc/runtime_config.h"

#include "flight/imu.h"

#include "flight/target_attitude.h"

// Drop the external (VOT_C) setpoint if nothing arrived within this window; the
// app streams at ~30 ms. When stale, the mode falls back to STANDALONE MANUAL
// (the pilot flies via the sticks) -- it does NOT need the app to be running.
#define TARGET_STALE_US      200000

// Attitude proportional gain (1/s): body rate = 2 * Kp * gain * q_err_vec.
#define TARGET_ATT_KP        8.0f

// Body-rate clamps (deg/s). Conservative for a 300 km/h / ~80deg platform; the
// inner rate PID still tracks these exactly as it does stick-commanded rates.
#define TARGET_MAX_RATE_RP   300.0f
#define TARGET_MAX_RATE_YAW  180.0f

// ---- standalone MANUAL mode (no external setpoint: pilot flies via the sticks) ----
#define TARGET_MANUAL_GAIN        1.0f    // attitude gain for the manual path
#define TARGET_MANUAL_LEAD_DEG    40.0f   // max setpoint lead vs actual (caps rate + soft settle)
#define TARGET_MANUAL_PITCH_SIGN  (+1.0f) // positive pitch stick -> NOSE DOWN (pitch decreases)
#define TARGET_MANUAL_YAW_SIGN    (+1.0f) // positive ROLL stick -> heading right (pan)
#define TARGET_MANUAL_PITCH_MIN   0.0f    // stay upright: never past vertical / inverted
#define TARGET_MANUAL_PITCH_MAX   88.0f

static quaternion qSp = QUATERNION_INITIALIZE;
static float targetGain = 0.0f;
static timeUs_t lastUpdateUs = 0;
static float rateSp[XYZ_AXIS_COUNT] = { 0.0f, 0.0f, 0.0f };

// manual-mode integrated attitude setpoint (deg, BF convention) + timing
static float manPitch = 0.0f, manYaw = 0.0f;
static bool manInit = false;
static timeUs_t manLastUs = 0;

void targetAttitudeInit(void)
{
    qSp.w = 1.0f;
    qSp.x = qSp.y = qSp.z = 0.0f;
    targetGain = 0.0f;
    lastUpdateUs = 0;
    rateSp[FD_ROLL] = rateSp[FD_PITCH] = rateSp[FD_YAW] = 0.0f;
    manInit = false;
    manLastUs = 0;
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

bool targetAttitudeIsFresh(void)
{
    if (lastUpdateUs == 0) {
        return false;
    }
    return cmpTimeUs(micros(), lastUpdateUs) < (timeDelta_t)TARGET_STALE_US;
}

// Euler (deg, Betaflight convention with -yaw half-angle) -> quaternion, mirroring
// imu.c::imuComputeQuaternionFromRPY. Used to build the level-bank manual setpoint.
static void quatFromEulerBF(float rollDeg, float pitchDeg, float yawDeg, quaternion *q)
{
    const float r = DEGREES_TO_RADIANS(rollDeg) * 0.5f;
    const float p = DEGREES_TO_RADIANS(pitchDeg) * 0.5f;
    const float y = DEGREES_TO_RADIANS(-yawDeg) * 0.5f;
    const float cr = cos_approx(r), sr = sin_approx(r);
    const float cp = cos_approx(p), sp = sin_approx(p);
    const float cy = cos_approx(y), sy = sin_approx(y);
    q->w = cr * cp * cy + sr * sp * sy;
    q->x = sr * cp * cy - cr * sp * sy;
    q->y = cr * sp * cy + sr * cp * sy;
    q->z = cr * cp * sy - sr * sp * cy;
}

// Quaternion -> Euler (deg, Betaflight convention), computed directly from q so it
// is the EXACT inverse of quatFromEulerBF (independent of the SITL rMat override,
// which only affects the attitude.values used for display/MSP).
static void bfEulerFromQuat(const quaternion *q, float *rollDeg, float *pitchDeg, float *yawDeg)
{
    const float w = q->w, x = q->x, y = q->y, z = q->z;
    const float r20 = 2.0f * (x * z - w * y);          // rMat[2][0]
    const float r21 = 2.0f * (y * z + w * x);          // rMat[2][1]
    const float r22 = 1.0f - 2.0f * (x * x + y * y);   // rMat[2][2]
    const float r10 = 2.0f * (x * y + w * z);          // rMat[1][0]
    const float r00 = 1.0f - 2.0f * (y * y + z * z);   // rMat[0][0]
    float s = -r20;
    s = constrainf(s, -1.0f, 1.0f);
    *rollDeg  = RADIANS_TO_DEGREES(atan2_approx(r21, r22));
    *pitchDeg = RADIANS_TO_DEGREES(asinf(s));
    *yawDeg   = RADIANS_TO_DEGREES(-atan2_approx(r10, r00));
}

// q_err = conj(qCur) (x) qSp ; body-rate setpoint = 2*Kp*gain*vec(q_err), clamped.
static void targetComputeRate(const quaternion *qCur, const quaternion *qsp, float gain)
{
    const float cw =  qCur->w, cx = -qCur->x, cy = -qCur->y, cz = -qCur->z;
    const float sw = qsp->w, sx = qsp->x, sy = qsp->y, sz = qsp->z;

    float ew = cw * sw - cx * sx - cy * sy - cz * sz;
    float ex = cw * sx + cx * sw + cy * sz - cz * sy;
    float ey = cw * sy - cx * sz + cy * sw + cz * sx;
    float ez = cw * sz + cx * sy - cy * sx + cz * sw;

    if (ew < 0.0f) { ex = -ex; ey = -ey; ez = -ez; }   // shortest path

    const float k = 2.0f * TARGET_ATT_KP * gain;
    rateSp[FD_ROLL]  = constrainf(RADIANS_TO_DEGREES(k * ex), -TARGET_MAX_RATE_RP,  TARGET_MAX_RATE_RP);
    rateSp[FD_PITCH] = constrainf(RADIANS_TO_DEGREES(k * ey), -TARGET_MAX_RATE_RP,  TARGET_MAX_RATE_RP);
    rateSp[FD_YAW]   = constrainf(RADIANS_TO_DEGREES(k * ez), -TARGET_MAX_RATE_YAW, TARGET_MAX_RATE_YAW);

    DEBUG_SET(DEBUG_ANGLE_TARGET, 0, lrintf(rateSp[FD_ROLL]));
    DEBUG_SET(DEBUG_ANGLE_TARGET, 1, lrintf(rateSp[FD_PITCH]));
    DEBUG_SET(DEBUG_ANGLE_TARGET, 2, lrintf(rateSp[FD_YAW]));
    DEBUG_SET(DEBUG_ANGLE_TARGET, 3, lrintf(gain * 1000.0f));
}

// Build the manual attitude setpoint from the pilot's sticks: heading and pitch
// are slewed at the FC's configured stick rates (getSetpointRate), bank forced
// level (roll = 0). Standalone -- no external app required.
static void targetBuildManualSetpoint(timeUs_t now, const quaternion *qCur, quaternion *qsp)
{
    float roll, pitch, yaw;
    bfEulerFromQuat(qCur, &roll, &pitch, &yaw);   // override-independent BF euler
    UNUSED(roll);

    float dt = (manLastUs == 0) ? 0.0f : cmpTimeUs(now, manLastUs) * 1e-6f;
    manLastUs = now;
    dt = constrainf(dt, 0.0f, 0.05f);

    if (!manInit) {
        manPitch = pitch;
        manYaw = yaw;
        manInit = true;
    }

    // pilot stick -> angular rate (deg/s) using the FC's own configured rate
    // profile. ROLL stick pans the heading (at ~80deg pitch a body-roll is a
    // world-yaw); PITCH stick changes pitch (positive stick = nose down).
    const float pitchStickRate = getSetpointRate(FD_PITCH);
    const float rollStickRate  = getSetpointRate(FD_ROLL);

    manPitch -= TARGET_MANUAL_PITCH_SIGN * pitchStickRate * dt;
    manYaw   += TARGET_MANUAL_YAW_SIGN   * rollStickRate  * dt;

    // anti-runaway: bound the setpoint lead vs the actual attitude so it can't
    // outrun the controller and settles fast when the stick is released.
    manPitch = constrainf(manPitch, pitch - TARGET_MANUAL_LEAD_DEG, pitch + TARGET_MANUAL_LEAD_DEG);
    float yawErr = manYaw - yaw;
    while (yawErr >  180.0f) yawErr -= 360.0f;
    while (yawErr < -180.0f) yawErr += 360.0f;
    manYaw = yaw + constrainf(yawErr, -TARGET_MANUAL_LEAD_DEG, TARGET_MANUAL_LEAD_DEG);

    // stay upright: never command past vertical or below the min flight pitch.
    manPitch = constrainf(manPitch, TARGET_MANUAL_PITCH_MIN, TARGET_MANUAL_PITCH_MAX);

    quatFromEulerBF(0.0f, manPitch, manYaw, qsp);   // roll = 0 -> wings level
}

// NOINLINE: this is called once per PID loop (not per gyro sample). Keeping it out
// of line stops LTO from pulling its body into the FAST_CODE pidController(), which
// otherwise overflows the small ITCM on F7 targets (e.g. SPEDIXF722).
NOINLINE void targetAttitudeUpdate(timeUs_t currentTimeUs)
{
    if (!FLIGHT_MODE(TARGET_MODE)) {
        manInit = false;
        manLastUs = 0;
        rateSp[FD_ROLL] = rateSp[FD_PITCH] = rateSp[FD_YAW] = 0.0f;
        return;
    }

    quaternion qCur;
    getQuaternion(&qCur);

    if (targetAttitudeIsFresh()) {
        // External setpoint streamed by VOT_C (autonomous tracker): follow it.
        manInit = false;
        targetComputeRate(&qCur, &qSp, targetGain);
    } else {
        // STANDALONE MANUAL: the pilot flies via the sticks (level bank, soft),
        // no external app needed -- e.g. targeting switch not in the tracking
        // position, or VOT_C not running.
        quaternion qspManual;
        targetBuildManualSetpoint(currentTimeUs, &qCur, &qspManual);
        targetComputeRate(&qCur, &qspManual, TARGET_MANUAL_GAIN);
    }
}

float targetAttitudeRateSetpoint(int axis)
{
    return rateSp[axis];
}
