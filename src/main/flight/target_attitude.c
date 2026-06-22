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

#include "flight/imu.h"

#include "flight/target_attitude.h"

// Drop the mode if no fresh setpoint within this window. The app streams at
// ~30 ms, so 200 ms tolerates ~6 dropped packets before the failsafe fires.
#define TARGET_STALE_US      200000

// Attitude proportional gain (1/s): body rate = 2 * Kp * gain * q_err_vec.
// The app's [0..1] gain scales this for the "gentle far / sharp near" behaviour,
// so keep Kp moderate here and let the app open it up as the target centres.
#define TARGET_ATT_KP        8.0f

// Body-rate clamps (deg/s). Conservative for a 300 km/h / ~80deg platform; the
// inner rate PID still tracks these exactly as it does stick-commanded rates.
#define TARGET_MAX_RATE_RP   300.0f
#define TARGET_MAX_RATE_YAW  180.0f

static quaternion qSp = QUATERNION_INITIALIZE;
static float targetGain = 0.0f;
static timeUs_t lastUpdateUs = 0;
static float rateSp[XYZ_AXIS_COUNT] = { 0.0f, 0.0f, 0.0f };

void targetAttitudeInit(void)
{
    qSp.w = 1.0f;
    qSp.x = qSp.y = qSp.z = 0.0f;
    targetGain = 0.0f;
    lastUpdateUs = 0;
    rateSp[FD_ROLL] = rateSp[FD_PITCH] = rateSp[FD_YAW] = 0.0f;
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

void targetAttitudeUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (!targetAttitudeIsFresh()) {
        rateSp[FD_ROLL] = rateSp[FD_PITCH] = rateSp[FD_YAW] = 0.0f;
        DEBUG_SET(DEBUG_ANGLE_TARGET, 0, 0);
        DEBUG_SET(DEBUG_ANGLE_TARGET, 1, 0);
        DEBUG_SET(DEBUG_ANGLE_TARGET, 2, 0);
        DEBUG_SET(DEBUG_ANGLE_TARGET, 3, 0);
        return;
    }

    quaternion qCur;
    getQuaternion(&qCur);

    // Body-frame attitude error: q_err = conj(qCur) (x) qSp.
    // Both quaternions are body->earth; q_err expresses the desired orientation
    // relative to the current body frame, so its vector part maps straight to
    // body roll/pitch/yaw (X-fwd / Y-right / Z-down).
    const float cw =  qCur.w;
    const float cx = -qCur.x;
    const float cy = -qCur.y;
    const float cz = -qCur.z;

    const float sw = qSp.w, sx = qSp.x, sy = qSp.y, sz = qSp.z;

    float ew = cw * sw - cx * sx - cy * sy - cz * sz;
    float ex = cw * sx + cx * sw + cy * sz - cz * sy;
    float ey = cw * sy - cx * sz + cy * sw + cz * sx;
    float ez = cw * sz + cx * sy - cy * sx + cz * sw;

    // Shortest path: a quaternion and its negation are the same rotation; pick
    // the hemisphere with w >= 0 so we never spin the long way round.
    if (ew < 0.0f) {
        ex = -ex;
        ey = -ey;
        ez = -ez;
    }

    // body rate (deg/s) = 2 * Kp * gain * q_err_vec  (small-angle: q_vec ~ 0.5*theta)
    const float k = 2.0f * TARGET_ATT_KP * targetGain;
    const float rRoll  = RADIANS_TO_DEGREES(k * ex);
    const float rPitch = RADIANS_TO_DEGREES(k * ey);
    const float rYaw   = RADIANS_TO_DEGREES(k * ez);

    rateSp[FD_ROLL]  = constrainf(rRoll,  -TARGET_MAX_RATE_RP,  TARGET_MAX_RATE_RP);
    rateSp[FD_PITCH] = constrainf(rPitch, -TARGET_MAX_RATE_RP,  TARGET_MAX_RATE_RP);
    rateSp[FD_YAW]   = constrainf(rYaw,   -TARGET_MAX_RATE_YAW, TARGET_MAX_RATE_YAW);

    DEBUG_SET(DEBUG_ANGLE_TARGET, 0, lrintf(rateSp[FD_ROLL]));
    DEBUG_SET(DEBUG_ANGLE_TARGET, 1, lrintf(rateSp[FD_PITCH]));
    DEBUG_SET(DEBUG_ANGLE_TARGET, 2, lrintf(rateSp[FD_YAW]));
    DEBUG_SET(DEBUG_ANGLE_TARGET, 3, lrintf(targetGain * 1000.0f));
}

float targetAttitudeRateSetpoint(int axis)
{
    return rateSp[axis];
}
