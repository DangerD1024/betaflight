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

// TARGET_MODE attitude controller.
//
// An external app (the VOT_C interceptor) streams a desired ATTITUDE QUATERNION
// (body->earth, same convention as imu.c::getQuaternion) plus a [0..1] gain over
// MSP (MSP_SET_TARGET_ATTITUDE). Each PID loop this module computes the body-frame
// attitude error and turns it into a per-axis body-rate setpoint (deg/s), which
// pid.c injects in place of the stick/angle setpoint when FLIGHT_MODE(TARGET_MODE)
// is active. Gimbal-lock-free (operates on the quaternion, never Euler), so it is
// safe at the ~80deg pitch the interceptor flies at.

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

// Which stick pans the heading in STANDALONE MANUAL (the other one is ignored).
typedef enum {
    TARGET_ROTATION_ROLL = 0,
    TARGET_ROTATION_YAW,
} targetRotation_e;

// CLI-settable TARGET_MODE tunables, so the rate law / autonomous loop can be tuned
// from real flights with `set ...; save` (no reflash). Gains stored x10.
typedef struct targetAttitudeConfig_s {
    uint16_t att_kp;        // x10: AUTONOMOUS attitude P gain (80 = 8.0)
    uint16_t max_accel;     // deg/s^2: rate-slew accel limit (0 = off)
    uint16_t max_rate_rp;   // deg/s: roll/pitch body-rate clamp
    uint16_t max_rate_yaw;  // deg/s: yaw body-rate clamp
    uint16_t level_gain;    // x10: manual wings-level P gain (15 = 1.5; 0 = off)
    uint8_t  rate_deadband; // deg/s: manual resting-stick deadband
    uint8_t  rotation;      // targetRotation_e: ROLL or YAW stick pans the heading
} targetAttitudeConfig_t;

PG_DECLARE(targetAttitudeConfig_t, targetAttitudeConfig);

void targetAttitudeInit(void);

// Store a new desired attitude quaternion (w,x,y,z, body->earth) and gain [0..1].
// Called from the MSP task; stamps the receipt time for the freshness failsafe.
void targetAttitudeSet(float w, float x, float y, float z, float gain);

// Store a new BODY-FRAME correction quaternion (w,x,y,z) and gain [0..1]: the
// absolute setpoint is composed HERE as q_sp = q_cur (x) q_corr against the FC's
// own current attitude at receipt (MSP_SET_TARGET_CORRECTION). Preferred over
// targetAttitudeSet: the app-side absolute q_sp was anchored to the MSP_ATTITUDE
// Euler echo (~100 ms stale, 0.1 deg roll/pitch + 1 deg yaw quantization), which
// re-entered q_err as setpoint jitter and rate-command spikes every frame.
void targetAttitudeSetCorrection(float w, float x, float y, float z, float gain);

// True while a setpoint has arrived within TARGET_STALE_US. Gates mode activation
// (core.c) and setpoint injection (pid.c): if the app stops streaming, the mode
// drops and the FC reverts to its other active modes.
bool targetAttitudeIsFresh(void);

// Recompute the body-rate setpoint from the latest quaternion error. Call once per
// PID loop, before the per-axis loop, so all three axes use one attitude snapshot.
void targetAttitudeUpdate(timeUs_t currentTimeUs);

// Body-rate setpoint (deg/s) for one axis (FD_ROLL/FD_PITCH/FD_YAW).
float targetAttitudeRateSetpoint(int axis);
