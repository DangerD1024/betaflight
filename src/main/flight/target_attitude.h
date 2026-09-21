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
// There is exactly ONE setpoint contract: MSP_SET_TARGET_CORRECTION (231), a
// body-frame correction quaternion that the FC composes against its own current
// attitude. The absolute-attitude command (229) is RETIRED.
//
// It used to be selectable per airframe, which is what made the 2026-09-07 flight
// fly its whole engagement on the manual stick law: the FC and the app disagreed
// about which command to speak, the FC refused every packet, and because the app
// streams fire-and-forget neither side could see it. Two contracts meant two things
// that could disagree; one contract removes the failure rather than detecting it.
//
// Reported to the app by MSP_TARGET_INFO so a mismatched pairing fails loudly at
// startup instead of at 100 m. Bumped when the wire format or the supported set
// changes: v1 offered 229/231 selectably, v2 is correction-only.
#define TARGET_INFO_PROTOCOL_VERSION 2

typedef struct targetAttitudeConfig_s {
    uint16_t att_kp;        // x10: AUTONOMOUS attitude P gain (80 = 8.0)
    uint16_t max_accel;     // deg/s^2: rate-slew accel limit (0 = off)
    uint16_t max_rate_rp;   // deg/s: roll/pitch body-rate clamp
    uint16_t max_rate_yaw;  // deg/s: yaw body-rate clamp
    uint16_t level_gain;    // x10: manual wings-level P gain (15 = 1.5; 0 = off)
    uint8_t  rate_deadband; // deg/s: manual resting-stick deadband
    uint8_t  rotation;      // targetRotation_e: ROLL or YAW stick pans the heading
    // Camera-servo nudge while TARGET + MSP OVERRIDE are both on. New fields must be
    // APPENDED here: pgLoad() copies MIN(storedSize, pgSize) over the reset template, so
    // appending keeps a pilot's tuned target_* values across a flash. Reordering or
    // removing a field needs a PG version bump instead.
    int16_t  servo_correction; // us added to servo_index while both modes are on (0 = off)
    uint16_t servo_speed;      // ms for the correction to ramp fully in/out (0 = instant)
    uint8_t  servo_index;      // servo output to nudge; same 0-based index as CLI `servo <n> ...`
} targetAttitudeConfig_t;

PG_DECLARE(targetAttitudeConfig_t, targetAttitudeConfig);

// Called by msp.c when the RETIRED absolute-attitude command (229) arrives. The FC
// refuses the packet, but the fact that something is still SENDING it is the single
// most useful thing we can know: it means an old app is paired with this firmware and
// is talking to us in a dialect we no longer answer. Without this the symptom is
// indistinguishable from an app that is not running at all -- both show "no setpoint
// has ever arrived" -- and on 2026-09-07 telling those apart cost two flights.
void targetAttitudeNoteRetiredCommand(void);

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

// The last MSP_SET_TARGET_CORRECTION command as received, in the wire's own
// fixed-point scale (quaternion components x16384 in out[0..3], gain x10000 in
// out[4]). Logged to the blackbox so a flight records what the app actually
// commanded, not only what the FC derived from it.
void targetAttitudeGetLastCommand(int16_t out[5]);

// Mode-activation state for the blackbox slow frame, so "was TARGET really on,
// and was it following the app or the sticks?" is readable from the log alone:
//   bit0  TARGET_MODE flight mode active
//   bit1  autonomous: a 231 setpoint is fresh and driving the rate law
//   bit2  BOXMSPOVERRIDE active (MSP RC override applied to the masked channels)
uint8_t targetAttitudeBlackboxState(void);

// Advance the camera-servo correction ramp one loop and return the offset (us) to add to
// servo servo_index. Call exactly once per loop -- both the ramp rate and the arm-delay
// timing depend on the interval between calls, so skipping loops stretches both.
//
// Returns 0 unless ALL of these hold:
//   - the aircraft is ARMED
//   - TARGET_MODE is active and BOXMSPOVERRIDE is active as an RC mode
//   - at least 2 s (TARGET_SERVO_ARM_DELAY_US) has elapsed since the last of those
//     became true -- so arming with TARGET already enabled starts the clock at the arm
//     instant, and the reverse starts it at the enable instant
//   - target_servo_correction is non-zero
// Losing any of them clears the 2 s latch, so re-engaging waits it out again from zero.
// The ramp back to 0 on the way out is NOT delayed: centring the camera is the safe
// direction and should not be held off.
int16_t targetServoOffsetUpdate(void);
