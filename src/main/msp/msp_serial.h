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

#pragma once

#include "drivers/time.h"

#include "io/serial.h"

#include "msp/msp.h"

// Each MSP port requires state and a receive buffer, revisit this default if someone needs more than 3 MSP ports.
#define MAX_MSP_PORT_COUNT 3

typedef enum {
    MSP_IDLE,
    MSP_HEADER_START,
    MSP_HEADER_M,
    MSP_HEADER_X,

    MSP_HEADER_V1,
    MSP_PAYLOAD_V1,
    MSP_CHECKSUM_V1,

    MSP_HEADER_V2_OVER_V1,
    MSP_PAYLOAD_V2_OVER_V1,
    MSP_CHECKSUM_V2_OVER_V1,

    MSP_HEADER_V2_NATIVE,
    MSP_PAYLOAD_V2_NATIVE,
    MSP_CHECKSUM_V2_NATIVE,

    MSP_COMMAND_RECEIVED
} mspState_e;

typedef enum {
    MSP_PACKET_COMMAND,
    MSP_PACKET_REPLY
} mspPacketType_e;

typedef enum {
    MSP_EVALUATE_NON_MSP_DATA,
    MSP_SKIP_NON_MSP_DATA
} mspEvaluateNonMspData_e;

typedef enum {
    MSP_PENDING_NONE,
    MSP_PENDING_BOOTLOADER_ROM,
    MSP_PENDING_CLI,
    MSP_PENDING_BOOTLOADER_FLASH,
} mspPendingSystemRequest_e;

#define MSP_PORT_INBUF_SIZE 192
#define MSP_PORT_OUTBUF_SIZE_MIN 320

#ifdef USE_FLASHFS
#define MSP_PORT_DATAFLASH_BUFFER_SIZE 4096
#define MSP_PORT_DATAFLASH_INFO_SIZE 16
#define MSP_PORT_OUTBUF_SIZE (MSP_PORT_DATAFLASH_BUFFER_SIZE + MSP_PORT_DATAFLASH_INFO_SIZE)
#else
#define MSP_PORT_OUTBUF_SIZE MSP_PORT_OUTBUF_SIZE_MIN // As of 2021/08/10 MSP_BOXNAMES generates a 307 byte response for page 1.
#endif

typedef struct __attribute__((packed)) {
    uint8_t size;
    uint8_t cmd;
} mspHeaderV1_t;

typedef struct __attribute__((packed)) {
    uint16_t size;
} mspHeaderJUMBO_t;

typedef struct __attribute__((packed)) {
    uint8_t  flags;
    uint16_t cmd;
    uint16_t size;
} mspHeaderV2_t;

#define MSP_MAX_HEADER_SIZE     9

struct serialPort_s;
typedef struct mspPort_s {
    struct serialPort_s *port; // null when port unused.
    timeMs_t lastActivityMs;
    mspPendingSystemRequest_e pendingRequest;
    mspState_e c_state;
    mspPacketType_e packetType;
    uint8_t inBuf[MSP_PORT_INBUF_SIZE];
    uint16_t cmdMSP;
    uint8_t cmdFlags;
    mspVersion_e mspVersion;
    uint_fast16_t offset;
    uint_fast16_t dataSize;
    uint8_t checksum1;
    uint8_t checksum2;
    bool sharedWithTelemetry;
    mspDescriptor_t descriptor;
} mspPort_t;

void mspSerialInit(void);
bool mspSerialWaiting(void);
void mspSerialProcess(mspEvaluateNonMspData_e evaluateNonMspData, mspProcessCommandFnPtr mspProcessCommandFn, mspProcessReplyFnPtr mspProcessReplyFn);
void mspSerialAllocatePorts(void);
void mspSerialReleasePortIfAllocated(struct serialPort_s *serialPort);
void mspSerialReleaseSharedTelemetryPorts(void);
mspDescriptor_t getMspSerialPortDescriptor(const uint8_t portIdentifier);
int mspSerialPush(serialPortIdentifier_e port, uint8_t cmd, uint8_t *data, int datalen, mspDirection_e direction, mspVersion_e mspVersion);
uint32_t mspSerialTxBytesFree(void);

// Frames the parser threw away, aggregated across all MSP ports.
//
// Every reject path in mspSerialProcessReceivedData() used to just set c_state back to
// MSP_IDLE and move on, with no record that anything happened. That makes two very
// different failures look identical to the companion computer: "my frame never
// arrived" and "my frame arrived corrupt and you discarded it". Both read as a missing
// reply, and one is a wiring/baud fault while the other is a TX-side fault -- they have
// different fixes and pointing at the wrong one wastes a bench session.
//
// Split three ways because the split is what carries the information:
//   checksum    a COMPLETE frame arrived and its checksum/CRC did not match. Bytes were
//               altered in transit: noise, a baud mismatch, or two writers interleaving
//               on the same fd.
//   header      a frame started ('$') but the header did not parse -- wrong direction
//               byte, an over-size length, a truncated V2 header. This is desync: the
//               parser lost byte alignment, which a single lost or injected byte causes.
//   strayIdle   bytes arrived while the parser was waiting for '$'. On a quiet, healthy
//               link this stays at zero, so any growth is line noise or another device
//               driving the same wire.
//
// A clean loss -- the frame never reaching the UART at all -- increments NONE of these.
// That is the point: if the companion computer's sent count runs ahead of the FC's
// received count while all three counters here stay zero, the bytes did not reach the
// FC's parser, which exonerates the wire and points at the FC's serial RX buffer
// overflowing or the sender never actually writing.
typedef struct mspDiscardCounters_s {
    uint32_t checksum;
    uint32_t header;
    uint32_t strayIdle;
} mspDiscardCounters_t;

const mspDiscardCounters_t *mspGetDiscardCounters(void);
