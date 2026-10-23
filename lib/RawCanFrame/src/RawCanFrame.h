#ifndef RAW_CAN_FRAME_H
#define RAW_CAN_FRAME_H

// Payload of the raw 11-bit CAN pressure frames (RAWCAN_ENABLE = 1).
//
// For receivers that cannot subscribe to DroneCAN messages, such as an
// ArduPilot 4.6 Lua script attached with CAN_D1_PROTOCOL2 = 10, which only
// receives 11-bit frames. lua_scripts/adm_static_pressure_log.lua decodes them.
//
// Two frame kinds, each at its own rate:
//   ID RAWCAN_ID      filtered pressure: the value published as StaticPressure
//                     (filtered unless BARO_LP_ENABLE = 0), sent together with
//                     it at BARO_PUB_HZ
//   ID RAWCAN_RAW_ID  unfiltered pressure: every sensor sample before the
//                     filter, at the sample rate (BARO_SAMPLE_HZ)
// Each frame:
//   DLC     5
//   0-3     static pressure, Pa, float32 little-endian
//   4       sequence counter 0-255, one counter per frame kind: +1 per
//           published value (filtered) or per sample (unfiltered). It also
//           advances for frames that could not be sent, so a receiver sees
//           every gap.
//
// Plain C++ with no Arduino dependency so it can be unit tested on the host.

#include <stdint.h>
#include <string.h>

static constexpr uint16_t RAW_CAN_ID_MAX = 0x7FF;
static constexpr uint8_t RAW_CAN_PRESSURE_DLC = 5;

// Fill out[0..4]; returns the number of bytes used.
inline uint8_t raw_can_pack_pressure(float pressure_pa, uint8_t seq, uint8_t out[8])
{
    uint32_t bits;
    static_assert(sizeof(bits) == sizeof(pressure_pa), "float must be 32 bit");
    memcpy(&bits, &pressure_pa, sizeof(bits));
    out[0] = (uint8_t)(bits);
    out[1] = (uint8_t)(bits >> 8);
    out[2] = (uint8_t)(bits >> 16);
    out[3] = (uint8_t)(bits >> 24);
    out[4] = seq;
    return RAW_CAN_PRESSURE_DLC;
}

#endif // RAW_CAN_FRAME_H
