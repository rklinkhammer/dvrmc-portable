#pragma once
#include <cstdint>
#include "types.hpp"
namespace dvrmc { namespace reg {
constexpr uint32_t ID=0x00, ABI=0x04, MOTORS=0x08, PERIOD_US=0x0c;
constexpr uint32_t COMMIT=0x10, APPLIED=0x14, STATUS=0x18, STOP=0x1c;
constexpr uint32_t KEEPALIVE=0x20, SNAPSHOT=0x24, SNAPSHOT_SEQUENCE=0x28;
constexpr uint32_t CONFIG_BASE=0x100, CONFIG_STRIDE=0x100;
constexpr uint32_t TELEMETRY_BASE=0x1000, TELEMETRY_STRIDE=0x100;
constexpr uint32_t MAGIC=0x4456524d, ABI_VERSION=0x00010000;
constexpr uint32_t PENDING=1, REJECTED=2, STOPPED=4;
constexpr uint32_t POS_LO=0, POS_HI=4, SPEED=8, DUTY=12, FAULTS=16;
constexpr uint32_t ENCODER_ERRORS=20, RAMPED_TARGET=24;
constexpr uint32_t config(unsigned motor, field::Index f) {
    return CONFIG_BASE + motor * CONFIG_STRIDE + uint32_t(f)*4;
}
constexpr uint32_t telemetry(unsigned motor, uint32_t field_offset) {
    return TELEMETRY_BASE + motor * TELEMETRY_STRIDE + field_offset;
}
}}
