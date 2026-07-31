// j1939.hpp. Minimal J1939 encode / decode for the cooling loop messages.
//
// Only the handful of parameter groups this subsystem actually needs are
// implemented.  Scaling and offsets follow SAE J1939-71 where a standard SPN
// exists, so the traffic is readable on any off-the-shelf bus analyser rather
// than needing a bespoke DBC.  Where no standard SPN fits (pump speed command),
// a proprietary PGN in the manufacturer-specific range is used and documented.

#ifndef EAE_J1939_HPP
#define EAE_J1939_HPP

#include <cstdint>

#include "eae/can_bus.hpp"

namespace eae {
namespace j1939 {

// Parameter group numbers.
enum : std::uint32_t {
    kPgnEngineTemp1 = 0xFEEE,   // ET1, contains SPN 110 coolant temperature
    kPgnPumpCommand = 0xFF10,   // proprietary B: commanded pump speed
    kPgnPumpStatus = 0xFF11,    // proprietary B: pump speed, current, faults
    kPgnCoolingStatus = 0xFF12, // proprietary B: to the PowerView 450
    kPgnPowerRequest = 0xFF13,  // proprietary B: derate / shutdown to inverter
};

// Build a 29-bit identifier from priority, PGN and source address.
inline std::uint32_t makeId(std::uint8_t priority, std::uint32_t pgn,
                            std::uint8_t source) {
    return (static_cast<std::uint32_t>(priority & 0x07) << 26) |
           ((pgn & 0x3FFFF) << 8) | source;
}

inline std::uint32_t pgnOf(std::uint32_t id) { return (id >> 8) & 0x3FFFF; }
inline std::uint8_t sourceOf(std::uint32_t id) {
    return static_cast<std::uint8_t>(id & 0xFF);
}

// SPN 110: 1 degC per bit, -40 degC offset, one byte.  0xFF means not available.
inline std::uint8_t encodeTemperature(double celsius) {
    double raw = celsius + 40.0;
    if (raw < 0.0) raw = 0.0;
    if (raw > 250.0) raw = 250.0;
    return static_cast<std::uint8_t>(raw + 0.5);
}

inline bool decodeTemperature(std::uint8_t raw, double &celsius) {
    if (raw >= 0xFB) return false;  // 0xFB..0xFF are error / not-available
    celsius = static_cast<double>(raw) - 40.0;
    return true;
}

// Duty cycle, 0.4 % per bit (J1939 convention for percent in one byte).
inline std::uint8_t encodePercent(double percent) {
    if (percent < 0.0) percent = 0.0;
    if (percent > 100.0) percent = 100.0;
    return static_cast<std::uint8_t>(percent / 0.4 + 0.5);
}

inline double decodePercent(std::uint8_t raw) {
    return static_cast<double>(raw) * 0.4;
}

// Speed, 0.125 rpm per bit, little endian, two bytes.
inline void encodeRpm(int rpm, std::uint8_t &lo, std::uint8_t &hi) {
    if (rpm < 0) rpm = 0;
    std::uint32_t raw = static_cast<std::uint32_t>(rpm * 8);
    if (raw > 0xFAFF) raw = 0xFAFF;
    lo = static_cast<std::uint8_t>(raw & 0xFF);
    hi = static_cast<std::uint8_t>((raw >> 8) & 0xFF);
}

inline int decodeRpm(std::uint8_t lo, std::uint8_t hi) {
    std::uint32_t raw = static_cast<std::uint32_t>(hi) << 8 | lo;
    if (raw >= 0xFB00) return -1;  // not available
    return static_cast<int>(raw / 8);
}

// --- Message builders -------------------------------------------------------

CanFrame makePumpCommand(double duty_percent, bool enable);
CanFrame makePumpStatus(int rpm, double current_a, bool fault);
CanFrame makeCoolingStatus(double coolant_c, double fan_duty,
                           std::uint8_t state, std::uint8_t fault_code);
CanFrame makePowerRequest(bool derate, bool shutdown, double limit_percent);
CanFrame makeEngineTemp1(double coolant_c);

}  // namespace j1939
}  // namespace eae

#endif  // EAE_J1939_HPP
