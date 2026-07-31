#include "eae/can_bus.hpp"

#include <algorithm>

#include "eae/j1939.hpp"

namespace eae {

void VirtualCanBus::attach(std::uint8_t node) {
    auto it = std::find_if(queues_.begin(), queues_.end(),
                           [node](const Queue &q) { return q.node == node; });
    if (it == queues_.end()) queues_.push_back(Queue{node, {}});
}

bool VirtualCanBus::send(const CanFrame &frame) {
    if (drop_remaining_ > 0) {
        --drop_remaining_;
        ++frames_dropped_;
        return true;  // the sender cannot tell: that is the whole problem
    }

    CanFrame stamped = frame;
    stamped.timestamp_us = now_us_;
    const std::uint8_t source = j1939::sourceOf(frame.id);

    for (auto &q : queues_) {
        if (q.node == source) continue;  // a node does not hear itself
        if (q.frames.size() >= queue_depth_) {
            // Oldest frame is discarded.  Losing the stale one rather than
            // the fresh one is the right trade for periodic status data.
            q.frames.pop_front();
            ++frames_overrun_;
        }
        q.frames.push_back(stamped);
    }
    ++frames_sent_;
    return true;
}

bool VirtualCanBus::receive(std::uint8_t node, CanFrame &out) {
    for (auto &q : queues_) {
        if (q.node != node) continue;
        if (q.frames.empty()) return false;
        out = q.frames.front();
        q.frames.pop_front();
        return true;
    }
    return false;
}

namespace j1939 {

CanFrame makePumpCommand(double duty_percent, bool enable) {
    CanFrame f;
    f.id = makeId(6, kPgnPumpCommand, kAddrController);
    f.data[0] = enable ? 0x01 : 0x00;
    f.data[1] = encodePercent(duty_percent);
    return f;
}

CanFrame makePumpStatus(int rpm, double current_a, bool fault) {
    CanFrame f;
    f.id = makeId(6, kPgnPumpStatus, kAddrPump);
    encodeRpm(rpm, f.data[0], f.data[1]);
    // Current, 0.1 A per bit, one byte, saturating at 25.5 A.  The WP32 draws
    // at most 15 A at 24 V, so one byte is comfortable.
    double raw = current_a * 10.0;
    if (raw < 0.0) raw = 0.0;
    if (raw > 250.0) raw = 250.0;
    f.data[2] = static_cast<std::uint8_t>(raw + 0.5);
    f.data[3] = fault ? 0x01 : 0x00;
    return f;
}

CanFrame makeCoolingStatus(double coolant_c, double fan_duty,
                           std::uint8_t state, std::uint8_t fault_code) {
    CanFrame f;
    f.id = makeId(6, kPgnCoolingStatus, kAddrController);
    f.data[0] = encodeTemperature(coolant_c);
    f.data[1] = encodePercent(fan_duty);
    f.data[2] = state;
    f.data[3] = fault_code;
    return f;
}

CanFrame makePowerRequest(bool derate, bool shutdown, double limit_percent) {
    CanFrame f;
    // Priority 3: a power-limit request outranks periodic status traffic.
    f.id = makeId(3, kPgnPowerRequest, kAddrController);
    f.data[0] = static_cast<std::uint8_t>((derate ? 0x01 : 0x00) |
                                          (shutdown ? 0x02 : 0x00));
    f.data[1] = encodePercent(limit_percent);
    return f;
}

CanFrame makeEngineTemp1(double coolant_c) {
    CanFrame f;
    f.id = makeId(6, kPgnEngineTemp1, kAddrController);
    f.data[0] = encodeTemperature(coolant_c);  // SPN 110
    return f;
}

}  // namespace j1939
}  // namespace eae
