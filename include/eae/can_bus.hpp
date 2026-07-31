// can_bus.hpp. CAN transport abstraction and an in-process virtual bus.
//
// The controller talks to an interface, never to a concrete bus.  On the
// machine that interface is backed by SocketCAN or the CR0403 CANopen stack;
// here it is backed by VirtualCanBus so the whole cooling loop can be run and
// unit tested on a development host with no hardware attached.  That split is
// the entire point: the control code under test is bit-for-bit the code that
// would ship.

#ifndef EAE_CAN_BUS_HPP
#define EAE_CAN_BUS_HPP

#include <array>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace eae {

struct CanFrame {
    std::uint32_t id = 0;               // 29-bit identifier when extended
    bool extended = true;               // J1939 is always 29-bit
    std::uint8_t dlc = 8;
    std::array<std::uint8_t, 8> data{}; // unused bytes stay 0xFF per J1939
    std::uint64_t timestamp_us = 0;

    CanFrame() { data.fill(0xFF); }
};

// Node addresses on the machine bus (J1939 source addresses).
enum : std::uint8_t {
    kAddrController = 0x21,   // CR0403 cooling controller
    kAddrPump = 0x2A,         // EMP WP32
    kAddrDisplay = 0x28,      // PowerView 450
    kAddrInverter = 0x00,     // traction inverter
};

class ICanBus {
public:
    virtual ~ICanBus() = default;

    // Returns false if the frame could not be queued (bus off, buffer full).
    virtual bool send(const CanFrame &frame) = 0;

    // Pops one frame addressed to `node`, if any.  Non-blocking by design:
    // a control task must never block on I/O inside its scan.
    virtual bool receive(std::uint8_t node, CanFrame &out) = 0;

    virtual void advance(std::uint64_t now_us) = 0;
};

// In-process bus with per-node receive queues.
//
// Two behaviours are modelled deliberately because they are the ones that
// break real systems: finite queue depth (so a node that stops reading loses
// frames rather than growing without bound) and injectable frame loss (so the
// receive-timeout logic can actually be exercised, instead of being written
// and never proven).
class VirtualCanBus : public ICanBus {
public:
    explicit VirtualCanBus(std::size_t queue_depth = 64)
        : queue_depth_(queue_depth) {}

    bool send(const CanFrame &frame) override;
    bool receive(std::uint8_t node, CanFrame &out) override;
    void advance(std::uint64_t now_us) override { now_us_ = now_us; }

    // Register a node so it gets a receive queue.  Frames are delivered to
    // every registered node except the sender, which mirrors a real broadcast
    // bus: filtering is the receiver's job.
    void attach(std::uint8_t node);

    // Drop the next `n` frames sent, to exercise timeout and recovery paths.
    void injectLoss(int n) { drop_remaining_ = n; }

    std::uint64_t framesSent() const { return frames_sent_; }
    std::uint64_t framesDropped() const { return frames_dropped_; }
    std::uint64_t framesOverrun() const { return frames_overrun_; }

private:
    struct Queue {
        std::uint8_t node;
        std::deque<CanFrame> frames;
    };

    std::vector<Queue> queues_;
    std::size_t queue_depth_;
    std::uint64_t now_us_ = 0;
    std::uint64_t frames_sent_ = 0;
    std::uint64_t frames_dropped_ = 0;
    std::uint64_t frames_overrun_ = 0;
    int drop_remaining_ = 0;
};

}  // namespace eae

#endif  // EAE_CAN_BUS_HPP
