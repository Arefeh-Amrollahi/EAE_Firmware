// state_machine.hpp. Operating-mode state machine for the cooling loop.
//
// Kept deliberately separate from the control maths.  A reviewer should be able
// to read the mode logic without wading through PID tuning, and the transitions
// can then be unit tested exhaustively without instantiating a plant.

#ifndef EAE_STATE_MACHINE_HPP
#define EAE_STATE_MACHINE_HPP

#include <cstdint>

namespace eae {

enum class State : std::uint8_t {
    Init = 0,
    Standby = 1,   // key off, everything parked
    Prime = 2,     // pump at full flow, purging air before load is allowed
    Running = 3,   // normal closed-loop control
    Derate = 4,    // over temperature, requesting reduced power
    Fault = 5,     // latched fault, safe outputs
    PostRun = 6,   // key off, clearing residual heat soak
};

const char *toString(State s);

enum class Fault : std::uint8_t {
    None = 0,
    TemperatureSensor = 1,  // harness open or shorted
    CoolantLevel = 2,       // sustained low level
    OverTemperature = 3,    // shutdown threshold exceeded
    PumpComms = 4,          // no status frame from the WP32
    PumpStall = 5,          // commanded to run, reporting no shaft speed
    FanCircuit = 6,         // output diagnosis reports open load or short
};

const char *toString(Fault f);

struct Thresholds {
    double setpoint_c = 45.0;
    double fan_on_c = 38.0;
    double fan_full_c = 55.0;
    double hysteresis_c = 3.0;
    double warn_c = 60.0;
    double derate_c = 65.0;
    double shutdown_c = 75.0;

    double shutdown_confirm_s = 5.0;
    double prime_s = 3.0;
    double post_run_s = 120.0;
    double level_trip_s = 30.0;
    double can_timeout_s = 0.5;

    double pump_min_duty = 40.0;
    double fan_min_duty = 20.0;

    // Cold start.  Below about 20 C the NTC exceeds the CR0403 resistance
    // range and the input over-ranges, which the sensor layer reports as a
    // cold machine rather than a fault.  At -30 C, 50 % ethylene glycol is
    // roughly forty times more viscous than at 20 C, so commanding full pump
    // duty into it drives high pressure drop and high motor current for no
    // cooling benefit, since the loop has no heat in it yet.
    double pump_cold_limit = 55.0;
    double cold_ramp_s = 20.0;
    double cold_clamp_c = 20.0;
    double fan_min_on_s = 10.0;
};

// Everything the state machine needs to decide a transition.  Passing a
// struct rather than a long argument list means a new condition can be added
// without touching every call site.
struct Conditions {
    bool ignition = false;
    bool temperature_valid = true;
    double temperature_c = 25.0;
    bool level_ok = true;
    bool pump_comms_ok = true;
    bool pump_commanded = false;   // what we asked for on the previous scan
    bool pump_turning = true;      // what the feedback says actually happened
    bool fan_circuit_ok = true;
};

class StateMachine {
public:
    explicit StateMachine(const Thresholds &t) : th_(t) {}

    // Advances one scan.  Returns the state after the transition.
    State step(const Conditions &c, double dt_s);

    State state() const { return state_; }
    Fault fault() const { return fault_; }
    double timeInState() const { return t_state_; }

    // True once the coolant has been above the shutdown threshold for longer
    // than the confirm time.  Exposed separately from the latched fault code
    // because the two answer different questions: the fault code says what
    // broke, this says how hot the machine is right now.  A stalled pump and
    // an over-temperature can be true at the same time, and the operator needs
    // the root cause reported while the machine still gets shut down.
    bool overTemperatureConfirmed() const {
        return t_over_temp_ >= th_.shutdown_confirm_s;
    }

    void setThresholds(const Thresholds &t) { th_ = t; }
    const Thresholds &thresholds() const { return th_; }

private:
    void transitionTo(State s);
    Fault evaluateFaults(const Conditions &c);

    Thresholds th_;
    State state_ = State::Init;
    Fault fault_ = Fault::None;
    double t_state_ = 0.0;
    double t_level_low_ = 0.0;
    double t_over_temp_ = 0.0;
    double t_comms_lost_ = 0.0;
    double t_pump_no_flow_ = 0.0;
};

}  // namespace eae

#endif  // EAE_STATE_MACHINE_HPP
