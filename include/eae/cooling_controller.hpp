// cooling_controller.hpp. Ties the state machine, PID and CAN layer together.

#ifndef EAE_COOLING_CONTROLLER_HPP
#define EAE_COOLING_CONTROLLER_HPP

#include <cstdint>

#include "eae/can_bus.hpp"
#include "eae/pid.hpp"
#include "eae/state_machine.hpp"

namespace eae {

// Hard-wired inputs read directly by the CR0403, as opposed to CAN inputs.
struct DiscreteInputs {
    double temperature_resistance_ohm = 3457.0;  // IN4, resistance mode
    bool temperature_over_range = false;         // > 3.6 kOhm, i.e. cold
    bool ignition = false;                       // IN0
    bool level_wet = true;                       // IN8, LMC100
    bool fan_diagnosis_ok = true;                // CR0403 output diagnosis
    double supply_voltage = 27.0;
};

struct Commands {
    bool pump_enable = false;
    double pump_duty = 0.0;
    bool fan_contactor = false;
    double fan_duty = 0.0;
    bool derate_request = false;
    bool shutdown_request = false;
    bool lamp_warning = false;
    bool lamp_fault = false;
    State state = State::Init;
    Fault fault = Fault::None;
    double temperature_c = 0.0;
    bool temperature_valid = true;
};

class CoolingController {
public:
    CoolingController(ICanBus &bus, const Thresholds &th, const PidGains &g);

    // One control scan.  now_us is used only for CAN timestamps and transmit
    // scheduling; all control timing uses dt_s so the loop stays testable at
    // any rate.
    Commands scan(const DiscreteInputs &in, double dt_s, std::uint64_t now_us);

    const StateMachine &stateMachine() const { return sm_; }
    Pid &pid() { return pid_; }

    int lastPumpRpm() const { return pump_rpm_; }
    double timeSincePumpStatus() const { return t_since_pump_status_; }

private:
    void serviceReceive(std::uint64_t now_us);
    void transmit(const Commands &c, std::uint64_t now_us, double dt_s);
    double fanDutyFor(double temperature_c, double dt_s);
    double pumpDutyFor(double temperature_c) const;

    ICanBus &bus_;
    StateMachine sm_;
    Pid pid_;

    int pump_rpm_ = 0;
    double pump_current_a_ = 0.0;
    bool pump_reported_fault_ = false;
    double t_since_pump_status_ = 0.0;

    bool last_pump_enable_ = false;
    bool fan_latched_on_ = false;
    double t_fan_on_ = 0.0;
    double t_since_tx_status_ = 0.0;
    double t_since_tx_command_ = 0.0;
};

// --- Sensor conversion ------------------------------------------------------
// H-WTMS resistance to degrees Celsius, log-interpolated over the datasheet
// table.  Returns false only for a genuinely faulted harness; a cold
// over-range is valid data clamped at +20 C.
bool resistanceToCelsius(double ohms, bool over_range, double &celsius);

}  // namespace eae

#endif  // EAE_COOLING_CONTROLLER_HPP
