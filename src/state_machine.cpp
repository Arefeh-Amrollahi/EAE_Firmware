#include "eae/state_machine.hpp"

namespace eae {

const char *toString(State s) {
    switch (s) {
        case State::Init: return "INIT";
        case State::Standby: return "STANDBY";
        case State::Prime: return "PRIME";
        case State::Running: return "RUNNING";
        case State::Derate: return "DERATE";
        case State::Fault: return "FAULT";
        case State::PostRun: return "POSTRUN";
    }
    return "?";
}

const char *toString(Fault f) {
    switch (f) {
        case Fault::None: return "none";
        case Fault::TemperatureSensor: return "temperature sensor out of range";
        case Fault::CoolantLevel: return "coolant level low";
        case Fault::OverTemperature: return "coolant over-temperature";
        case Fault::PumpComms: return "no pump status on CAN";
        case Fault::PumpStall: return "pump commanded but not turning";
        case Fault::FanCircuit: return "fan output diagnosis";
    }
    return "?";
}

void StateMachine::transitionTo(State s) {
    if (s == state_) return;
    state_ = s;
    t_state_ = 0.0;
}

Fault StateMachine::evaluateFaults(const Conditions &c) {
    // Priority is causes before symptoms, and it is deterministic rather than
    // whichever timer happens to expire first.  A stalled pump makes the loop
    // overheat; reporting the over-temperature would send a technician to look
    // at the radiator when the pump is the thing that failed.  The machine is
    // still shut down on temperature, that is handled separately, driven by
    // overTemperatureConfirmed(), so the diagnosis and the protection are not
    // competing for the same latch.
    if (!c.temperature_valid) return Fault::TemperatureSensor;
    if (t_level_low_ >= th_.level_trip_s) return Fault::CoolantLevel;
    if (c.pump_commanded && t_comms_lost_ > th_.can_timeout_s)
        return Fault::PumpComms;
    if (t_pump_no_flow_ > th_.prime_s) return Fault::PumpStall;
    if (!c.fan_circuit_ok) return Fault::FanCircuit;
    if (t_over_temp_ >= th_.shutdown_confirm_s) return Fault::OverTemperature;
    return Fault::None;
}

State StateMachine::step(const Conditions &c, double dt_s) {
    t_state_ += dt_s;

    // --- Accumulate the timed conditions ----------------------------------
    t_level_low_ = c.level_ok ? 0.0 : t_level_low_ + dt_s;
    t_comms_lost_ = c.pump_comms_ok ? 0.0 : t_comms_lost_ + dt_s;

    // Compare what was commanded against what the feedback reports.  Timing
    // this from the command rather than from entry into RUNNING means a pump
    // that is already seized is caught during priming, before the machine is
    // ever loaded.
    t_pump_no_flow_ = (c.pump_commanded && !c.pump_turning)
                          ? t_pump_no_flow_ + dt_s
                          : 0.0;
    t_over_temp_ = (c.temperature_valid && c.temperature_c >= th_.shutdown_c)
                       ? t_over_temp_ + dt_s
                       : 0.0;

    // --- Faults are evaluated before the mode logic ------------------------
    // No state can then accidentally mask a fault by handling its own
    // transitions first.  Faults latch: an intermittent coolant problem that
    // quietly self-clears is exactly what an operator needs to be told about.
    const Fault f = evaluateFaults(c);
    if (f != Fault::None && fault_ == Fault::None) {
        fault_ = f;
        transitionTo(State::Fault);
        return state_;
    }

    // --- Mode logic --------------------------------------------------------
    switch (state_) {
        case State::Init:
            // One scan of settling so the input filters hold valid data before
            // any decision is taken on them.
            transitionTo(State::Standby);
            break;

        case State::Standby:
            if (c.ignition) {
                fault_ = Fault::None;  // a key cycle clears latched faults
                transitionTo(State::Prime);
            }
            break;

        case State::Prime:
            if (!c.ignition) {
                transitionTo(State::Standby);
            } else if (t_state_ >= th_.prime_s) {
                transitionTo(State::Running);
            }
            break;

        case State::Running:
            if (!c.ignition) {
                transitionTo(State::PostRun);
            } else if (c.temperature_valid && c.temperature_c >= th_.derate_c) {
                transitionTo(State::Derate);
            }
            break;

        case State::Derate:
            if (!c.ignition) {
                transitionTo(State::PostRun);
            } else if (c.temperature_valid &&
                       c.temperature_c < th_.derate_c - th_.hysteresis_c) {
                // Recover only once comfortably back inside the band, so the
                // inverter is not cycled in and out of derate at the threshold.
                transitionTo(State::Running);
            }
            break;

        case State::PostRun:
            if (c.ignition) {
                transitionTo(State::Running);
            } else if (t_state_ >= th_.post_run_s) {
                transitionTo(State::Standby);
            }
            break;

        case State::Fault:
            if (!c.ignition) transitionTo(State::Standby);
            break;
    }
    return state_;
}

}  // namespace eae
