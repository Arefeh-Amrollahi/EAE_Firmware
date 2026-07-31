#include "eae/pid.hpp"

#include <algorithm>

namespace eae {

Pid::Pid(PidGains gains, double out_min, double out_max)
    : gains_(gains), out_min_(out_min), out_max_(out_max) {}

void Pid::reset() {
    integral_ = 0.0;
    have_prev_ = false;
    saturated_ = false;
}

double Pid::update(double setpoint, double measurement, double dt_s) {
    // Positive error means "too hot", so more cooling.  Signing it this way
    // keeps every gain positive, which is one fewer thing to get wrong at
    // commissioning time.
    const double error = measurement - setpoint;
    const double p = gains_.kp * error;

    if (dt_s <= 0.0) {
        // A scan overrun reporting dt = 0 must not divide by zero or spike the
        // actuator.  Fall back to proportional action for this scan only.
        prev_measurement_ = measurement;
        have_prev_ = true;
        return std::clamp(p + integral_, out_min_, out_max_);
    }

    // Integrate, then clamp the accumulator itself.  Clamping only the output
    // would let the integrator keep winding up through a long saturated climb,
    // such as a loaded ramp haul, and then hold the fan at full duty long
    // after the temperature has come back down.
    integral_ += gains_.ki * error * dt_s;
    integral_ = std::clamp(integral_, out_min_, out_max_);

    double d = 0.0;
    if (have_prev_) {
        // Derivative on the measurement, not the error: a setpoint change made
        // from the display must not produce a derivative kick.
        d = gains_.kd * (measurement - prev_measurement_) / dt_s;
    }
    prev_measurement_ = measurement;
    have_prev_ = true;

    const double raw = p + integral_ + d;
    const double out = std::clamp(raw, out_min_, out_max_);
    saturated_ = (out != raw);
    return out;
}

}  // namespace eae
