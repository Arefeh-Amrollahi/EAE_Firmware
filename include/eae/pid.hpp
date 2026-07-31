// pid.hpp. Positional PID with clamped integrator and derivative on measurement.

#ifndef EAE_PID_HPP
#define EAE_PID_HPP

namespace eae {

struct PidGains {
    double kp = 8.0;    // percent duty per degC of error
    double ki = 0.25;   // percent duty per degC per second
    double kd = 12.0;   // percent duty per (degC per second)
};

class Pid {
public:
    Pid(PidGains gains, double out_min, double out_max);

    // dt_s must be positive; a non-positive dt returns the proportional term
    // only, rather than dividing by zero, because a scan overrun that reports
    // dt = 0 must not be allowed to spike the actuator.
    double update(double setpoint, double measurement, double dt_s);

    void reset();

    void setGains(const PidGains &g) { gains_ = g; }
    PidGains gains() const { return gains_; }

    double integral() const { return integral_; }
    bool saturated() const { return saturated_; }

private:
    PidGains gains_;
    double out_min_;
    double out_max_;
    double integral_ = 0.0;
    double prev_measurement_ = 0.0;
    bool have_prev_ = false;
    bool saturated_ = false;
};

}  // namespace eae

#endif  // EAE_PID_HPP
