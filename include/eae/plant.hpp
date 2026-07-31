// plant.hpp. Simulated hardware: the coolant loop and the WP32's own ECU.
//
// This is the half of the system that does NOT ship.  It exists so the
// firmware above can be run end to end, on a laptop, against something that
// pushes back the way real hardware does.

#ifndef EAE_PLANT_HPP
#define EAE_PLANT_HPP

#include <cstdint>

#include "eae/can_bus.hpp"

namespace eae {

class ThermalPlant {
public:
    ThermalPlant(double coolant_litres = 15.0, double t_initial_c = 25.0,
                 double t_ambient_c = 32.0);

    // Advances the lumped model one step.
    //   C dT/dt = Q_in * flow - UA(fan, flow) * (T - T_ambient)
    double step(double heat_load_w, double fan_duty, double pump_duty,
                double dt_s);

    double temperature() const { return temperature_c_; }
    void setUaScale(double s) { ua_scale_ = s; }   // < 1.0 = fouled core
    void setAmbient(double c) { ambient_c_ = c; }

    // Inverse of the sensor curve, so the plant can hand the controller a
    // resistance exactly as the CR0403 input would measure it.
    static double celsiusToResistance(double celsius);

private:
    double capacitance_j_per_k_;
    double temperature_c_;
    double ambient_c_;
    double ua_scale_ = 1.0;
};

// Simulated EMP WP32 node: accepts speed commands on CAN and broadcasts its
// own status, including a first-order speed lag and a current estimate.
class PumpNode {
public:
    explicit PumpNode(ICanBus &bus) : bus_(bus) {}

    void step(double dt_s, std::uint64_t now_us);

    void setSeized(bool seized) { seized_ = seized; }  // fault injection
    int rpm() const { return rpm_; }
    double dutyCommanded() const { return duty_cmd_; }

private:
    ICanBus &bus_;
    double duty_cmd_ = 0.0;
    bool enabled_ = false;
    int rpm_ = 0;
    bool seized_ = false;
    double t_since_tx_ = 0.0;
};

}  // namespace eae

#endif  // EAE_PLANT_HPP
