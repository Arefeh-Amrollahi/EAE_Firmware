#include "eae/plant.hpp"

#include <algorithm>
#include <cmath>

#include "eae/j1939.hpp"

namespace eae {
namespace {

struct TablePoint {
    double celsius;
    double ohms;
};

constexpr TablePoint kNtcTable[] = {
    {-20.0, 28146.0}, {-10.0, 15873.0}, {0.0, 9256.0},  {10.0, 5572.0},
    {20.0, 3457.0},   {30.0, 2830.0},   {40.0, 1443.0}, {50.0, 992.0},
    {60.0, 660.0},    {70.0, 475.0},    {80.0, 329.0},  {90.0, 244.0},
    {100.0, 175.0},   {110.0, 134.0},   {120.0, 99.0},  {140.0, 60.0},
    {160.0, 47.0},
};
constexpr int kNtcPoints = sizeof(kNtcTable) / sizeof(kNtcTable[0]);

}  // namespace

ThermalPlant::ThermalPlant(double coolant_litres, double t_initial_c,
                           double t_ambient_c)
    // 50 % ethylene glycol: roughly 3.6 kJ/kg.K, density near 1.0 kg/L.  The
    // cold plates and hoses add mass, so this is a lower bound on the real
    // capacitance and therefore a faster, more pessimistic plant to control.
    : capacitance_j_per_k_(coolant_litres * 1000.0 * 3.6),
      temperature_c_(t_initial_c),
      ambient_c_(t_ambient_c) {}

double ThermalPlant::step(double heat_load_w, double fan_duty,
                          double pump_duty, double dt_s) {
    const double flow = std::clamp(pump_duty / 100.0, 0.0, 1.0);

    double ua;
    if (flow < 0.05) {
        // No flow means no transport: the sensor decouples from the heat
        // source entirely.  Modelling that explicitly is what makes a stalled
        // pump test meaningful rather than merely slower.
        ua = 2.0;
    } else {
        constexpr double kUaNatural = 25.0;  // radiator with no fan
        constexpr double kUaForced = 320.0;  // radiator at full fan
        ua = (kUaNatural + kUaForced * std::clamp(fan_duty / 100.0, 0.0, 1.0)) *
             flow * ua_scale_;
    }

    const double dT = (heat_load_w * flow - ua * (temperature_c_ - ambient_c_)) /
                      capacitance_j_per_k_;
    temperature_c_ += dT * dt_s;
    return temperature_c_;
}

double ThermalPlant::celsiusToResistance(double celsius) {
    if (celsius <= kNtcTable[0].celsius) return kNtcTable[0].ohms;
    if (celsius >= kNtcTable[kNtcPoints - 1].celsius)
        return kNtcTable[kNtcPoints - 1].ohms;
    for (int i = 0; i < kNtcPoints - 1; ++i) {
        if (celsius >= kNtcTable[i].celsius &&
            celsius <= kNtcTable[i + 1].celsius) {
            const double frac = (celsius - kNtcTable[i].celsius) /
                                (kNtcTable[i + 1].celsius - kNtcTable[i].celsius);
            return std::exp(std::log(kNtcTable[i].ohms) +
                            frac * (std::log(kNtcTable[i + 1].ohms) -
                                    std::log(kNtcTable[i].ohms)));
        }
    }
    return kNtcTable[kNtcPoints - 1].ohms;
}

void PumpNode::step(double dt_s, std::uint64_t now_us) {
    CanFrame f;
    while (bus_.receive(kAddrPump, f)) {
        if (j1939::pgnOf(f.id) == j1939::kPgnPumpCommand) {
            enabled_ = (f.data[0] != 0);
            duty_cmd_ = j1939::decodePercent(f.data[1]);
        }
    }

    // First-order speed lag.  The WP32 is a brushless unit with its own
    // controller, so it ramps rather than stepping; giving the stall detector
    // an instantaneous response would let it pass a test it should not.
    const int target = (enabled_ && !seized_)
                           ? static_cast<int>(duty_cmd_ * 45.0)
                           : 0;
    rpm_ += static_cast<int>((target - rpm_) * std::min(1.0, dt_s * 4.0));

    // Roughly cubic with speed, capped at the datasheet 15 A limit for the
    // 24 V pump.
    const double frac = static_cast<double>(rpm_) / 4500.0;
    const double current = std::min(15.0, 15.0 * frac * frac * frac);

    t_since_tx_ += dt_s;
    if (t_since_tx_ >= 0.100) {
        bus_.advance(now_us);
        bus_.send(j1939::makePumpStatus(rpm_, current, seized_));
        t_since_tx_ = 0.0;
    }
}

}  // namespace eae
