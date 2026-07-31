#include "eae/cooling_controller.hpp"

#include <algorithm>
#include <cmath>

#include "eae/j1939.hpp"

namespace eae {
namespace {

// H-WTMS unloaded resistance vs. temperature, transcribed from the datasheet
// drawing (Rev 3, 10/12/2018), coldest first.
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

// CR0403 resistance input range is 0.016...3.6 kOhm (datasheet p.3).
constexpr double kResistanceMaxOhm = 3600.0;
constexpr double kShortOhm = 20.0;
constexpr double kColdClampC = 20.0;

// Status transmit periods.
constexpr double kStatusPeriodS = 0.100;
constexpr double kCommandPeriodS = 0.050;

}  // namespace

bool resistanceToCelsius(double ohms, bool over_range, double &celsius) {
    if (over_range || ohms > kResistanceMaxOhm) {
        // Colder than about +20 C.  Not a fault: there is simply no cooling
        // demand down there, so clamping is safe and conservative.  Treating a
        // winter cold-soak as a sensor failure would strand the machine.
        celsius = kColdClampC;
        return true;
    }
    if (ohms < kShortOhm) {
        return false;  // below the sensor's hottest point: a short, not a read
    }

    // Interpolate in log-resistance, which tracks an NTC far better than a
    // linear fit on raw ohms (log R varies as 1/T for a Beta model).  Residual
    // error is well under the CR0403's own +/-3 % input accuracy.
    for (int i = 0; i < kNtcPoints - 1; ++i) {
        const double r_lo = kNtcTable[i].ohms;
        const double r_hi = kNtcTable[i + 1].ohms;
        if (ohms <= r_lo && ohms >= r_hi) {
            const double frac =
                (std::log(r_lo) - std::log(ohms)) / (std::log(r_lo) - std::log(r_hi));
            celsius = kNtcTable[i].celsius +
                      frac * (kNtcTable[i + 1].celsius - kNtcTable[i].celsius);
            return true;
        }
    }

    // Hotter than the last table entry: extrapolate off the final segment
    // rather than declaring a fault, so a real over-temperature is still seen
    // and acted upon.
    const double r_lo = kNtcTable[kNtcPoints - 2].ohms;
    const double r_hi = kNtcTable[kNtcPoints - 1].ohms;
    const double frac =
        (std::log(r_lo) - std::log(ohms)) / (std::log(r_lo) - std::log(r_hi));
    celsius = kNtcTable[kNtcPoints - 2].celsius +
              frac * (kNtcTable[kNtcPoints - 1].celsius -
                      kNtcTable[kNtcPoints - 2].celsius);
    return true;
}

CoolingController::CoolingController(ICanBus &bus, const Thresholds &th,
                                     const PidGains &g)
    : bus_(bus), sm_(th), pid_(g, 0.0, 100.0) {}

void CoolingController::serviceReceive(std::uint64_t /*now_us*/) {
    CanFrame f;
    // Drain the queue every scan.  Reading only one frame per scan would let a
    // burst build a backlog that is then acted on stale.
    while (bus_.receive(kAddrController, f)) {
        switch (j1939::pgnOf(f.id)) {
            case j1939::kPgnPumpStatus: {
                const int rpm = j1939::decodeRpm(f.data[0], f.data[1]);
                if (rpm >= 0) pump_rpm_ = rpm;
                pump_current_a_ = static_cast<double>(f.data[2]) * 0.1;
                pump_reported_fault_ = (f.data[3] != 0);
                t_since_pump_status_ = 0.0;
                break;
            }
            default:
                break;  // not ours; a real node would also handle TP and DM1
        }
    }
}

double CoolingController::pumpDutyFor(double temperature_c) const {
    const Thresholds &th = sm_.thresholds();
    if (temperature_c <= th.fan_on_c) return th.pump_min_duty;
    const double span = th.fan_full_c - th.fan_on_c;
    double frac = span > 0.0 ? (temperature_c - th.fan_on_c) / span : 1.0;
    frac = std::clamp(frac, 0.0, 1.0);
    return th.pump_min_duty + frac * (100.0 - th.pump_min_duty);
}

bool CoolingController::isCold(double temperature_c, bool valid) const {
    return valid && temperature_c <= sm_.thresholds().cold_clamp_c;
}

double CoolingController::pumpColdLimit(double dt_s) {
    // Ramped rather than stepped, because the load the pump sees is the
    // viscosity of the fluid and not a fixed impedance.  Twenty seconds lets
    // the motor work the cold coolant through the loop with a lower peak
    // current, and lets shear heating begin to thin it.
    const Thresholds &th = sm_.thresholds();
    t_cold_run_ += dt_s;
    const double frac = std::clamp(t_cold_run_ / th.cold_ramp_s, 0.0, 1.0);
    return th.pump_min_duty + frac * (th.pump_cold_limit - th.pump_min_duty);
}

double CoolingController::fanDutyFor(double temperature_c, double dt_s) {
    const Thresholds &th = sm_.thresholds();

    // Hysteresis plus a minimum on-time.  Either alone is not enough: the
    // hysteresis stops chatter around the threshold, the minimum on-time stops
    // short-cycling when a transient load pushes the temperature across it.
    if (temperature_c >= th.fan_on_c) {
        fan_latched_on_ = true;
    } else if (temperature_c < th.fan_on_c - th.hysteresis_c &&
               t_fan_on_ >= th.fan_min_on_s) {
        fan_latched_on_ = false;
    }

    if (!fan_latched_on_) {
        t_fan_on_ = 0.0;
        pid_.reset();
        return 0.0;
    }
    t_fan_on_ += dt_s;

    const double span = th.fan_full_c - th.fan_on_c;
    double frac = span > 0.0 ? (temperature_c - th.fan_on_c) / span : 1.0;
    frac = std::clamp(frac, 0.0, 1.0);
    const double open_loop = th.fan_min_duty + frac * (100.0 - th.fan_min_duty);

    const double closed_loop = pid_.update(th.setpoint_c, temperature_c, dt_s);

    // Open-loop map guarantees an immediate sensible response the moment the
    // threshold is crossed; the PID trims on top and does the steady-state
    // work.  Waiting for the integrator alone would mean a fan that reacts in
    // ten seconds rather than one scan.
    return std::clamp(std::max(open_loop, closed_loop), th.fan_min_duty, 100.0);
}

void CoolingController::transmit(const Commands &c, std::uint64_t now_us,
                                 double dt_s) {
    t_since_tx_command_ += dt_s;
    t_since_tx_status_ += dt_s;
    bus_.advance(now_us);

    if (t_since_tx_command_ >= kCommandPeriodS) {
        bus_.send(j1939::makePumpCommand(c.pump_duty, c.pump_enable));
        t_since_tx_command_ = 0.0;
    }

    if (t_since_tx_status_ >= kStatusPeriodS) {
        bus_.send(j1939::makeCoolingStatus(
            c.temperature_c, c.fan_duty, static_cast<std::uint8_t>(c.state),
            static_cast<std::uint8_t>(c.fault)));
        bus_.send(j1939::makeEngineTemp1(c.temperature_c));
        t_since_tx_status_ = 0.0;
    }

    // Power requests are event-driven, not periodic: they are sent every scan
    // while asserted so a single lost frame cannot leave the inverter running
    // at full power into a hot loop.
    if (c.derate_request || c.shutdown_request) {
        bus_.send(j1939::makePowerRequest(c.derate_request, c.shutdown_request,
                                          c.shutdown_request ? 0.0 : 55.0));
    }
}

Commands CoolingController::scan(const DiscreteInputs &in, double dt_s,
                                 std::uint64_t now_us) {
    Commands out;
    t_since_pump_status_ += dt_s;
    serviceReceive(now_us);

    // --- 1. Condition the hard-wired inputs --------------------------------
    double temp_c = 0.0;
    const bool temp_valid = resistanceToCelsius(
        in.temperature_resistance_ohm, in.temperature_over_range, temp_c);

    Conditions cond;
    cond.ignition = in.ignition;
    cond.temperature_valid = temp_valid;
    cond.temperature_c = temp_c;
    cond.level_ok = in.level_wet;
    cond.pump_comms_ok = t_since_pump_status_ <= sm_.thresholds().can_timeout_s;
    cond.pump_commanded = last_pump_enable_;   // issued on the previous scan
    cond.pump_turning = pump_rpm_ > 0;
    cond.fan_circuit_ok = in.fan_diagnosis_ok;

    // --- 2. Advance the mode logic -----------------------------------------
    const State s = sm_.step(cond, dt_s);
    if (s == State::Standby || s == State::Fault) pid_.reset();

    out.state = s;
    out.fault = sm_.fault();
    out.temperature_c = temp_c;
    out.temperature_valid = temp_valid;

    // --- 3. Output stage ----------------------------------------------------
    const Thresholds &th = sm_.thresholds();
    switch (s) {
        case State::Init:
        case State::Standby:
            break;  // everything already parked

        case State::Prime:
            out.pump_enable = true;
            out.fan_contactor = true;   // energised, held at zero duty
            if (isCold(temp_c, temp_valid)) {
                // Cold start: ramp instead of purging at full flow.  Air still
                // leaves the loop, only more slowly, and the deaeration line
                // does the rest during normal running.
                out.pump_duty = pumpColdLimit(dt_s);
            } else {
                out.pump_duty = 100.0;  // full flow purges air out of the core
            }
            break;

        case State::Running:
        case State::Derate:
            out.pump_enable = true;
            out.fan_contactor = true;
            if (temp_valid) {
                out.pump_duty = pumpDutyFor(temp_c);
                if (isCold(temp_c, temp_valid)) {
                    out.pump_duty = std::min(out.pump_duty, pumpColdLimit(dt_s));
                } else {
                    t_cold_run_ = 0.0;
                }
                out.fan_duty = fanDutyFor(temp_c, dt_s);
                out.lamp_warning = temp_c >= th.warn_c;
            } else {
                out.pump_duty = 100.0;
                out.fan_duty = 100.0;
            }
            if (s == State::Derate) {
                out.pump_duty = 100.0;
                out.fan_duty = 100.0;
                out.derate_request = true;
                out.lamp_warning = true;
            }
            break;

        case State::PostRun:
            // Heat soak into a stopped loop with no airflow is a well known way
            // to cook an inverter.  Keep the coolant moving after key-off.
            out.pump_enable = true;
            out.pump_duty = 60.0;
            out.fan_contactor = true;
            if (temp_valid) out.fan_duty = fanDutyFor(temp_c, dt_s);
            break;

        case State::Fault:
            out.lamp_fault = true;
            out.derate_request = true;
            out.fan_contactor = true;
            out.fan_duty = 100.0;
            if (sm_.fault() == Fault::CoolantLevel) {
                // No coolant means no flow.  Running the WP32 dry destroys its
                // seal and bearing, so stop it and ask for a full shutdown.
                out.pump_enable = false;
                out.shutdown_request = true;
            } else if (sm_.fault() == Fault::OverTemperature) {
                out.pump_enable = true;
                out.pump_duty = 100.0;
                out.shutdown_request = true;
            } else {
                // Sensor, comms or fan faults: keep removing heat by whatever
                // means remain and derate rather than shut down, so the machine
                // can still be driven off the working face.
                out.pump_enable = true;
                out.pump_duty = 100.0;
            }
            break;
    }

    // An over-temperature forces a shutdown request no matter which fault code
    // happened to latch first.  A stalled pump is the root cause and is reported
    // as such, but on its own it would mask the consequence and the
    // inverter would keep making heat into a loop that cannot carry it away.
    if (temp_valid && sm_.overTemperatureConfirmed()) {
        out.shutdown_request = true;
        out.derate_request = true;
        out.lamp_warning = true;
        out.fan_contactor = true;
        out.fan_duty = 100.0;
    }

    // Below 18 V the SPAL drive is outside its rated window and the LMC100 is
    // unpowered, so shed the largest parasitic load first.
    if (in.supply_voltage < 18.0) {
        out.fan_duty = 0.0;
        out.fan_contactor = false;
        out.lamp_warning = true;
    }

    // --- 4. Publish ---------------------------------------------------------
    last_pump_enable_ = out.pump_enable;
    transmit(out, now_us, dt_s);
    return out;
}

}  // namespace eae
