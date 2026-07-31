// main.cpp. Host-side runner for the cooling loop firmware.
//
// Wires the controller to a virtual CAN bus, a simulated WP32 pump node and a
// thermal plant, then runs a chosen duty cycle.  Every setpoint and gain is
// settable from the command line so the loop can be retuned without rebuilding.

#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>

#include "eae/can_bus.hpp"
#include "eae/cooling_controller.hpp"
#include "eae/j1939.hpp"
#include "eae/plant.hpp"

namespace {

struct Options {
    std::string scenario = "duty";
    double duration_s = 760.0;
    double scan_ms = 50.0;
    double ambient_c = 32.0;
    bool verbose = false;
    eae::Thresholds thresholds;
    eae::PidGains gains;
};

void usage(const char *argv0) {
    std::cout <<
        "EAE cooling loop firmware, host simulation\n\n"
        "Usage: " << argv0 << " [options]\n\n"
        "  --scenario NAME     duty | overload | leak | sensor | canloss "
        "| seized   (default duty)\n"
        "  --setpoint C        coolant setpoint            (default 45)\n"
        "  --fan-on C          fan switch-on threshold     (default 38)\n"
        "  --fan-full C        full fan threshold          (default 55)\n"
        "  --derate C          derate request threshold    (default 65)\n"
        "  --shutdown C        shutdown request threshold  (default 75)\n"
        "  --kp K --ki K --kd K                            (default 8 / 0.25 / 12)\n"
        "  --ambient C         ambient air temperature     (default 32)\n"
        "  --scan-ms MS        controller scan period      (default 50)\n"
        "  --duration S        simulated run length        (default 760)\n"
        "  --verbose           print every scan\n"
        "  --help\n";
}

bool parseArgs(int argc, char **argv, Options &o) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&](double &dst) {
            if (i + 1 >= argc) {
                std::cerr << "error: " << a << " needs a value\n";
                std::exit(2);
            }
            dst = std::atof(argv[++i]);
        };
        if (a == "--help" || a == "-h") { usage(argv[0]); return false; }
        else if (a == "--scenario") {
            if (i + 1 >= argc) { std::cerr << "error: --scenario needs a value\n"; std::exit(2); }
            o.scenario = argv[++i];
        }
        else if (a == "--setpoint") next(o.thresholds.setpoint_c);
        else if (a == "--fan-on") next(o.thresholds.fan_on_c);
        else if (a == "--fan-full") next(o.thresholds.fan_full_c);
        else if (a == "--derate") next(o.thresholds.derate_c);
        else if (a == "--shutdown") next(o.thresholds.shutdown_c);
        else if (a == "--kp") next(o.gains.kp);
        else if (a == "--ki") next(o.gains.ki);
        else if (a == "--kd") next(o.gains.kd);
        else if (a == "--ambient") next(o.ambient_c);
        else if (a == "--scan-ms") next(o.scan_ms);
        else if (a == "--duration") next(o.duration_s);
        else if (a == "--verbose") o.verbose = true;
        else {
            std::cerr << "error: unknown option " << a << "\n";
            std::exit(2);
        }
    }

    // Validate rather than trusting the operator.  A setpoint above the derate
    // threshold would command the machine straight into a power reduction.
    if (o.thresholds.setpoint_c >= o.thresholds.derate_c ||
        o.thresholds.derate_c >= o.thresholds.shutdown_c ||
        o.thresholds.fan_on_c >= o.thresholds.fan_full_c) {
        std::cerr << "error: thresholds must satisfy "
                     "fan-on < fan-full and setpoint < derate < shutdown\n";
        std::exit(2);
    }
    return true;
}

// The PowerView 450 is a receive-only node here: it does nothing but drain its
// queue and count what arrived.  Without it the bus would report a permanent
// overrun, which would be an artefact of the simulation rather than a finding.
class DisplayNode {
public:
    explicit DisplayNode(eae::ICanBus &bus) : bus_(bus) {}
    void step() {
        eae::CanFrame f;
        while (bus_.receive(eae::kAddrDisplay, f)) ++received_;
    }
    std::uint64_t received() const { return received_; }

private:
    eae::ICanBus &bus_;
    std::uint64_t received_ = 0;
};

struct Sample {
    double load_w;
    bool ignition;
    bool level_wet;
};

Sample profile(const std::string &scenario, double t) {
    if (scenario == "duty") {
        double load = 0.0;
        if (t >= 10.0 && t < 120.0) load = 3000.0;
        else if (t < 400.0 && t >= 120.0) load = 9000.0;
        else if (t < 500.0 && t >= 400.0) load = 16000.0;
        else if (t >= 500.0) load = 2000.0;
        return {load, t >= 10.0 && t < 560.0, true};
    }
    if (scenario == "overload") {
        return {t < 10.0 ? 0.0 : 26000.0, t >= 10.0, true};
    }
    if (scenario == "leak") {
        return {t < 10.0 ? 0.0 : 9000.0, t >= 10.0, t < 300.0};
    }
    // sensor, canloss and seized share a plain 9 kW load; the fault is
    // injected into the hardware model rather than the duty cycle.
    return {t < 10.0 ? 0.0 : 9000.0, t >= 10.0, true};
}

}  // namespace

int main(int argc, char **argv) {
    Options opt;
    if (!parseArgs(argc, argv, opt)) return 0;

    eae::VirtualCanBus bus;
    bus.attach(eae::kAddrController);
    bus.attach(eae::kAddrPump);
    bus.attach(eae::kAddrDisplay);

    eae::CoolingController controller(bus, opt.thresholds, opt.gains);
    eae::PumpNode pump(bus);
    DisplayNode display(bus);
    eae::ThermalPlant plant(15.0, 25.0, opt.ambient_c);
    if (opt.scenario == "overload") plant.setUaScale(0.5);

    const double dt = opt.scan_ms / 1000.0;

    std::cout << "================================================================\n"
              << "EPIROC EAE cooling loop firmware, scenario '" << opt.scenario
              << "'\n"
              << "setpoint " << opt.thresholds.setpoint_c << " C   derate "
              << opt.thresholds.derate_c << " C   shutdown "
              << opt.thresholds.shutdown_c << " C   scan " << opt.scan_ms
              << " ms\n"
              << "gains kp=" << opt.gains.kp << " ki=" << opt.gains.ki
              << " kd=" << opt.gains.kd << "\n"
              << "================================================================\n"
              << std::setw(8) << "t[s]" << std::setw(9) << "state"
              << std::setw(9) << "T[C]" << std::setw(8) << "pump%"
              << std::setw(7) << "fan%" << std::setw(8) << "rpm"
              << std::setw(8) << "load kW" << std::setw(8) << "derate"
              << std::setw(7) << "stop" << "  event\n"
              << "----------------------------------------------------------------\n";

    double t = 0.0;
    double last_print = -1e9;
    eae::State prev_state = eae::State::Init;
    eae::Fault prev_fault = eae::Fault::None;
    bool loss_injected = false;

    while (t < opt.duration_s) {
        const Sample s = profile(opt.scenario, t);
        const std::uint64_t now_us = static_cast<std::uint64_t>(t * 1e6);

        // --- fault injection into the simulated hardware --------------------
        if (opt.scenario == "canloss" && t >= 200.0 && !loss_injected) {
            bus.injectLoss(100000);   // bus wire cut from here on
            loss_injected = true;
        }
        if (opt.scenario == "seized" && t >= 200.0) pump.setSeized(true);

        eae::DiscreteInputs in;
        in.ignition = s.ignition;
        in.level_wet = s.level_wet;
        in.temperature_over_range = plant.temperature() < 20.0;
        in.temperature_resistance_ohm =
            (opt.scenario == "sensor" && t >= 200.0)
                ? 5.0     // harness shorted to chassis
                : eae::ThermalPlant::celsiusToResistance(plant.temperature());

        const eae::Commands cmd = controller.scan(in, dt, now_us);
        pump.step(dt, now_us);
        display.step();

        // The inverter and DC-DC obey the requests we send, so the heat load is
        // inside the loop rather than being an open-loop disturbance.
        double applied = s.load_w;
        if (cmd.shutdown_request) applied *= 0.05;
        else if (cmd.derate_request) applied *= 0.55;

        plant.step(applied, cmd.fan_contactor ? cmd.fan_duty : 0.0,
                   cmd.pump_enable ? cmd.pump_duty : 0.0, dt);

        std::string event;
        if (cmd.state != prev_state) {
            event = std::string("-> ") + eae::toString(cmd.state);
            prev_state = cmd.state;
        }
        if (cmd.fault != prev_fault && cmd.fault != eae::Fault::None) {
            event += std::string("  FAULT: ") + eae::toString(cmd.fault);
            prev_fault = cmd.fault;
        }

        if (opt.verbose || t - last_print >= 20.0 || !event.empty()) {
            std::cout << std::fixed << std::setprecision(1)
                      << std::setw(8) << t
                      << std::setw(9) << eae::toString(cmd.state)
                      << std::setw(9) << cmd.temperature_c
                      << std::setw(8) << (cmd.pump_enable ? cmd.pump_duty : 0.0)
                      << std::setw(7) << (cmd.fan_contactor ? cmd.fan_duty : 0.0)
                      << std::setw(8) << pump.rpm()
                      << std::setw(8) << applied / 1000.0
                      << std::setw(8) << (cmd.derate_request ? "YES" : "-")
                      << std::setw(7) << (cmd.shutdown_request ? "YES" : "-")
                      << "  " << event << "\n";
            last_print = t;
        }
        t += dt;
    }

    std::cout << "----------------------------------------------------------------\n"
              << "final state " << eae::toString(controller.stateMachine().state())
              << ", fault: " << eae::toString(controller.stateMachine().fault())
              << ", coolant " << std::setprecision(1) << plant.temperature()
              << " C\n"
              << "CAN: " << bus.framesSent() << " sent, "
              << bus.framesDropped() << " dropped, " << bus.framesOverrun()
              << " overrun, " << display.received()
              << " received by the display\n";
    return 0;
}
