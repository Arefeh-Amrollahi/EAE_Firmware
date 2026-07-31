// test_all.cpp. GTest suite for the cooling loop firmware.
//
// Each test name states the requirement being protected, not the value being
// compared, so a failure report reads as a statement about the machine rather
// than about the code.

#include <gtest/gtest.h>

#include <cmath>

#include "eae/can_bus.hpp"
#include "eae/cooling_controller.hpp"
#include "eae/j1939.hpp"
#include "eae/pid.hpp"
#include "eae/plant.hpp"
#include "eae/state_machine.hpp"

using namespace eae;

namespace {

// Cycles a state machine for a wall-clock duration at the real scan rate.
State runFor(StateMachine &sm, Conditions c, double seconds, double dt = 0.05) {
    State s = sm.state();
    for (int i = 0; i < static_cast<int>(seconds / dt); ++i) s = sm.step(c, dt);
    return s;
}

}  // namespace

// ---------------------------------------------------------------------------
// Sensor conversion
// ---------------------------------------------------------------------------
TEST(Sensor, TablePointsConvertExactly) {
    double c = 0.0;
    ASSERT_TRUE(resistanceToCelsius(660.0, false, c));
    EXPECT_NEAR(c, 60.0, 0.5);
    ASSERT_TRUE(resistanceToCelsius(175.0, false, c));
    EXPECT_NEAR(c, 100.0, 0.5);
}

TEST(Sensor, InterpolatesBetweenTablePoints) {
    double c = 0.0;
    ASSERT_TRUE(resistanceToCelsius(1200.0, false, c));
    EXPECT_GT(c, 40.0);
    EXPECT_LT(c, 50.0);
}

TEST(Sensor, ColdOverRangeIsNotAFault) {
    // Below about +20 C the NTC exceeds the CR0403's 3.6 kOhm input range.
    // That is a cold machine, not a broken sensor, and must not strand it.
    double c = 0.0;
    EXPECT_TRUE(resistanceToCelsius(50000.0, true, c));
    EXPECT_DOUBLE_EQ(c, 20.0);
}

TEST(Sensor, ShortedHarnessIsRejected) {
    double c = 0.0;
    EXPECT_FALSE(resistanceToCelsius(5.0, false, c));
}

TEST(Sensor, RoundTripsThroughThePlantInverse) {
    for (double t : {25.0, 40.0, 55.0, 70.0, 95.0}) {
        double back = 0.0;
        ASSERT_TRUE(resistanceToCelsius(ThermalPlant::celsiusToResistance(t),
                                        false, back));
        EXPECT_NEAR(back, t, 0.5) << "at " << t << " C";
    }
}

// ---------------------------------------------------------------------------
// PID
// ---------------------------------------------------------------------------
TEST(Pid, ProportionalActionRespondsToError) {
    Pid pid({4.0, 0.0, 0.0}, 0.0, 100.0);
    EXPECT_NEAR(pid.update(45.0, 50.0, 0.05), 20.0, 1e-9);
}

TEST(Pid, OutputIsClampedToTheActuatorRange) {
    Pid pid({50.0, 0.0, 0.0}, 0.0, 100.0);
    EXPECT_DOUBLE_EQ(pid.update(45.0, 90.0, 0.05), 100.0);
    EXPECT_DOUBLE_EQ(pid.update(45.0, 10.0, 0.05), 0.0);
}

TEST(Pid, IntegratorDoesNotWindUpWhileSaturated) {
    // The failure this protects against: a long saturated climb winds the
    // integrator far past the output limit, and the fan then stays at full
    // duty long after the coolant has come back down.
    Pid pid({0.0, 5.0, 0.0}, 0.0, 100.0);
    for (int i = 0; i < 2000; ++i) pid.update(45.0, 80.0, 0.05);
    EXPECT_LE(pid.integral(), 100.0);

    // Once the error reverses, the output must fall away promptly.
    double out = 100.0;
    for (int i = 0; i < 100; ++i) out = pid.update(45.0, 30.0, 0.05);
    EXPECT_LT(out, 100.0);
}

TEST(Pid, DerivativeActsOnMeasurementNotSetpoint) {
    // A setpoint change made from the display must not kick the actuator.
    Pid pid({1.0, 0.0, 100.0}, 0.0, 100.0);
    pid.update(45.0, 50.0, 0.05);
    const double after_setpoint_step = pid.update(20.0, 50.0, 0.05);
    // Only the proportional term should have moved: 1.0 * (50 - 20) = 30.
    EXPECT_NEAR(after_setpoint_step, 30.0, 1e-6);
}

TEST(Pid, ZeroTimestepDoesNotProduceInfinity) {
    Pid pid({1.0, 1.0, 1.0}, 0.0, 100.0);
    pid.update(45.0, 50.0, 0.05);
    const double out = pid.update(45.0, 60.0, 0.0);
    EXPECT_TRUE(std::isfinite(out));
}

TEST(Pid, ResetClearsAccumulatedState) {
    Pid pid({0.0, 5.0, 0.0}, 0.0, 100.0);
    for (int i = 0; i < 100; ++i) pid.update(45.0, 80.0, 0.05);
    ASSERT_GT(pid.integral(), 0.0);
    pid.reset();
    EXPECT_DOUBLE_EQ(pid.integral(), 0.0);
}

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------
TEST(StateMachine, StaysInStandbyWithTheKeyOff) {
    StateMachine sm{Thresholds{}};
    Conditions c;
    c.ignition = false;
    EXPECT_EQ(runFor(sm, c, 5.0), State::Standby);
}

TEST(StateMachine, KeyOnPrimesBeforeRunning) {
    StateMachine sm{Thresholds{}};
    Conditions c;
    c.ignition = true;
    EXPECT_EQ(runFor(sm, c, 1.0), State::Prime);
    EXPECT_EQ(runFor(sm, c, 4.0), State::Running);
}

TEST(StateMachine, KeyOffEntersPostRunNotStandby) {
    // Heat soak after key-off is a known way to cook an inverter, so the loop
    // must keep circulating rather than stopping dead.
    StateMachine sm{Thresholds{}};
    Conditions c;
    c.ignition = true;
    runFor(sm, c, 5.0);
    c.ignition = false;
    EXPECT_EQ(runFor(sm, c, 1.0), State::PostRun);
    EXPECT_EQ(runFor(sm, c, 125.0), State::Standby);
}

TEST(StateMachine, DerateEngagesAndRecoversWithHysteresis) {
    StateMachine sm{Thresholds{}};
    Conditions c;
    c.ignition = true;
    runFor(sm, c, 5.0);

    c.temperature_c = 66.0;
    EXPECT_EQ(runFor(sm, c, 0.5), State::Derate);

    // Just below the threshold is not enough: recovery needs the hysteresis
    // band, otherwise the inverter is cycled in and out of derate.
    c.temperature_c = 64.0;
    EXPECT_EQ(runFor(sm, c, 0.5), State::Derate);

    c.temperature_c = 60.0;
    EXPECT_EQ(runFor(sm, c, 0.5), State::Running);
}

TEST(StateMachine, SustainedOverTemperatureLatchesAFault) {
    StateMachine sm{Thresholds{}};
    Conditions c;
    c.ignition = true;
    runFor(sm, c, 5.0);
    c.temperature_c = 80.0;

    // Brief excursions must not trip: the confirm time exists to reject noise.
    EXPECT_NE(runFor(sm, c, 2.0), State::Fault);
    EXPECT_EQ(runFor(sm, c, 5.0), State::Fault);
    EXPECT_EQ(sm.fault(), Fault::OverTemperature);
}

TEST(StateMachine, CoolantSloshDoesNotTripButRealLossDoes) {
    StateMachine sm{Thresholds{}};
    Conditions c;
    c.ignition = true;
    runFor(sm, c, 5.0);
    c.level_ok = false;
    EXPECT_NE(runFor(sm, c, 10.0), State::Fault);
    EXPECT_EQ(runFor(sm, c, 25.0), State::Fault);
    EXPECT_EQ(sm.fault(), Fault::CoolantLevel);
}

TEST(StateMachine, InvalidSensorFaultsImmediately) {
    StateMachine sm{Thresholds{}};
    Conditions c;
    c.ignition = true;
    runFor(sm, c, 5.0);
    c.temperature_valid = false;
    EXPECT_EQ(runFor(sm, c, 0.2), State::Fault);
    EXPECT_EQ(sm.fault(), Fault::TemperatureSensor);
}

TEST(StateMachine, FaultsLatchUntilTheKeyIsCycled) {
    StateMachine sm{Thresholds{}};
    Conditions c;
    c.ignition = true;
    runFor(sm, c, 5.0);
    c.temperature_valid = false;
    runFor(sm, c, 0.5);
    ASSERT_EQ(sm.state(), State::Fault);

    c.temperature_valid = true;   // condition clears on its own
    EXPECT_EQ(runFor(sm, c, 5.0), State::Fault);

    c.ignition = false;           // key cycle
    runFor(sm, c, 0.5);
    c.ignition = true;
    const State s = runFor(sm, c, 0.5);
    EXPECT_TRUE(s == State::Prime || s == State::Running);
    EXPECT_EQ(sm.fault(), Fault::None);
}

TEST(StateMachine, StalledPumpIsReportedAsTheRootCause) {
    // A stalled pump and an over-temperature are both true at once, because
    // one causes the other.  The technician needs the cause, so the stall is
    // what latches, but see the controller test below: the machine must
    // still be shut down on temperature regardless of which code latched.
    StateMachine sm{Thresholds{}};
    Conditions c;
    c.ignition = true;
    runFor(sm, c, 5.0);
    c.temperature_c = 90.0;
    c.pump_commanded = true;
    c.pump_turning = false;
    runFor(sm, c, 6.0);
    EXPECT_EQ(sm.fault(), Fault::PumpStall);
    EXPECT_TRUE(sm.overTemperatureConfirmed());
}

// ---------------------------------------------------------------------------
// J1939 encoding
// ---------------------------------------------------------------------------
TEST(J1939, IdentifierPacksPriorityPgnAndSource) {
    const std::uint32_t id = j1939::makeId(6, j1939::kPgnPumpCommand,
                                           kAddrController);
    EXPECT_EQ(j1939::pgnOf(id), j1939::kPgnPumpCommand);
    EXPECT_EQ(j1939::sourceOf(id), kAddrController);
    EXPECT_EQ((id >> 26) & 0x07u, 6u);
}

TEST(J1939, TemperatureRoundTripsWithinOneBit) {
    for (double t : {-40.0, 0.0, 25.0, 90.0, 150.0}) {
        double back = 0.0;
        ASSERT_TRUE(j1939::decodeTemperature(j1939::encodeTemperature(t), back));
        EXPECT_NEAR(back, t, 1.0) << "at " << t;
    }
}

TEST(J1939, NotAvailableTemperatureIsRejected) {
    double back = 0.0;
    EXPECT_FALSE(j1939::decodeTemperature(0xFF, back));
}

TEST(J1939, PercentAndRpmRoundTrip) {
    EXPECT_NEAR(j1939::decodePercent(j1939::encodePercent(63.2)), 63.2, 0.4);
    std::uint8_t lo = 0, hi = 0;
    j1939::encodeRpm(3400, lo, hi);
    EXPECT_EQ(j1939::decodeRpm(lo, hi), 3400);
}

// ---------------------------------------------------------------------------
// Virtual CAN bus
// ---------------------------------------------------------------------------
TEST(CanBus, FramesReachOtherNodesButNotTheSender) {
    VirtualCanBus bus;
    bus.attach(kAddrController);
    bus.attach(kAddrPump);

    bus.send(j1939::makePumpCommand(75.0, true));

    CanFrame f;
    EXPECT_TRUE(bus.receive(kAddrPump, f));
    EXPECT_EQ(j1939::pgnOf(f.id), j1939::kPgnPumpCommand);
    EXPECT_NEAR(j1939::decodePercent(f.data[1]), 75.0, 0.4);
    EXPECT_FALSE(bus.receive(kAddrController, f));
}

TEST(CanBus, QueueOverrunDiscardsTheOldestFrame) {
    VirtualCanBus bus(4);
    bus.attach(kAddrController);
    bus.attach(kAddrPump);
    for (int i = 0; i < 10; ++i) bus.send(j1939::makePumpCommand(i * 10.0, true));
    EXPECT_GT(bus.framesOverrun(), 0u);

    // What survives must be the freshest data, not the stalest.
    CanFrame f;
    ASSERT_TRUE(bus.receive(kAddrPump, f));
    EXPECT_NEAR(j1939::decodePercent(f.data[1]), 60.0, 0.5);
}

TEST(CanBus, InjectedLossIsInvisibleToTheSender) {
    VirtualCanBus bus;
    bus.attach(kAddrController);
    bus.attach(kAddrPump);
    bus.injectLoss(3);
    for (int i = 0; i < 3; ++i)
        EXPECT_TRUE(bus.send(j1939::makePumpCommand(50.0, true)));
    CanFrame f;
    EXPECT_FALSE(bus.receive(kAddrPump, f));
    EXPECT_EQ(bus.framesDropped(), 3u);
}

// ---------------------------------------------------------------------------
// Controller integration
// ---------------------------------------------------------------------------
TEST(Controller, PrimesAtFullFlowThenModulates) {
    VirtualCanBus bus;
    bus.attach(kAddrController);
    bus.attach(kAddrPump);
    CoolingController ctl(bus, Thresholds{}, PidGains{});
    PumpNode pump(bus);

    DiscreteInputs in;
    in.ignition = true;
    in.temperature_resistance_ohm = ThermalPlant::celsiusToResistance(30.0);

    Commands c;
    for (int i = 0; i < 20; ++i) {
        c = ctl.scan(in, 0.05, i * 50000);
        pump.step(0.05, i * 50000);
    }
    EXPECT_EQ(c.state, State::Prime);
    EXPECT_DOUBLE_EQ(c.pump_duty, 100.0);

    for (int i = 20; i < 200; ++i) {
        c = ctl.scan(in, 0.05, i * 50000);
        pump.step(0.05, i * 50000);
    }
    EXPECT_EQ(c.state, State::Running);
    EXPECT_DOUBLE_EQ(c.fan_duty, 0.0) << "fan must stay off below the threshold";
    EXPECT_NEAR(c.pump_duty, 40.0, 0.01);
}

TEST(Controller, ColdStartLimitsThePumpInsteadOfPrimingAtFullFlow) {
    // The sensor layer already reports a cold machine when the NTC over-ranges.
    // This test holds the loop to acting on that rather than ignoring it.
    VirtualCanBus bus;
    bus.attach(kAddrController);
    bus.attach(kAddrPump);
    CoolingController ctl(bus, Thresholds{}, PidGains{});
    PumpNode pump(bus);

    DiscreteInputs in;
    in.ignition = true;
    in.temperature_resistance_ohm = 50000.0;
    in.temperature_over_range = true;

    Commands c;
    for (int i = 0; i < 20; ++i) {
        c = ctl.scan(in, 0.05, i * 50000);
        pump.step(0.05, i * 50000);
    }
    EXPECT_EQ(c.state, State::Prime);
    EXPECT_LT(c.pump_duty, 100.0) << "cold prime must not command full duty";
    EXPECT_GE(c.pump_duty, Thresholds{}.pump_min_duty)
        << "the pump must still turn";
}

TEST(Controller, WarmStartStillPrimesAtFullFlow) {
    VirtualCanBus bus;
    bus.attach(kAddrController);
    bus.attach(kAddrPump);
    CoolingController ctl(bus, Thresholds{}, PidGains{});
    PumpNode pump(bus);

    DiscreteInputs in;
    in.ignition = true;
    in.temperature_resistance_ohm = ThermalPlant::celsiusToResistance(30.0);

    Commands c;
    for (int i = 0; i < 20; ++i) {
        c = ctl.scan(in, 0.05, i * 50000);
        pump.step(0.05, i * 50000);
    }
    EXPECT_DOUBLE_EQ(c.pump_duty, 100.0);
}

TEST(Controller, HotCoolantDrivesBothActuatorsToFull) {
    VirtualCanBus bus;
    bus.attach(kAddrController);
    bus.attach(kAddrPump);
    CoolingController ctl(bus, Thresholds{}, PidGains{});
    PumpNode pump(bus);

    DiscreteInputs in;
    in.ignition = true;
    in.temperature_resistance_ohm = ThermalPlant::celsiusToResistance(58.0);

    Commands c;
    for (int i = 0; i < 200; ++i) {
        c = ctl.scan(in, 0.05, i * 50000);
        pump.step(0.05, i * 50000);
    }
    EXPECT_DOUBLE_EQ(c.fan_duty, 100.0);
    EXPECT_DOUBLE_EQ(c.pump_duty, 100.0);
}

TEST(Controller, LosingPumpStatusFramesRaisesACommsFault) {
    // This is the test that justifies the whole virtual bus: the timeout path
    // cannot be proven any other way without cutting a real wire.
    VirtualCanBus bus;
    bus.attach(kAddrController);
    bus.attach(kAddrPump);
    CoolingController ctl(bus, Thresholds{}, PidGains{});
    PumpNode pump(bus);

    DiscreteInputs in;
    in.ignition = true;
    in.temperature_resistance_ohm = ThermalPlant::celsiusToResistance(40.0);

    Commands c;
    for (int i = 0; i < 200; ++i) {
        c = ctl.scan(in, 0.05, i * 50000);
        pump.step(0.05, i * 50000);
    }
    ASSERT_EQ(c.state, State::Running);

    bus.injectLoss(1000000);   // bus wire cut
    for (int i = 200; i < 260; ++i) {
        c = ctl.scan(in, 0.05, i * 50000);
        pump.step(0.05, i * 50000);
    }
    EXPECT_EQ(c.fault, Fault::PumpComms);
    EXPECT_TRUE(c.derate_request);
}

TEST(Controller, LowLevelStopsThePumpRatherThanRunningItDry) {
    VirtualCanBus bus;
    bus.attach(kAddrController);
    bus.attach(kAddrPump);
    CoolingController ctl(bus, Thresholds{}, PidGains{});
    PumpNode pump(bus);

    DiscreteInputs in;
    in.ignition = true;
    in.temperature_resistance_ohm = ThermalPlant::celsiusToResistance(40.0);

    Commands c;
    for (int i = 0; i < 200; ++i) {
        c = ctl.scan(in, 0.05, i * 50000);
        pump.step(0.05, i * 50000);
    }
    in.level_wet = false;
    for (int i = 200; i < 1000; ++i) {
        c = ctl.scan(in, 0.05, i * 50000);
        pump.step(0.05, i * 50000);
    }
    EXPECT_EQ(c.fault, Fault::CoolantLevel);
    EXPECT_FALSE(c.pump_enable);
    EXPECT_TRUE(c.shutdown_request);
    EXPECT_DOUBLE_EQ(c.fan_duty, 100.0) << "the fan can still remove heat";
}

TEST(Controller, OverTemperatureShutsDownEvenWhenAnotherFaultLatchedFirst) {
    // The failure this guards against: the pump stalls, the loop cooks, the
    // stall latches first, and the inverter is only asked to derate while the
    // coolant climbs past 90 C.
    VirtualCanBus bus;
    bus.attach(kAddrController);
    bus.attach(kAddrPump);
    CoolingController ctl(bus, Thresholds{}, PidGains{});
    PumpNode pump(bus);
    pump.setSeized(true);

    DiscreteInputs in;
    in.ignition = true;
    in.temperature_resistance_ohm = ThermalPlant::celsiusToResistance(90.0);

    Commands c;
    for (int i = 0; i < 400; ++i) {
        c = ctl.scan(in, 0.05, i * 50000);
        pump.step(0.05, i * 50000);
    }
    EXPECT_EQ(c.fault, Fault::PumpStall) << "root cause is reported";
    EXPECT_TRUE(c.shutdown_request) << "but the machine is still shut down";
}

TEST(Controller, UndervoltageShedsTheFanFirst) {
    VirtualCanBus bus;
    bus.attach(kAddrController);
    bus.attach(kAddrPump);
    CoolingController ctl(bus, Thresholds{}, PidGains{});
    PumpNode pump(bus);

    DiscreteInputs in;
    in.ignition = true;
    in.temperature_resistance_ohm = ThermalPlant::celsiusToResistance(58.0);
    Commands c;
    for (int i = 0; i < 200; ++i) {
        c = ctl.scan(in, 0.05, i * 50000);
        pump.step(0.05, i * 50000);
    }
    ASSERT_DOUBLE_EQ(c.fan_duty, 100.0);

    in.supply_voltage = 16.0;
    c = ctl.scan(in, 0.05, 200 * 50000);
    EXPECT_DOUBLE_EQ(c.fan_duty, 0.0);
    EXPECT_FALSE(c.fan_contactor);
    EXPECT_TRUE(c.pump_enable) << "flow matters more than air when volts are low";
}

TEST(Controller, ClosedLoopSettlesNearTheSetpointUnderSteadyLoad) {
    // The tuning test: with a constant heat load the loop must reach a steady
    // temperature inside the control band, not oscillate or run away.
    VirtualCanBus bus;
    bus.attach(kAddrController);
    bus.attach(kAddrPump);
    CoolingController ctl(bus, Thresholds{}, PidGains{});
    PumpNode pump(bus);
    ThermalPlant plant(15.0, 25.0, 32.0);

    DiscreteInputs in;
    in.ignition = true;

    for (int i = 0; i < 24000; ++i) {   // 1200 s at 50 ms
        in.temperature_over_range = plant.temperature() < 20.0;
        in.temperature_resistance_ohm =
            ThermalPlant::celsiusToResistance(plant.temperature());
        const Commands c = ctl.scan(in, 0.05, i * 50000ull);
        pump.step(0.05, i * 50000ull);
        plant.step(9000.0, c.fan_contactor ? c.fan_duty : 0.0,
                   c.pump_enable ? c.pump_duty : 0.0, 0.05);
    }
    EXPECT_GT(plant.temperature(), 35.0);
    EXPECT_LT(plant.temperature(), 60.0)
        << "steady state must sit below the derate threshold";
    EXPECT_EQ(ctl.stateMachine().state(), State::Running);
}
