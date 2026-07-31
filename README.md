# EAE_Firmware

Cooling loop firmware for the inverter and DC-DC circuit of the EAE Electrical
and Controls Challenge, Section 7.1. C++17, CMake, GoogleTest.

The controller runs against a virtual CAN bus, a simulated EMP WP32 pump node
and a lumped thermal model, so the whole loop can be built and exercised on a
development host with no hardware attached. The control code under test is the
code that would ship. Only the plant and the bus backing are swapped.

## Quick start

```bash
./run.sh test                                   # build and run 36 GoogleTest cases via CTest
./run.sh run --scenario duty                    # normal shift
./run.sh run --scenario overload --verbose      # blocked radiator, derate then shutdown
./run.sh all                                    # tests plus every scenario
./run.sh run --setpoint 42 --derate 60 --kp 12  # retune at runtime, no rebuild
```

Requires a C++17 compiler, CMake 3.16 or later, and network access on the first
configure, since GoogleTest is fetched then and is not vendored. Passing
`-DEAE_BUILD_TESTS=OFF` gives an offline build of the application alone.

## What it looks like when it runs

The `overload` scenario models a radiator core fouled to half its effectiveness
under a sustained 26 kW load, which is more heat than the loop can reject. The
controller ramps, requests a derate, and finally requests a shutdown.

```
================================================================
EPIROC EAE cooling loop firmware, scenario 'overload'
setpoint 45 C   derate 65 C   shutdown 75 C   scan 50 ms
gains kp=8 ki=0.25 kd=12
================================================================
    t[s]    state     T[C]   pump%   fan%     rpm load kW  derate   stop  event
----------------------------------------------------------------
     0.0  STANDBY     25.0     0.0    0.0       0     0.0       -      -  -> STANDBY
    10.0    PRIME     25.0   100.0    0.0     900    26.0       -      -  -> PRIME
    13.1  RUNNING     26.5    40.0    0.0    3957    26.0       -      -  -> RUNNING
    53.1  RUNNING     34.2    40.0    0.0    1804    26.0       -      -
    73.1  RUNNING     38.0    40.2   20.2    1804    26.0       -      -
    93.2  RUNNING     42.6    56.1   41.5    2513    26.0       -      -
   113.2  RUNNING     48.6    77.4   69.9    3472    26.0       -      -
   133.2  RUNNING     56.2   100.0  100.0    4496    26.0       -      -
   155.8   DERATE     65.0   100.0  100.0    4496    14.3     YES      -  -> DERATE
   215.8   DERATE     73.7   100.0  100.0    4496    14.3     YES      -
   230.7    FAULT     75.6   100.0  100.0    4496     1.3     YES    YES  -> FAULT: coolant over-temperature
```

Reading across: the pump holds its 40 % floor while the coolant is cold, the fan
starts at 38 C and reaches full duty by 55 C, the derate request at 65 C cuts
the applied load from 26 kW to 14.3 kW, and the shutdown request at 75 C drops
it to standby losses. The loop then cools back down, which is the behaviour the
integration test checks.


## Layout

```
include/eae/    can_bus.hpp  j1939.hpp  pid.hpp  state_machine.hpp
                cooling_controller.hpp  plant.hpp
src/            can_bus.cpp  pid.cpp  state_machine.cpp
                cooling_controller.cpp  plant.cpp  main.cpp
tests/          test_all.cpp
CMakeLists.txt  run.sh  .gitignore
```

## How the brief is met

| Requirement | Where |
|---|---|
| Simulate sending and receiving over CANBUS | `can_bus.*`, `j1939.*`. 29 bit J1939 identifiers, SPN 110 coolant temperature, proprietary PGNs for pump command and status and for power requests, per node receive queues, injectable frame loss |
| Use a PID loop | `pid.*`. Clamped integrator, derivative on measurement, guard against a zero timestep |
| Create a state machine | `state_machine.*`. 7 states, 6 fault codes, deterministic fault priority |
| Pass command line arguments for setpoints | `main.cpp`. `--setpoint --derate --shutdown --fan-on --fan-full --kp --ki --kd --ambient --scan-ms --duration --scenario`, all validated |
| External dependencies managed by CMake | `FetchContent` on GoogleTest, pinned to tag `v1.15.2` |
| Build on Linux, shell script to launch with runtime params | `run.sh build|test|run|all|clean`, forwards arguments verbatim |
| GTest unit testing | `tests/test_all.cpp`. 34 cases across sensor, PID, state machine, J1939, bus and integration |
| Do not ship dependencies | `.gitignore` excludes `build/` and `_deps/`. Nothing is vendored |

Built with `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion`,
zero warnings.

## Scenarios

| Name | What it exercises |
|---|---|
| `duty` | Prime, modulation across a shift, key off post-run |
| `overload` | Radiator core at half effectiveness under full load. Derate, then over temperature shutdown, then recovery once the inverter obeys |
| `leak` | Level switch goes dry. Debounce, alarm, trip, pump stopped |
| `sensor` | Temperature harness shorted. Fail safe to full cooling, derate instead of shutdown |
| `canloss` | Bus wire cut. Pump status watchdog expires |
| `seized` | Pump commanded but not turning. Stall detected from the command, not from the mode |

## Two design decisions worth reading the code for

**Fault priority reports causes, not symptoms.** A seized pump makes the loop
overheat. If the over temperature latched first, a technician would be sent to
look at the radiator when the pump is what failed. The priority is therefore
sensor, level, comms, stall, fan, over temperature, and it is fixed instead of
depending on which timer expires first.

That raises an obvious question: if the stall latches, does the machine still
get shut down? It has to. The shutdown request is driven by
`StateMachine::overTemperatureConfirmed()` independently of the latched fault
code, so diagnosis and protection do not compete for the same latch. The test
`Controller.OverTemperatureShutsDownEvenWhenAnotherFaultLatchedFirst` exists to
hold that behaviour in place.

**The stall detector times from the command, not from the mode.** Comparing
what was asked for on the previous scan against what the feedback reports now
means a pump that is already seized is caught during priming, before the
machine is ever loaded.

## Assumptions

1. The J1939 message set for the WP32 is defined in EMP document 9980010068,
   which was not supplied. Pump command and status use proprietary B PGNs
   (0xFF10 and 0xFF11) as placeholders. Substituting the real definitions is a
   change to `j1939.cpp` alone.
2. The traction inverter and DC-DC accept a power limit request over CAN. The
   PGN is a placeholder for whatever the vendor protocol specifies.
3. Thermal constants (15 L, UA values) are order of magnitude figures chosen to
   close the loop so the tuning can be shown to be stable. Real gains come off
   rig data.
4. The CR0403 runs CODESYS 2.3, so this C++ is the reference model and test
   bench and not the deployed artefact. The deployed logic is the Structured
   Text transliteration in the `EAE_Coding` deliverable. Keeping one testable
   reference implementation of the control law, instead of two written
   independently, is the reason for the split.
