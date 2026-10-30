# 13 · Contributor Guide

**Previous: [12 · Simulation, Testing, and Replay](12-simulation-testing-and-replay.md)** · **[Index](../README.md)**

---

This guide turns the architecture into a practical workflow. It focuses on where changes belong, which neighboring files must be updated, and how to verify the resulting behavior without bypassing PX4's safety and timing boundaries.

## 1. Start with ownership, not a filename search

Before editing, answer:

1. **Who owns the state or decision?** Commander, Navigator, EKF2, a controller, an output driver, or a service?
2. **What is the interface?** uORB topic, parameter, library API, shell command, MAVLink stream, DDS manifest, or startup script?
3. **What triggers execution?** Sensor publication, HRT, poll loop, command, or another topic?
4. **What is the timing contract?** Latest value or queued samples, sample timestamp, timeout, backup schedule, queue priority?
5. **What happens when data is missing or invalid?** A happy-path change without stale/failure behavior is incomplete.
6. **Which configurations must include it?** Kconfig, board variants, ROMFS, SITL, metadata, and tests.

A keyword search can find similar code, but it cannot decide which component should own the change.

## 2. Recipe: add a work-queue module

Use `src/templates/template_module/` and `src/examples/work_item/` as starting points.

### Files and responsibilities

1. Create `src/modules/<name>/` or the appropriate driver directory.
2. Add `CMakeLists.txt` with `px4_add_module()`:
   - unique `MODULE`;
   - shell `MAIN`;
   - sources, dependencies, YAML, and priority/stack metadata as needed.
3. Add a leaf `Kconfig` symbol and include it from the parent Kconfig menu.
4. Enable it in deliberate `.px4board` configurations.
5. Implement `<MAIN>_main()` and delegate to `ModuleBase::main()`.
6. Derive from `ModuleBase`, `ModuleParams`, and `px4::ScheduledWorkItem` when appropriate.
7. Choose the work queue deliberately and implement uppercase `Run()`.
8. Handle `start`, `stop`, `status`, parameter updates, and `should_exit()`.
9. Add a startup command to the appropriate airframe/vehicle/board script if it must auto-start.
10. Add tests and any required logging/telemetry metadata.

### Skeleton execution path

```text
rcS: <name> start
    → <name>_main
    → ModuleBase::main(desc)
    → <Name>::task_spawn
    → construct WorkItem on selected queue
    → start() → ScheduleOnInterval or register callback
    → Run() on queue
    → exit_and_cleanup(desc) on stop
```

Do not copy an old CRTP-based `ModuleBase` example blindly. This revision uses the descriptor-based, non-template base (`platforms/common/include/px4_platform_common/module.h:67-95`).

## 3. Recipe: add or change a uORB topic

1. Add/edit the schema under `msg/` or `msg/versioned/` according to the project's current compatibility rules.
2. Register the file in `msg/CMakeLists.txt`.
3. Decide whether it is:
   - latest state (default depth 1), or
   - a sample stream (`ORB_QUEUE_LENGTH` > 1).
4. Define `# TOPICS` aliases only when separate topic IDs are actually required; do not use aliases to imitate sensor instances.
5. Document units, coordinate frame, timestamp meaning, validity, reset counters, and NaN behavior in the schema.
6. Rebuild to generate C/C++ types, metadata, message hash, and serializers.
7. Use `uORB::Publication<T>` / `uORB::Subscription` or the appropriate multi-instance classes.
8. Set `timestamp` and `timestamp_sample` explicitly.
9. Decide which publication triggers consumers and whether related topics need a documented order.
10. Add logging, MAVLink, DDS, replay, and test integration only where deliberately required.

### Topic review questions

- Can a subscriber start before the publisher?
- What happens after timeout or a device ID change?
- Can queued samples overflow?
- Is instance number being mistaken for permanent identity?
- Does the consumer use `update()` when it only wants new data?
- Does a callback do anything beyond scheduling?

## 4. Recipe: add a parameter

1. Add the parameter definition to the owning module's YAML metadata, not generated headers.
2. Declare the corresponding typed `Param<T>` member through the module's existing `ModuleParams` pattern.
3. Include it in `updateParams()` refresh through the base-class mechanism.
4. Choose default, range, units, volatility, reboot requirement, and read-only behavior deliberately.
5. Use `param set-default` in airframe/board defaults where saved user choices should be respected.
6. Rebuild to regenerate `px4_parameters.hpp` and parameter metadata.
7. Verify:
   - default resolution;
   - set/reset behavior;
   - `parameter_update` reaction;
   - persistence after reboot;
   - armed/deferred behavior where relevant;
   - ground-station metadata.

Do not expect a parameter change to take effect in a module that never calls `updateParams()`.

## 5. Recipe: add a board or sensor driver

Follow the detailed sequences in [05](05-drivers-and-platform-abstraction.md). The minimum dependency checklist is:

```text
board .px4board feature selection
    + NuttX peripheral defconfig (for MCU)
    + board runtime pin/bus/timer data
    + board src/CMakeLists.txt
    + rc.board_sensors or other start line
    + chip driver Kconfig/CMake/MAIN/probe/RunImpl
    + stable device type and calibration identity
    + correct publication helper and units
    + failure/reset/stop-start tests
```

Never hardcode one board's bus, CS, DRDY, or rotation in portable chip logic. Never perform bus transactions in a DRDY ISR.

For outputs, verify timer DMA capability and function assignment separately. A PWM-capable pin is not automatically DShot-capable.

## 6. Recipe: change estimation or sensor processing

1. Identify whether the change belongs in:
   - chip conversion;
   - reusable calibration;
   - `VehicleIMU` integration;
   - source selection;
   - control-rate filtering;
   - EKF2 aiding/fusion;
   - output prediction;
   - estimator selection.
2. Preserve sample time, integration `dt`, units, and coordinate frames.
3. Update device/calibration counters if identity or calibration semantics changed.
4. Handle reset deltas/counters when output can jump.
5. Keep estimator IMU and control-rate angular-velocity paths distinct.
6. Test with:
   - stale samples;
   - queue gaps;
   - calibration changes;
   - sensor failover;
   - estimator instance switches;
   - aiding loss and recovery;
   - verbose ULog diagnostics.

Do not evaluate an estimator change only by final position plots. Innovations, test ratios, status flags, selection history, and timestamps reveal why the output changed.

## 7. Recipe: change control or actuation

First identify the cascade level:

| Change | Likely location |
|---|---|
| Manual stick-to-trajectory behavior | `flight_mode_manager` FlightTasks |
| MC position/velocity loop | `mc_pos_control` |
| MC attitude loop | `mc_att_control` |
| MC rate loop | `mc_rate_control` + `src/lib/rate_control` |
| FW guidance/energy control | `fw_mode_manager`, `fw_lateral_longitudinal_control`, `src/lib/tecs` |
| Torque/thrust to actuator geometry | `control_allocator`, `src/lib/control_allocation` |
| Logical actuator to physical pin | `mixer_module` and the concrete output driver |
| Arming permission | Commander—not the allocator or driver |
| Final disarmed/kill/termination values | `MixingOutput` and output configuration |

Verification should cover:

- correct mode-entry level;
- estimator resets and invalid setpoints;
- saturation and anti-windup;
- slew and output limits;
- disarmed, prearm, kill, and termination;
- motor-stop NaN behavior;
- function-to-pin mapping;
- relevant MC/FW/VTOL/rover variants.

## 8. Recipe: add a MAVLink stream or DDS topic

### MAVLink stream

1. Implement a `MavlinkStream` subclass under `src/modules/mavlink/streams/`.
2. Subscribe to internal topics and implement `send()` plus `update_data()` if high-rate collection is needed.
3. Register the stream in `mavlink_messages.cpp`.
4. Configure a deliberate default rate/mode where appropriate.
5. Respect link-budget and constant-rate behavior.
6. Test on constrained/lossy links as well as localhost.

### DDS topic

1. Ensure the PX4 message and generated serializer exist.
2. Add the correct `publications`, `subscriptions`, or `subscriptions_multi` entry to `dds_topics.yaml`.
3. Choose rate limit, instance routing, and message version deliberately.
4. Regenerate the bridge and use compatible `px4_msgs`/ROS 2 definitions.
5. Implement external semantic validation: frames, units, validity, source, timestamps, and NaN fields.
6. Test with the XRCE agent and relevant ROS 2 integration tests.

### Zenoh topic

1. Ensure the datatype is enabled through the generated `ZENOH_PUBSUB_*` Kconfig selection.
2. Update Zenoh's own `dds_topics.yaml` for generated defaults, or configure the persistent `pub.csv`/`sub.csv` mappings deliberately.
3. Rebuild the generated CDR/type-hash factory and restart after changing runtime mappings.
4. Configure `ZENOH_ENABLE`, domain, locator, and publisher options independently of uXRCE-DDS.
5. Use compatible `rmw_zenoh_cpp` and ROS 2 type-hash settings; test graph/liveliness behavior on the target ROS release.
6. Account for local-HRT timestamp replacement on inbound messages and the absence of uXRCE-style session time synchronization.

MAVLink, uXRCE-DDS, and Zenoh exposure are independent. Adding one does not add the others, and enabling Zenoh does not automatically disable uXRCE-DDS.

## 9. Debugging map

### Module will not start

```text
expanded boardconfig → module library → MAIN registration
→ startup command → full boot log → probe/start error → module status
```

Check that a quiet optional probe is not hiding a required device failure.

### Module runs but produces nothing

- Is its input topic advertised and updating (`uorb top`)?
- Is the subscription instance correct?
- Is a `vehicle_control_mode` flag gating publication?
- Is data stale/invalid or NaN by contract?
- Is the callback registered?
- Is a backup schedule firing?
- Is the work queue blocked by another long-running item?

### Timing or missed-sample problem

Inspect in this order:

1. source `timestamp_sample` and publication `timestamp`;
2. uORB generation gaps and queue depth;
3. work-queue load and pending-schedule coalescing;
4. HRT/backup schedule behavior;
5. callback context and ISR rules;
6. logger dropouts versus real sensor gaps;
7. lockstep component registration in SITL.

### Estimator problem

Compare:

- raw driver topics;
- `vehicle_imu` and `vehicle_imu_status`;
- `sensor_selection`;
- control-rate versus estimator IMU paths;
- aiding timestamps and validity;
- estimator status flags/innovations/test ratios;
- multi-EKF health and selector changes;
- reset counters and deltas.

### Actuator problem

Trace every layer rather than looking only at the pin:

```text
mode flags → controller setpoints → torque/thrust
→ Control Allocation status → actuator_motors/servos
→ function providers → actuator_armed + limits
→ actuator_outputs → physical protocol/timer/CAN
```

A zero at one layer can have a different meaning at another. Check NaN and natural output units.

## 10. Verification matrix

Choose the cheapest test that actually exercises the changed contract, then add the next level when cross-module behavior is involved.

| Test level | Best for | Limitation |
|---|---|---|
| Unit test | Math, parsers, allocation, state helpers, isolated logic | Does not prove startup, scheduling, or mode integration |
| Functional test | uORB/parameters/system infrastructure | Shared state can affect other tests |
| SITL process test | Startup and multi-module behavior | Host timing differs from NuttX |
| MAVSDK integration | Missions, commands, mode transitions, GCS-visible behavior | Usually high-level; inspect logs for internals |
| ROS 2 integration | DDS topic/service compatibility and external modes | Requires aligned message/workspace versions |
| Replay | Estimator investigation from recorded inputs | Not universal deterministic firmware re-execution |
| HITL / hardware | Board init, buses, timers, real-time and output behavior | Requires hardware-safety controls |

Run formatting and relevant static checks using the upstream contributor workflow. For C/C++ changes, upstream `AGENTS.md` requires `make format` before committing and CI checks `make check_format`. Read scoped `.github/instructions/*.instructions.md` for the paths being changed.

## 11. Architecture review checklist

### Ownership and interfaces

- [ ] One component clearly owns each new state machine and output topic.
- [ ] No hidden global duplicates existing uORB state.
- [ ] Message units, frames, timestamps, validity, instances, and NaN behavior are documented.
- [ ] External protocol conversion happens in the correct bridge.

### Timing and concurrency

- [ ] ISRs, HRT callouts, and uORB callbacks only perform minimal work.
- [ ] Work-queue choice and execution duration are justified.
- [ ] Queue depth and sample-loss behavior are intentional.
- [ ] Timeout/backup behavior exists where input can stop.
- [ ] Command/status/shared fields are safe across execution contexts.

### Safety and failure behavior

- [ ] Arming and mode authority remain with Commander.
- [ ] Final output safety remains in the output layer.
- [ ] Stale, invalid, non-finite, and missing data have defined behavior.
- [ ] Sensor/estimator resets and device changes are handled.
- [ ] Saturation, actuator limits, and failure paths are tested.

### Build and integration

- [ ] Kconfig, `CMakeLists.txt`, and `MAIN` entry are complete.
- [ ] Required `.msg`, YAML, ROMFS, airframe, and board registrations are updated.
- [ ] Generated files were regenerated, not hand-edited.
- [ ] SITL and NuttX differences are considered.
- [ ] Relevant logs, tests, and metadata are included.

## 12. High-value invariants to preserve

1. **uORB transports data; work queues execute code.**
2. **Callbacks schedule; they do not run the subscriber's control loop.**
3. **Build inclusion and runtime startup are independent.**
4. **Device identity is not uORB instance order.**
5. **Estimator IMU and control-rate IMU are separate paths.**
6. **Commander owns operating authority; Navigator owns autonomous traversal.**
7. **Control flags enable stages; topics carry the actual setpoints.**
8. **Control Allocation handles geometry; `mixer_module` handles output mapping and safety.**
9. **Simulation replaces drivers at topic boundaries, not the flight stack.**
10. **Persistence, logging, and telemetry are separate data paths.**

Keeping these invariants intact makes a change easier to test, review, port, and maintain.

---

**Previous: [12 · Simulation, Testing, and Replay](12-simulation-testing-and-replay.md)** · **[Index](../README.md)**
