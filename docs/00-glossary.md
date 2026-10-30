# 00 · Glossary

Acronyms and terms used in this analysis, the PX4 source tree, and PX4 documentation. The explanations describe what each term means **in PX4**, even where the same word has a broader meaning elsewhere.

**[← Back to index](../README.md)** · Next: [01 · Architecture Overview](01-architecture-overview.md)

---

## Project and vehicle terms

| Term | Stands for | What it means in PX4 |
|---|---|---|
| PX4 | *(project name)* | The open-source autopilot stack analyzed here. It supports aircraft and ground vehicles, not only quadcopters. |
| UAV | Unmanned Aerial Vehicle | A drone or unmanned aircraft. PX4 also controls UGVs and other platforms. |
| UGV | Unmanned Ground Vehicle | A rover controlled by one of the `rover_*` modules. |
| MC | Multicopter | A rotary-wing vehicle. Source prefixes include `mc_att_control` and `mc_rate_control`. |
| FW | Fixed Wing | An airplane or flying wing. Fixed-wing thrust is normally positive body X. |
| VTOL | Vertical Take-Off and Landing | A vehicle combining hover and fixed-wing flight, such as a standard VTOL, tiltrotor, or tailsitter. |
| FMU | Flight Management Unit | The main PX4 processor and a name used by many Pixhawk-board generations. |
| PX4IO | *(project/component name)* | An optional I/O coprocessor and its firmware/driver path, separate from direct FMU outputs. |
| GCS | Ground Control Station | Software such as QGroundControl that communicates with PX4, normally using MAVLink. |
| QGC / QGroundControl | *(application name)* | The primary PX4 ground station for configuration, missions, parameters, and flight display. |
| Offboard | *(mode name)* | Control supplied by an external computer through supported setpoint topics rather than internal navigation. |
| Airframe | *(configuration concept)* | A vehicle configuration identified by `SYS_AUTOSTART`; scripts select defaults, geometry, and vehicle type. |

## Platforms, boards, and operating systems

| Term | Stands for | What it means in PX4 |
|---|---|---|
| HAL | Hardware Abstraction Layer | A general term for the PX4 platform, board, bus, and architecture interfaces that hide hardware details. PX4 does not use ArduPilot's `AP_HAL` API. |
| RTOS | Real-Time Operating System | NuttX on most MCU flight controllers. It supplies tasks, scheduling, drivers, and board initialization. |
| NuttX | *(project name)* | The RTOS used by PX4 on microcontrollers. PX4 has its own platform layer above NuttX. |
| POSIX | Portable Operating System Interface | The Unix-like API used for SITL and Linux targets. POSIX does not necessarily mean simulation. |
| QURT | Qualcomm QuRT | The platform used for supported Qualcomm Hexagon/SLPI targets. |
| SITL | Software-In-The-Loop | PX4 compiled for a host computer and connected to simulated physics and sensors. |
| HITL / HIL | Hardware-In-The-Loop | Real flight-controller hardware running PX4 while a simulator supplies physics and sensor data. |
| SIH | Simulation In Hardware | PX4's internal dynamics model. It can run in SITL or on a flight controller. Despite the name, it also supports host-only simulation. |
| MCU | Microcontroller Unit | The chip running a NuttX flight-controller build. |
| SoC | System on Chip | A processor and its integrated peripherals. Platform code adapts PX4 to each supported family. |
| DSP | Digital Signal Processor / Processing | Can mean a signal-processing algorithm or a processor such as Qualcomm Hexagon; context distinguishes them. |
| Board label | *(build term)* | The `.px4board` filename suffix: `default`, `test`, `bootloader`, `rover`, and so on. |
| BSP | Board Support Package | The board's runtime headers, initialization, bus/timer descriptions, linker files, and startup hooks. |
| ROMFS | Read-Only Memory File System | Startup scripts, airframes, and defaults assembled at build time. NuttX links an embedded image; POSIX uses a generated file tree. |
| NSH | NuttX Shell | The shell that runs PX4 startup commands on NuttX. |
| PXH | PX4 Shell | The interactive command interface used by the POSIX PX4 server. |
| Flat build | *(NuttX term)* | A build without the kernel/userspace split used by protected configurations. |
| Protected build | *(NuttX term)* | A build with separate kernel and userspace sides; some PX4 calls cross the boundary through `boardctl`. |

## Hardware and low-level interfaces

| Term | Stands for | What it means in PX4 |
|---|---|---|
| GPIO | General-Purpose Input/Output | Digital pins used for chip select, data-ready interrupts, power control, safety, and LEDs. |
| UART | Universal Asynchronous Receiver-Transmitter | A serial interface used by GPS, telemetry, RC receivers, and other peripherals. |
| SPI | Serial Peripheral Interface | A fast bus used by many IMUs and other sensors. Board tables describe buses, CS, and DRDY pins. |
| I²C | Inter-Integrated Circuit | A lower-speed bus used by compasses, barometers, rangefinders, and other devices. |
| CAN | Controller Area Network | A robust bus used by DroneCAN/UAVCAN and Cyphal devices. |
| CS | Chip Select | The SPI signal that selects one device on a shared bus. |
| DRDY | Data Ready | A sensor interrupt signal. Drivers normally timestamp it and schedule work rather than reading the bus inside the ISR. |
| DMA | Direct Memory Access | Hardware-assisted transfers used by SPI, UART, timers, and DShot without CPU copying for every byte. |
| ADC | Analog-to-Digital Converter | Hardware used for battery voltage/current and other analog inputs. |
| PWM | Pulse-Width Modulation | A timer-driven servo/ESC signal, often expressed in microseconds. |
| DShot | *(protocol name)* | A digital ESC protocol. Bidirectional DShot can also return telemetry. |
| PPM | Pulse-Position Modulation | A legacy method for carrying multiple RC channels on one signal. |
| ESC | Electronic Speed Controller | The device that drives a motor. It may accept PWM, DShot, CAN, or another protocol. |
| HRT | High-Resolution Timer | PX4's microsecond monotonic-time and callout API. In lockstep SITL it follows simulated time. |
| ISR | Interrupt Service Routine | Hardware-interrupt context. Work there must be short; bus transfers and module logic normally run later. |
| RTC | Real-Time Clock | Wall-clock date/time, distinct from monotonic HRT time. `time_persistor` helps preserve it across reboots. |

## Middleware, modules, and execution

| Term | Stands for | What it means in PX4 |
|---|---|---|
| uORB | *(PX4 object broker)* | PX4's typed publish/subscribe middleware. Modules use it to exchange snapshots or queued samples. |
| Topic | *(uORB term)* | A named channel such as `vehicle_attitude` with one generated message type. |
| Topic instance | *(uORB term)* | One numbered node for a topic, such as `sensor_accel0` and `sensor_accel1`. Instances separate multiple sensors. |
| Topic alias | *(message-generation term)* | Another topic name with the same payload type but a different `ORB_ID`, for example a VTOL virtual setpoint. It is not an instance number. |
| `.msg` | *(schema file)* | The source definition of a uORB message. Generators create structs, metadata, serializers, and related outputs. |
| `ORB_ID(name)` | *(C/C++ macro)* | A pointer to topic metadata. It differs from the generated enum value `ORB_ID::name`. |
| Advertisement | *(uORB term)* | Making a topic instance available as a publisher. Advertising with null data does not publish a valid sample. |
| Publication | *(uORB term)* | Copying a complete message into the topic node's buffer and advancing its generation. |
| Subscription | *(uORB term)* | A reader with its own cursor into a topic instance. Separate subscriptions do not share read progress. |
| Queue depth | *(uORB term)* | Number of retained samples. The default is one (latest value); `ORB_QUEUE_LENGTH` requests more. |
| Generation | *(uORB term)* | A counter incremented on publication. Subscriptions use it to detect updates and missed samples. |
| `updated()` | *(subscription API)* | Reports unread data without copying it or advancing the cursor. |
| `copy()` | *(subscription API)* | Copies the current/next readable value and advances the cursor, even if no update was checked first. |
| `update()` | *(subscription API)* | Performs check-and-copy only when unread data exists. |
| Module | *(PX4 term)* | A build-registered application, driver, or command. It may run as a dedicated task, a work-queue item, or only handle commands. |
| `ModuleBase` | *(class name)* | Shared implementation of `start`, `stop`, `status`, object lifetime, and command dispatch. |
| `ModuleParams` | *(class name)* | Base class that refreshes declared C++ parameter objects when `updateParams()` is called. |
| Work queue | *(PX4 runtime)* | A worker thread that runs attached `WorkItem`s serially. Queue choice affects priority and latency. |
| `WorkItem` | *(class name)* | An object that can be queued for uppercase `Run()` on a work queue. |
| `ScheduledWorkItem` | *(class name)* | A `WorkItem` with HRT-based delayed, absolute, or interval scheduling. |
| `run()` / `Run()` | *(method names)* | Lowercase `run()` is the dedicated-module loop in `ModuleBase`; uppercase `Run()` is work-queue execution. They are not interchangeable. |
| Callback work item | *(uORB/runtime term)* | A subscription callback that schedules a `WorkItem` when data arrives. The callback runs in the publisher's context; `Run()` runs later. |
| Lockstep | *(simulation term)* | A POSIX scheduler that ties PX4 time and component progress to simulation time. It does not make host execution real-time. |
| muORB | *(PX4 term)* | The multi-processor/remote uORB path used by supported split systems. |

## Sensors and estimation

| Term | Stands for | What it means in PX4 |
|---|---|---|
| IMU | Inertial Measurement Unit | A gyroscope and accelerometer package. Drivers publish raw/integrated samples; `VehicleIMU` produces estimator-ready deltas. |
| INS | Inertial Navigation System | Used in work-queue names and inertial estimation code. |
| Gyro | Gyroscope | Measures angular rate, normally in rad/s in PX4's body FRD convention. |
| Accel | Accelerometer | Measures specific force, normally in m/s² in PX4 sensor messages. |
| Mag | Magnetometer | A compass used for magnetic heading and related estimation. |
| Baro | Barometer | A pressure sensor used for altitude and air-density estimates. |
| GNSS | Global Navigation Satellite System | GPS, Galileo, GLONASS, BeiDou, and related systems. PX4 topic names often use `gnss`; protocol parameters may still say GPS. |
| GPS | Global Positioning System | Often used loosely in PX4 to mean any GNSS receiver. |
| RTK | Real-Time Kinematic | Carrier-phase GNSS positioning that can provide centimeter-level relative accuracy with corrections. |
| PPS | Pulse Per Second | A precise timing signal used to align GNSS UTC time with PX4 HRT. |
| EKF | Extended Kalman Filter | A state estimator that predicts motion and corrects it with sensor measurements. |
| EKF2 | *(PX4 module name)* | PX4's main EKF module and EKF library. The name does not mean ArduPilot's EKF2 implementation. |
| Multi-EKF | *(PX4 architecture)* | Multiple EKF2 instances using selected IMU/magnetometer combinations, supervised by `EKF2Selector`. |
| Estimator selector | *(component name)* | Chooses a healthy EKF instance and republishes its estimator topics as system-wide vehicle topics. It does not blend states. |
| Output predictor | *(EKF component)* | Produces current-time attitude/position/velocity by integrating IMU data and applying corrections from the delayed EKF. |
| Fusion horizon | *(estimation term)* | The delayed time at which the EKF processes measurements so delayed aiding data can be aligned correctly. |
| Aiding source | *(estimation term)* | A non-IMU measurement used to correct the EKF, such as GNSS, barometer, magnetometer, airspeed, or optical flow. |
| Innovation | *(estimation term)* | The difference between an observed measurement and the estimator's prediction. Large innovations can indicate inconsistency. |
| Sensor voting | *(PX4 term)* | Health- and priority-based source selection. It usually does **not** mean numerical majority voting or averaging. |
| Calibration count | *(message field)* | A counter that changes when calibration changes, allowing downstream code to invalidate old learned bias. |
| FIFO | First In, First Out | A sensor or software buffer containing samples in arrival order. IMU drivers often publish FIFO batches and integrated reports. |
| FRD | Forward-Right-Down | PX4's common body-axis convention. |
| NED | North-East-Down | PX4's common local navigation-frame convention. |
| ENU / FLU | East-North-Up / Forward-Left-Up | Conventions used by many external systems; simulation bridges convert them to NED/FRD. |
| AHRS | Attitude and Heading Reference System | A general term for attitude estimation. PX4 code primarily exposes concrete topics rather than one AHRS object. |
| IAS / TAS | Indicated / True Airspeed | IAS comes from differential pressure; TAS compensates for air density. Both appear in airspeed processing. |
| QNH | *(aviation pressure term)* | Sea-level pressure reference used by `SENS_BARO_QNH` to compute QNH-referenced barometric altitude above mean sea level. It is distinct from pressure altitude relative to the standard pressure datum. |

## Control, navigation, and safety

| Term | Stands for | What it means in PX4 |
|---|---|---|
| Setpoint | *(control term)* | A desired value or trajectory. Different topics represent position, attitude, rate, torque, thrust, or actuator commands. |
| Cascade control | *(control pattern)* | Outer loops produce setpoints for inner loops: position → attitude → rate → torque/thrust → actuators. |
| PID | Proportional–Integral–Derivative | A feedback controller using error, accumulated error, and a derivative-related term. PX4 rate control also uses feed-forward. |
| Feed-forward | *(control term)* | A command derived directly from the desired motion instead of waiting for feedback error. |
| Anti-windup | *(control term)* | Logic that prevents or reduces integrator growth when actuators saturate or control is inactive. |
| NaN | Not a Number | Used deliberately in many setpoints to mean “not controlled” or, in some actuator paths, “stopped.” Always check the message contract. |
| Control Allocation | *(PX4 subsystem)* | Converts desired torque and body thrust into motor and servo commands using vehicle effectiveness geometry. |
| Effectiveness matrix | *(allocation term)* | Describes how each actuator affects torque and thrust. The allocator uses it to solve for actuator commands. |
| `mixer_module` | *(library name)* | The shared output-function, channel-mapping, scaling, and safety library—not the legacy vehicle geometry mixer. |
| Output function | *(configuration term)* | A logical role such as Motor 1 or Servo 2 assigned to a physical output channel with `<prefix>_FUNCn`. |
| `actuator_outputs` | *(topic name)* | Driver-level outputs in natural units. They are not universally normalized. |
| Prearm | *(safety state)* | An actuator readiness state separate from the main armed/disarmed state. |
| Arming | *(safety transition)* | Commander's authorization to enter armed operation after checks. Publishing actuator setpoints alone does not arm outputs. |
| Failsafe | *(safety mechanism)* | A response to unhealthy conditions, such as RC loss, estimator failure, or low battery. Commander selects the action. |
| Failure detector | *(component name)* | Detects conditions such as attitude/altitude failures and publishes evidence used by Commander. |
| Land detector | *(module name)* | Estimates landed/takeoff states used by arming, controllers, missions, and auto-disarm. |
| Commander | *(module name)* | Owns arming, effective navigation state, health checks, failsafe selection, `vehicle_status`, and `vehicle_control_mode`. |
| Navigator | *(module name)* | Owns autonomous navigation modes and produces `position_setpoint_triplet`. It does not arm the vehicle. |
| `nav_state` | *(status field)* | The effective navigation state after mode intention, failsafe action, and external replacement. |
| `vehicle_control_mode` | *(topic name)* | Controller-enable flags derived by Commander. It enables consumers; it does not route message payloads. |
| FlightTask | *(class family)* | Multicopter mode-specific trajectory generation inside `flight_mode_manager`. |
| Mode executor | *(external-mode concept)* | An external component that can run a registered mode and issue commands while Commander retains safety authority. |
| Mission | *(navigation concept)* | An ordered set of navigation and action items stored through Dataman and executed by Navigator. |
| RTL | Return-To-Launch | The traditional name for Return mode. Current behavior can target Home, mission landing, or a rally/safe point depending on configuration. |
| Rally point | *(navigation concept)* | A predefined safe destination used by Return and mission logic. |
| Geofence | *(navigation/safety concept)* | Geographic/altitude limits checked by Navigator; Commander applies the configured breach action. |
| Dataman | *(module name)* | Indexed record storage for missions, fences, rally points, and related state. |
| TECS | Total Energy Control System | Fixed-wing altitude/airspeed control based on energy balance. |
| NPFG | *(guidance algorithm name)* | Nonlinear path-following guidance used by fixed-wing navigation/control code. |
| Quad-chute | *(VTOL failsafe)* | A fixed-wing-to-multicopter recovery response to dangerous transition or flight conditions. |

## Communication and persistence

| Term | Stands for | What it means in PX4 |
|---|---|---|
| MAVLink | Micro Air Vehicle Link | The main GCS/companion protocol. PX4 implements streams, commands, missions, parameters, FTP, logging, and time synchronization. |
| DDS | Data Distribution Service | A publish/subscribe standard used by ROS 2. |
| uXRCE-DDS | Micro XRCE-DDS | The resource-constrained DDS transport used by `uxrce_dds_client`. |
| Zenoh | *(protocol/project name)* | A separate PX4 pub/sub bridge for ROS 2 `rmw_zenoh`, using generated CDR type support and persistent CSV topic mappings rather than an XRCE agent. |
| `rmw_zenoh` | ROS Middleware Zenoh implementation | The ROS 2 middleware used with PX4's Zenoh bridge through `zenohd`/`rmw_zenohd`. |
| Key expression | *(Zenoh term)* | The hierarchical name used for a Zenoh endpoint. PX4's ROS-compatible keys include topic, type, and RIHS01 information. |
| RIHS01 | ROS Interface Hashing Standard 01 | The type-hash scheme embedded in default Zenoh/ROS 2 key expressions for recent ROS 2 versions. |
| XRCE agent | *(external component)* | The host-side agent that connects PX4's uXRCE-DDS client to ROS 2/DDS participants. |
| ROS 2 | Robot Operating System 2 | A robotics framework that exchanges selected PX4 messages through the DDS bridge. |
| CDR / uCDR | Common Data Representation / Micro CDR | Serialization used by DDS and the generated PX4 message serializers. |
| QoS | Quality of Service | DDS reliability, durability, and history settings. PX4 topic data and command services use different policies. |
| RC | Radio Control | Pilot input from a receiver or an emulated MAVLink source. |
| RSSI | Received Signal Strength Indicator | A link-quality indication for RC or telemetry. |
| SBUS / CRSF / ELRS | *(RC protocol/link names)* | Receiver protocols and links decoded into PX4 RC/manual-control messages. |
| DroneCAN / UAVCAN | *(protocol names)* | CAN protocol families for sensors, ESCs, and peripherals. UAVCAN is the older name still used by directories and options. |
| Cyphal | *(protocol name)* | A CAN-capable communication protocol supported by dedicated PX4 driver code. |
| FTP | File Transfer Protocol | MAVLink's file operation protocol, not Internet FTP. PX4 also exposes virtual parameter files through it. |
| ULog | *(PX4 log format)* | PX4's self-describing binary flight-log format containing message definitions, data, parameters, events, and dropouts. |
| Parameter | *(configuration term)* | A named typed setting stored in layered memory and optionally persisted to file or flash. |
| `parameter_update` | *(topic name)* | A summary notification that parameters changed. It does not carry the changed name/value; modules then refresh their parameters. |
| BSON | Binary JSON | The serialization used by PX4 parameter files and flash records. |
| Events | *(PX4 event protocol)* | Structured status/notification records sent to logs and MAVLink with sequence and retransmission handling. |

## Build and verification

| Term | Stands for | What it means in PX4 |
|---|---|---|
| Make | *(build tool)* | The user-facing front end that maps board target names to CMake builds. It is not the whole build implementation. |
| CMake | *(build tool)* | Configures source lists, generated code, module libraries, platforms, and firmware outputs. |
| Kconfig | *(configuration system)* | Selects PX4 features and modules. NuttX has a separate Kconfig configuration. |
| `.px4board` | *(file suffix)* | A saved PX4 Kconfig defconfig for one board label. Most non-default variants are merged with `default.px4board`. |
| `px4_add_module()` | *(CMake function)* | Registers module sources, entry point, dependencies, stack/priority metadata, and YAML configuration. |
| `MAIN` | *(module metadata)* | The shell command name, such as `commander`. It is not the CMake library target name. |
| Autostart | *(startup term)* | The airframe ID and generated/runtime script mechanism that selects vehicle configuration. |
| `.px4` firmware | *(package format)* | A JSON package with metadata and compressed/base64-encoded firmware and metadata—not the raw `.bin` image. |
| CI | Continuous Integration | Automated builds, checks, and tests run for changes. |
| GoogleTest / gtest | *(test framework)* | The C++ framework used by PX4 unit and functional tests. |
| MAVSDK | *(library/toolkit)* | Used by integration tests to control PX4 through MAVLink. |
| Replay | *(PX4 module/workflow)* | Replays logged sensor messages into estimator-related processing for investigation and comparison. |
| Sanitizer | *(compiler tooling)* | Address, thread, undefined-behavior, and related runtime checks available for host builds. |

## Naming patterns worth recognizing

| Pattern | Meaning | Example |
|---|---|---|
| `sensor_*` | Driver-level or early sensor measurement | `sensor_accel`, `sensor_gnss` |
| `vehicle_*` | Processed vehicle state, command, or setpoint | `vehicle_imu`, `vehicle_attitude` |
| `estimator_*` | Output from one EKF instance before selection | `estimator_attitude` |
| `*_setpoint` | Desired value consumed by a downstream stage | `trajectory_setpoint` |
| `*_status` | State, health, or diagnostic information | `vehicle_status`, `control_allocator_status` |
| `mc_*`, `fw_*` | Multicopter or fixed-wing implementation | `mc_rate_control`, `fw_att_control` |
| `*_virtual_mc`, `*_virtual_fw` | VTOL controller-specific virtual topics | `vehicle_torque_setpoint_virtual_mc` |
| `<PREFIX>_FUNCn` | Assign a logical function to physical output `n` | `PWM_MAIN_FUNC1` |
| `CONFIG_MODULES_*` | Kconfig symbol enabling a module source directory | `CONFIG_MODULES_COMMANDER` |
| `CONFIG_DRIVERS_*` | Kconfig symbol enabling a driver source directory | `CONFIG_DRIVERS_IMU_INVENSENSE_ICM42688P` |

---

**[← Back to index](../README.md)** · Next: [01 · Architecture Overview](01-architecture-overview.md)
