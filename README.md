# Robotic Arm Firmware (ESP-IDF)

Embedded firmware for a 4-axis stepper-motor robotic arm, built with ESP-IDF on ESP32.
The system exposes a lightweight HTTP control API over Wi-Fi and includes runtime safety
gating, motion control, and status reporting.

## Highlights

- 4 independent stepper axes (`0..3`) with configurable mechanics and soft limits
- HTTP control endpoints for relative and absolute motion commands
- Wi-Fi station mode with mDNS (`http://esp-arm.local/`)
- Dedicated safety task with fault handling and emergency stop path
- GPTimer-based pulse generation for clean STEP signal output
- Modular architecture (configuration, motion, comms, safety, HAL)

## System Architecture

- `main/main.c` - boot sequence, subsystem initialization, and task startup
- `main/config.*` - axis configuration, conversions (`deg <-> steps`), limits
- `main/board_pins.h` - board-level GPIO assignments per joint
- `main/hal_gpio.*` - low-level STEP/DIR/EN line control
- `main/pulse_gen.*` - GPTimer pulse generation per axis
- `main/motion.*` - target tracking, motion execution, and movement APIs
- `main/safety.*` - system state machine, fault flags, motion permission logic
- `main/comms.*` - Wi-Fi connection, mDNS, HTTP server, request handlers

## Control API

All endpoints are currently `GET`.

- `/crawl?joint=<0-3>&dir=<R|L>&deg=<degrees>`
  - Slow relative motion by angle.
  - `R` = clockwise (+), `L` = counterclockwise (-).
  - Also accepts `degree=` instead of `deg=`.
- `/rotate?joint=<0-3>&dir=<R|L>&deg=<degrees>&rpm=<rpm>`
  - Relative angular motion at requested output-shaft speed.
- `/cmd?axis=<0-3>&steps=<target_steps>&rpm=<rpm>`
  - Absolute target in step units (`rpm` optional).
- `/stop`
  - Controlled stop for all axes.
- `/estop`
  - Emergency stop: halts motion and disables motor drivers.
- `/state`
  - JSON system snapshot (`state`, `faults`, positions, moving flag).
- `/uptime`
  - JSON uptime counters.

Example requests:

```text
http://<device-ip>/cmd?axis=1&steps=2500
http://<device-ip>/crawl?joint=2&dir=R&deg=5
http://<device-ip>/rotate?joint=2&dir=L&deg=15&rpm=2.5
http://esp-arm.local/state
```

## Build and Flash

### Prerequisites

- ESP-IDF `v5.x` installed and exported in your shell
- ESP32 development board connected over USB
- Python environment required by ESP-IDF

### Steps

1. Set target:

```bash
idf.py set-target esp32
```

2. Configure firmware options:

```bash
idf.py menuconfig
```

3. Build, flash, and open serial monitor:

```bash
idf.py -p <PORT> flash monitor
```

Exit monitor with `Ctrl-]`.

## Configuration

### Wi-Fi credentials

Configure network credentials in menuconfig:

```text
Robotic Arm Configuration
  -> Wi-Fi SSID
  -> Wi-Fi Password
```

These are defined in `main/Kconfig.projbuild` and used through:
- `CONFIG_ROBOT_WIFI_SSID`
- `CONFIG_ROBOT_WIFI_PASSWORD`

### Axis and mechanics

Edit these files to match your hardware:
- `main/board_pins.h` for GPIO wiring
- `main/config.c` for microstepping, gear ratio, limits, and speed constraints

## Safety Model

- Drivers are disabled at boot and enabled only after system enters `READY`
- Motion commands are rejected unless safety state allows movement
- `ESTOP` immediately stops motion and disables all drivers
- Fault conditions transition system to `FAULT` state

## Current Limitations

- No coordinated multi-axis trajectory planner yet
- No acceleration/jerk profile (fixed-step-rate style control)
- Position is relative from boot unless homing is added

## Roadmap Ideas

- Homing and limit switch integration
- Trajectory planning (trapezoidal/S-curve profiles)
- Authentication layer for network API
- Optional WebSocket or MQTT control interface

## License

Add your preferred license before publishing (for example, MIT).
