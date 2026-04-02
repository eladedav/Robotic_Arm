# Robotic Arm Firmware (ESP-IDF)

Firmware for a 4-axis stepper-based robotic arm running on ESP32 with FreeRTOS.

## Current Features

- 4 axis motion control in step units (`axis=0..3`)
- Slow relative moves by angle over HTTP (`/crawl`)
- Relative angle moves with requested RPM over HTTP (`/rotate`)
- GPIO HAL for `STEP` / `DIR` / `EN` motor driver signals
- Safety supervision task with `FAULT` and `ESTOP` states
- Wi-Fi station mode + HTTP control API
- Runtime state and uptime endpoints

## Project Structure

- `main/main.c` - startup sequence and subsystem initialization
- `main/config.*` - axis pin map, mechanics, limits, and conversion helpers
- `main/hal_gpio.*` - low-level GPIO control for motor drivers
- `main/motion.*` - target tracking and step generation loop
- `main/safety.*` - state machine, fault handling, motion gating
- `main/comms.*` - Wi-Fi and HTTP server endpoints

## HTTP API

All endpoints are currently `GET`.

- `/crawl?joint=<0-3>&dir=<R|L>&deg=<degrees>` - relative move by degrees, very slow (also accepts `degree=` instead of `deg=`). `R` = clockwise (+), `L` = counterclockwise (-) in step space; target is clamped to joint angle limits in `config.c`
- `/rotate?joint=<0-3>&dir=<R|L>&deg=<degrees>&rpm=<rpm>` - relative move by degrees at requested output RPM (also accepts `degree=` instead of `deg=`); same direction and limit behavior as `/crawl`
- `/cmd?axis=<0-3>&steps=<target_steps>&rpm=<rpm>` - set absolute target in steps at requested speed (rpm optional; default if omitted)
- `/stop` - controlled stop for all axes
- `/estop` - emergency stop (disable drivers + stop motion)
- `/state` - returns state, faults, positions, moving flag (JSON)
- `/uptime` - returns uptime data (JSON)

Example:

```text
http://<device-ip>/cmd?axis=1&steps=2500
http://<device-ip>/crawl?joint=2&dir=R&deg=5
http://<device-ip>/rotate?joint=2&dir=L&deg=15&rpm=2.5
```

## Build and Flash

1. Set target (if needed):

```bash
idf.py set-target esp32
```

2. Configure project options:

```bash
idf.py menuconfig
```

3. Build/flash/monitor:

```bash
idf.py -p <PORT> flash monitor
```

Exit monitor with `Ctrl-]`.

## Configuration Notes

- Axis pins and kinematic parameters are defined in `main/config.c`.
- Wi-Fi credentials are currently defined in `main/comms.c` via `WIFI_SSID` and `WIFI_PASS`.
- Drivers are initialized disabled, then enabled when safety enters `READY`.

## Known Limitations (Current Implementation)

- Motion loop is simple fixed-rate stepping (`MOTION_TASK_PERIOD_MS`, default 1 ms).
- Crawl uses a longer per-step interval (`MOTION_CRAWL_STEP_PERIOD_MS` in `motion.c`, default 250 ms).
- No acceleration profile or coordinated multi-axis planner yet.
- `/cmd` uses absolute steps only; soft angle limits are applied for `/crawl` and `config_angle_deg_to_steps`.

## Cleanup from ESP-IDF Blink Template

This repository originally started from the blink template. Template documentation and unused `led_strip` dependency have been removed from project metadata.
