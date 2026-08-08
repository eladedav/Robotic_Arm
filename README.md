# Robotic Arm Firmware (ESP-IDF)

ESP32 firmware for a 4-axis stepper robotic arm with HTTP control (`/rotate`, `/movej`, `/jog_start`, `/setpos`, `/home`) and safety states.

## Motor Driver Pin Assignment

| Joint | STEP | DIR | EN |
|---|---:|---:|---:|
| J0 | GPIO25 | GPIO26 | GPIO27 |
| J1 | GPIO14 | GPIO21 | GPIO13 |
| J2 | GPIO33 | GPIO32 | GPIO22 |
| J3 | GPIO23 | GPIO18 | GPIO19 |

> These values come from `main/board_pins.h`.

## HTTP API

All endpoints are `GET`.

- `/rotate?joint=<0-3>&dir=<R|L>&deg=<degrees>&rpm=<rpm>`
- `/movej?rpm=<rpm>&j0=<deg>&j1=<deg>&j2=<deg>&j3=<deg>` (relative multi-axis move)
- `/jog_start?joint=<0-3>&dir=<R|L>&rpm=<rpm>`
- `/jog_stop` or `/jog_stop?joint=<0-3>`
- `/setpos?joint=<0-3>&deg=<degrees>` or `/setpos?joint=<0-3>&steps=<steps>`
- `/home?joint=<0-3>`
- `/cmd?axis=<0-3>&steps=<target_steps>&rpm=<rpm>`
- `/stop`
- `/estop`
- `/state`
- `/uptime`

Example:

```text
http://<device-ip>/cmd?axis=1&steps=2500&rpm=20
http://<device-ip>/rotate?joint=2&dir=L&deg=15&rpm=60
http://<device-ip>/movej?rpm=10&j0=5&j1=-3&j2=0&j3=2
http://<device-ip>/jog_start?joint=1&dir=R&rpm=20
http://<device-ip>/jog_stop?joint=1
http://<device-ip>/setpos?joint=0&deg=0
http://<device-ip>/home?joint=0
```

## Build and Flash

```bash
idf.py set-target esp32
idf.py menuconfig
idf.py -p <PORT> flash monitor
```

Exit monitor with `Ctrl-]`.

## Main Config Files

- `main/board_pins.h`: STEP/DIR/EN GPIO mapping
- `main/config.c`: mechanics (`full_steps_per_rev`, `microsteps`, `gear_ratio`), direction inversion (`dir_inverted`), and limits
- `main/comms.c`: HTTP endpoints
- `main/motion.c`: motion planning and command handling
- `main/pulse_gen.c`: hardware pulse generator

## Invert Joint Direction

If one motor rotates opposite to your expected `R/L` command, set that joint's `dir_inverted` in `main/config.c`:

```c
.dir_inverted = true,
```

This flips only the DIR pin logic for that axis and keeps all kinematics (`steps_per_deg`, limits, RPM conversion) unchanged.
