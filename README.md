# Robotic Arm Firmware (ESP-IDF)

ESP32 firmware for a 4-axis stepper robotic arm with HTTP control (`/cmd`, `/crawl`, `/rotate`) and safety states.

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

- `/crawl?joint=<0-3>&dir=<R|L>&deg=<degrees>`
- `/rotate?joint=<0-3>&dir=<R|L>&deg=<degrees>&rpm=<rpm>`
- `/cmd?axis=<0-3>&steps=<target_steps>&rpm=<rpm>`
- `/stop`
- `/estop`
- `/state`
- `/uptime`

Example:

```text
http://<device-ip>/cmd?axis=1&steps=2500&rpm=20
http://<device-ip>/crawl?joint=2&dir=R&deg=5
http://<device-ip>/rotate?joint=2&dir=L&deg=15&rpm=60
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
- `main/config.c`: mechanics (`full_steps_per_rev`, `microsteps`, `gear_ratio`) and limits
- `main/comms.c`: HTTP endpoints
- `main/motion.c`: motion planning and command handling
- `main/pulse_gen.c`: hardware pulse generator
