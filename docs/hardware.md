# Hardware

## Board

ESP32-S3 with 16 MB flash and 8 MB octal PSRAM (ESP32-S3-N16R8), for example an ESP32-S3-DevKitC-1 N16R8. PlatformIO settings: `firmware/platformio.ini` (espressif32 6.13.0, Arduino core 2.0.17, `qio_opi` memory). The console runs on both USB ports (the chip's own USB and the USB-UART); the PC tools use the chip's USB port.

`bash test.sh` starts with a board check: flash ≥ 16 MB, PSRAM ≥ 8 MB with a 4 MB write/read pattern test, and the reason of the last restart.

## YDLIDAR X4 (optional)

Pins are set in `firmware/src/config.h`; `-1` means not connected.

| X4 | ESP32-S3 | Note |
|---|---|---|
| VCC 5 V, GND | 5 V, GND | a 5 V supply with enough current for the X4 motor (see the X4 data sheet) |
| Tx | GPIO 41 (`LIDAR_RX_PIN`) | 3.3 V UART, 128000 bps 8N1 |
| Rx | GPIO 42 (`LIDAR_TX_PIN`) | |
| M_EN | GPIO 1 | high = motor on |
| DEV_EN | GPIO 2 | high = ranging on (or tie to 3.3 V and set -1) |
| M_SCTP | GPIO 14 | PWM 10 kHz, 50–100 % duty, lower = faster; 85 % typical |

The defaults avoid the pins the N16R8 uses internally (26–37), the strapping pins (0, 3, 45, 46), USB (19, 20), UART0 (43, 44) and the camera/SD pins of common ESP32-S3 camera boards (4–18, 38–40).

Check: `bash test.sh lidar` (10 s of packets, revolutions, checksum errors). The X4 reports angles clockwise from its 0° mark; the conversion to the ROS convention (counter-clockwise) is not done yet.

## Motor base (optional, not tested with this stack yet)

A serial link to a small motor controller (for example an Arduino Nano): the chip sends `pwm,<turn>,<fwd>\n` and reads `<left>,<right>\n` encoder counts at 57600 bps (`BASE_*` in `config.h`, disabled by default). A 5 V Nano TX needs a divider or level shifter before the ESP32 input.
