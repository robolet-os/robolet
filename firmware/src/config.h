// Wiring and options. Change the pin numbers here to match your board; -1 means "not connected".
// Defaults avoid the pins an ESP32-S3-N16R8 uses internally (26-37: flash and octal PSRAM),
// the strapping pins (0, 3, 45, 46), USB (19, 20), UART0 (43, 44), and the camera/SD pins of the
// common ESP32-S3 CAM boards (4-18, 38-40).
#pragma once

// ---- Robot preset. ROBOT_PRESET_JARABOT 1 (PlatformIO environment "jarabot": pio run -e jarabot -t upload): the
// Jarabot robot (github.com/firstbot1/jarabot) with an ESP32-S3 in place of its Raspberry Pi: YDLIDAR X4 Pro (no
// enable pins, M_CTR on LIDAR_M_SCTP_PIN), the Arduino Nano motor sketch on the base link. See docs/jarabot.md.
#ifndef ROBOT_PRESET_JARABOT
#define ROBOT_PRESET_JARABOT 0
#endif

// ---- USB serial console ----
#define CONSOLE_BAUD 115200

// ---- YDLIDAR X4 (datasheet: 5 V supply, 3.3 V UART at 128000 bps 8N1) ----
#define LIDAR_RX_PIN      41   // ESP32 receives here  <- X4 "Tx"
#define LIDAR_TX_PIN      42   // ESP32 sends here     -> X4 "Rx"
#if ROBOT_PRESET_JARABOT
#define LIDAR_M_EN_PIN    -1   // the X4 Pro has no enable pins: it scans when powered
#define LIDAR_DEV_EN_PIN  -1
#define LIDAR_M_SCTP_PIN  14   // -> X4 Pro "M_CTR" (PWM 10 kHz, lower duty = faster)
#define LIDAR_M_SCTP_DUTY 65   // percent; 65 % is the X4 Pro datasheet typical value
#else
#define LIDAR_M_EN_PIN     1   // -> X4 "M_EN"   (high = motor on)
#define LIDAR_DEV_EN_PIN   2   // -> X4 "DEV_EN" (high = ranging on); -1 if tied to 3.3 V
#define LIDAR_M_SCTP_PIN  14   // -> X4 "M_SCTP" speed (PWM 10 kHz, 50-100 % duty, lower duty = faster); -1 if not wired
#define LIDAR_M_SCTP_DUTY 85   // percent; 85 % is the datasheet typical value
#endif
#define LIDAR_BAUD    128000
// real robot ([r]): the lidar turns at 7 Hz (one control cycle per revolution); the duty is adjusted to keep it there
#define LIDAR_SPEED_CONTROL 1
// where the lidar sits: m ahead of the robot centre, and the direction of its 0 deg mark (deg, counter-clockwise from
// the robot's forward direction). Check with [l]: an object straight ahead must show up near 0 deg.
#if ROBOT_PRESET_JARABOT
#define LIDAR_MOUNT_X      0.025f
#else
#define LIDAR_MOUNT_X      0.0f
#endif
#define LIDAR_MOUNT_YAW_DEG 0.0f

// ---- Motor base link (optional). Sends "pwm,<turn>,<fwd>\n" and reads "<left>,<right>\n" encoder counts,
// the protocol of a simple Arduino Nano motor sketch (not part of this repository). 0 = disabled.
// Nano TX is 5 V: put a divider (e.g. 10k/20k) or level shifter before BASE_RX_PIN.
#define BASE_LINK_ENABLED ROBOT_PRESET_JARABOT
#define BASE_RX_PIN       47   // ESP32 receives here  <- Nano TX (through divider)
#define BASE_TX_PIN       21   // ESP32 sends here     -> Nano RX
#define BASE_BAUD      57600
#define BASE_TEST_FWD   1700   // value sent as <fwd> during the 1 s motor test (1500 = stop)
