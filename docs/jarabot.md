# Robolet on the Jarabot robot

[Jarabot](https://github.com/firstbot1/jarabot) is a differential-drive education robot: a Raspberry Pi 4 running ROS 2,
a YDLIDAR X4 Pro, and an Arduino Nano that drives the two motors and counts the wheel encoders (`motor.ino`). Here an
ESP32-S3-N16R8 takes the place of the Raspberry Pi and runs the whole navigation stack; the Arduino and its sketch stay
as they are (one safety change is recommended below). Status: firmware and PC test ready (`test_host/test_real.cpp`
with a simulated Jarabot); not yet tested on the robot.

## Two ways to connect

- **Relay (no rewiring):** the lidar (its USB adapter, CP2102) and the Arduino stay on the USB ports of the robot's
  Raspberry Pi, the ESP32-S3 goes on a third USB port, and `tools/relay.py` on the Raspberry Pi copies the serial bytes
  between them and the chip. The chip computes everything (mapping, localization, planning, motor commands); the
  Raspberry Pi only passes bytes. Firmware command `r relay`.
- **Wires:** the ESP32-S3 replaces the Raspberry Pi and takes the lidar and the Arduino on its own UART pins (below).
  Firmware command `r`.

## Relay

```bash
# on the PC: build and upload the Jarabot firmware, then plug the ESP32-S3 into the Raspberry Pi
cd firmware && pio run -e jarabot -t upload
rsync -a --exclude .pio firmware jarabot@<robot ip>:~/robolet/      # the tools on the Raspberry Pi
# on the Raspberry Pi (ssh), each in its own terminal; ROS 2 Humble, the same ROS_DOMAIN_ID as the PC
cd ~/robolet/firmware && python3 tools/ros2.py daemon --chip "r relay"
cd ~/robolet/firmware && python3 tools/relay.py                      # /dev/mylidar, /dev/mydriver (Jarabot udev rules)
cd ~/robolet/firmware && source /opt/ros/humble/setup.bash && /usr/bin/python3 tools/ros2_bridge.py
# on the PC
rviz2 -d tools/robolet.rviz --ros-args -p use_sim_time:=true
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/teleop/cmd_vel
```

With the USB adapter the lidar turns at its own default speed (the adapter drives M_CTR), so a control cycle is one
revolution at that speed. The relay stops the wheels itself when no motor command comes from the chip for 0.5 s.
The Jarabot ROS 2 nodes (bringup) must not run at the same time: they would hold the same serial ports.

## Wiring

| From | ESP32-S3 | Note |
|---|---|---|
| X4 Pro Tx | GPIO 41 | 3.3 V UART, 128000 bps |
| X4 Pro M_CTR | GPIO 14 | PWM 10 kHz; the firmware keeps the lidar at 7 Hz |
| X4 Pro VCC, GND | robot 5 V supply, GND | up to 1 A when the motor starts: not from an ESP32 pin |
| Nano D1 (TX) | GPIO 47 through a divider | 5 V signal: Nano TX – 10 kΩ – GPIO 47 – 20 kΩ – GND |
| Nano D0 (RX) | GPIO 21 | |
| Nano 5V, GND | robot 5 V supply, GND | the Nano's USB cable must be unplugged (its USB chip drives D0) |
| ESP32-S3 USB | PC (or the Raspberry Pi as a relay) | the ROS 2 bridge and the `ros2`-style tools |

All grounds are common. The Nano's `motor.ino` speaks 57600 bps: the ESP32 sends `pwm,<turn>,<fwd>` and reads
`<left>,<right>` encoder counts, as the Raspberry Pi did through the USB cable. To upload a new sketch to the Nano,
take the wires off D0/D1 and use its USB cable.

## Recommended change to motor.ino: stop without commands

`motor.ino` keeps the last command, so the wheels keep turning if the ESP32 stops or a wire comes off. The firmware sends
a command every cycle (7 per second) and a stop when no command comes for 0.5 s, but a timeout in the sketch also covers
a failure of the ESP32. Add (the sketch sets Timer0 to x8, so `millis()` runs 8 times fast: 4000 = 0.5 s):

```cpp
unsigned long lastCmd = 0;                       // with the other globals

  if (Serial.available()) {
    rcvStr = Serial.readStringUntil('\n');
    strLength = rcvStr.length();
    lastCmd = millis();                          // new
  }
  ...
  if (millis() - lastCmd > 4000UL) { roll = 1500; pitch = 1500; }   // new, before "if (roll <= 1000)"
```

## Steps

```bash
cd firmware
bash test.sh jarabot lidar     # Jarabot firmware (PlatformIO env "jarabot") + board check + lidar check
bash test.sh jarabot base      # + the motors: encoder counts, then 1 s forward (lift the wheels off the ground)
```

`lidar` must show about 7 revolutions per second and few checksum errors. Put a box 0.5 m straight ahead of the robot and
run `s` (lidar stream) in a serial monitor: the near reading must be at 0 deg; if it is at 180 deg, set
`LIDAR_MOUNT_YAW_DEG` to 180 in `src/config.h`. `base` prints the counts of 1 s forward; if one or both are negative, set
`left_encoder_sign` / `right_encoder_sign` of `/base_driver` to -1.

A run (three terminals on the PC, ROS 2 Humble):

```bash
python3 tools/ros2.py daemon --chip "r"                 # 1: the robot run; the robot starts at the middle of the map
source /opt/ros/humble/setup.bash
/usr/bin/python3 tools/ros2_bridge.py                   # 2: topics to ROS 2; goals, initial pose and keyboard to the chip
rviz2 -d tools/robolet.rviz --ros-args -p use_sim_time:=true      # 3
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/teleop/cmd_vel   # 4: drive while it maps
```

1. Mapping: drive slowly through the room with the keyboard; the map grows in rviz2. The map is 10 × 8 m around the
   start point.
2. `python3 tools/ros2.py service call /mission/finish_mapping` ends mapping: AMCL starts at the SLAM pose.
3. Goals: rviz2 "2D Goal Pose". The keyboard still works and wins over navigation for 0.5 s after each key.
4. Kidnapping: lift the robot and put it down elsewhere during a goal; the robot detects it, searches and turns in place.
5. `python3 tools/ros2.py mission stop` ends the run.

Parameters of `/base_driver` (`python3 tools/ros2.py param list /base_driver`): `wheel_radius` 0.035 m, `wheel_base`
0.220 m, `encoder_resolution` 1200, `max_vel_x` 0.26 m/s, `max_vel_theta` 1.0 rad/s, `pwm_deadband` 170,
`pwm_per_mps` 1269, `pwm_per_radps` 330 (the values of Jarabot's `jara_controller`), `cmd_timeout` 0.5 s, `pwm_step` 100.
