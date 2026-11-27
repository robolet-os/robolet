# msg/ — ROS 2 message definitions used on the chip

Field names, types and order follow the ROS 2 interface packages (builtin_interfaces, std_msgs, geometry_msgs,
nav_msgs, sensor_msgs, tf2_msgs; ROS 2 Jazzy / Lyrical). Those packages are Apache-2.0 licensed
(https://github.com/ros2/common_interfaces, https://github.com/ros2/rcl_interfaces, https://github.com/ros2/geometry2).
Only the types the firmware uses are here; comments of the original files are left out.

`tools/gen_msgs.py` turns them into C++ structs in `src/ros2_msgs.h`. Unbounded arrays (`T[]`) and strings get the fixed
capacities in `capacities.txt`, because the chip does not allocate memory per message.
