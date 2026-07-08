# ROS Message Definitions

These `.msg` files describe the recommended stable interfaces. They are not a complete ROS package yet.

For a buildable package, create `smart_cane_interfaces` with:

- `package.xml`;
- `CMakeLists.txt`;
- dependencies on `builtin_interfaces` and `rosidl_default_generators`;
- the message files under `msg/`.

The current integration nodes use JSON over `std_msgs/String` to remain compatible with the existing scripts.
