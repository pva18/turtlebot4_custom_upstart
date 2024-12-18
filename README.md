# turtlebot4_custom_upstart

This package provides a custom upstart configuration for the turtlebot4 robot.

To install the configuration, use the `scripts/install.py` script. Before usage, use the `turtlebot4_setup` package to setup the robot with the necessary environment variables. After the installation process in `scripts/install.py`, reinstall turtlebot4 service with turtlebot4-setup.
(turtlebot4-setup -> ROS Setup -> Upstart -> Uninstall -> Install -> Start)

**Note:**
*!NEW VERSION! See TODO below!*
The install script uses absolute paths for configuration files. For this reason, it is required for this package to be installed to "/home/ubuntu/ros2_ws/src/turtlebot4_custom_upstart".
If this packages is installed to another place you need to modify the install script.

**TODO:**
Test the new install script with paths resolved by `ament_index_python.packages.get_package_share_directory`.
