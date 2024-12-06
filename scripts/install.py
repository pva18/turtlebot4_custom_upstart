import os
import subprocess, shlex

# TODO: Do not use absolute paths for the ros2_ws paths.

def main():
    """
    This script installs the custom launch file and changes the WORKSPACE_SETUP variable in the robot setup script file.

    This script assumes that the ROS 2 workspace is located at /home/ubuntu/ros2_ws and was built using the colcon build command after downloading the turtlebot4_custom_upstart package.
    """

    ros_distro = os.environ["ROS_DISTRO"]

    # Back up the original launch file
    if not os.path.exists(f"/opt/ros/{ros_distro}/share/turtlebot4_bringup/launch/standard.launch.py.bak"):
        subprocess.run(
            shlex.split(
                "sudo mv "
                f"/opt/ros/{ros_distro}/share/turtlebot4_bringup/launch/standard.launch.py "
                f"/opt/ros/{ros_distro}/share/turtlebot4_bringup/launch/standard.launch.py.bak"
            )
        )
    print(
        "Backed up the original launch file (turtlebot4_bringup/launch/standard.launch.py)."
    )
    # Copy the custom launch file
    subprocess.run(
        shlex.split(
            "sudo cp "
            "/home/ubuntu/ros2_ws/src/turtlebot4_custom_upstart/launch/robot_upstart.launch.py "
            f"/opt/ros/{ros_distro}/share/turtlebot4_bringup/launch/standard.launch.py"
        )
    )
    print(
        "Copied the custom launch file (turtlebot4_custom_upstart/launch/robot_upstart.launch.py) to turtlebot4_bringup launch directory."
    )

    # Change the WORKSPACE_SETUP variable in the robot setup script file
    robot_setup_file = os.environ["ROBOT_SETUP"]
    with open(robot_setup_file, "r") as f:
        lines = f.readlines()
    with open(robot_setup_file, "w") as f:
        for line in lines:
            if "export WORKSPACE_SETUP" in line:
                f.write(
                    'export WORKSPACE_SETUP="/home/ubuntu/ros2_ws/install/setup.bash"\n'
                )
            else:
                f.write(line)
    print("Changed the WORKSPACE_SETUP variable in the robot setup script file.")

    print(
        "To apply the changes, reinstall the robot upstart service using the turtlebot4-setup tool."
    )


if __name__ == "__main__":
    main()
