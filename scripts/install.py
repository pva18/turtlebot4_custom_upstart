import os
import subprocess, shlex

# TODO: Do not use absolute paths for the ros2_ws paths.
# TODO: Test the correct working of paths
from ament_index_python.packages import get_package_share_directory


def main():
    """
    This script installs the custom launch file and changes the WORKSPACE_SETUP variable in the robot setup script file.

    This script assumes that the ROS 2 workspace is located at /home/ubuntu/ros2_ws and was built using the colcon build command after downloading the turtlebot4_custom_upstart package.
    """

    print("\n--------\nCHECK TODO!\n-------\n")

    turtlebot4_bringup_dir = get_package_share_directory("turtlebot4_bringup")
    turtlebot4_bringup_launch_dir = os.path.join(turtlebot4_bringup_dir, "launch")
    turtlebot4_custom_upstart_dir = get_package_share_directory("turtlebot4_custom_upstart")
    turtlebot4_custom_upstart_launch_dir = os.path.join(turtlebot4_custom_upstart_dir, "launch")

    # Back up the original launch file
    # if not os.path.exists("/opt/ros/humble/share/turtlebot4_bringup/launch/standard.launch.py.bak"):
    #     subprocess.run(
    #         shlex.split(
    #             "sudo mv "
    #             "/opt/ros/humble/share/turtlebot4_bringup/launch/standard.launch.py "
    #             "/opt/ros/humble/share/turtlebot4_bringup/launch/standard.launch.py.bak"
    #         )
    #     )
    if not os.path.exists(os.path.join(turtlebot4_bringup_launch_dir, "standard.launch.py.bak")):
        subprocess.run(
            shlex.split(
                " ".join(
                    [
                        "sudo mv",
                        os.path.join(turtlebot4_bringup_launch_dir, "standard.launch.py"),
                        os.path.join(turtlebot4_bringup_launch_dir, "standard.launch.py.bak"),
                    ]
                )
            )
        )
    print("Backed up the original launch file (turtlebot4_bringup/launch/standard.launch.py).")
    # Copy the custom launch file
    # subprocess.run(
    #     shlex.split(
    #         "sudo cp "
    #         "/home/ubuntu/ros2_ws/src/turtlebot4_custom_upstart/launch/robot_upstart.launch.py "
    #         "/opt/ros/humble/share/turtlebot4_bringup/launch/standard.launch.py"
    #     )
    # )
    subprocess.run(
        shlex.split(
            " ".join(
                [
                    "sudo cp",
                    os.path.join(turtlebot4_custom_upstart_launch_dir, "robot_upstart.launch.py"),
                    os.path.join(turtlebot4_bringup_launch_dir, "standard.launch.py"),
                ]
            )
        )
    )
    print(
        "Copied the custom launch file (turtlebot4_custom_upstart/launch/robot_upstart.launch.py) to turtlebot4_bringup launch directory."
    )

    workspace_setup_path = os.path.join(turtlebot4_custom_upstart_launch_dir, "../../../setup.bash")

    # Change the WORKSPACE_SETUP variable in the robot setup script file
    robot_setup_file = os.environ["ROBOT_SETUP"]
    with open(robot_setup_file, "r") as f:
        lines = f.readlines()
    with open(robot_setup_file, "w") as f:
        for line in lines:
            if "export WORKSPACE_SETUP" in line:
                # f.write('export WORKSPACE_SETUP="/home/ubuntu/ros2_ws/install/setup.bash"\n')
                f.write(f'export WORKSPACE_SETUP="{workspace_setup_path}"\n')
            else:
                f.write(line)
    print("Changed the WORKSPACE_SETUP variable in the robot setup script file.")

    print("To apply the changes, reinstall the robot upstart service using the turtlebot4-setup tool.")


if __name__ == "__main__":
    main()
