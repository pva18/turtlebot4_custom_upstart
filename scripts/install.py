import os
import subprocess, shlex
import robot_upstart

# See install process in https://github.com/turtlebot/turtlebot4_setup/blob/humble/turtlebot4_setup/ros_setup.py RobotUpstart class


def daemon_reload():
    subprocess.run(shlex.split("sudo systemctl daemon-reload"))


def uninstall():
    subprocess.run(shlex.split("sudo systemctl stop turtlebot4.service"))

    turtlebot4_job = robot_upstart.Job(
        name="turtlebot4", workspace_setup=os.environ["ROBOT_SETUP"]
    )
    turtlebot4_job.uninstall()

    daemon_reload()


def install():
    uninstall()

    rmw = os.environ["RMW_IMPLEMENTATION"]
    if rmw == "rmw_fastrtps_cpp":
        rmw_config = os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"]
    else:
        rmw_config = os.environ["CYCLONEDDS_URI"]

    turtlebot4_job = robot_upstart.Job(
        name="turtlebot4",
        workspace_setup="/home/ubuntu/ros2_ws/src/turtlebot4_custom_upstart/scripts/setup.bash",
        rmw=rmw,
        rmw_config=rmw_config,
        systemd_after="network-online.target",
    )

    turtlebot4_job.add(
        package="turtlebot4_custom_upstart", filename="robot_upstart.launch.py"
    )
    turtlebot4_job.install()

    daemon_reload()


def main():
    print("Reinstalling turtlebot4 service with updated configuration...")
    install()


if __name__ == "__main__":
    main()
