"""
record.launch.py
----------------

Real-robot data-collection launch. Brings up:

  * MoveIt Servo + Jaka arm controllers (taken from your existing servo
    launch via include) — but with the simulated joy/servo bridge node
    LEFT IN (you still teleop with the gamepad).
  * DH AG95 gripper driver.
  * RealSense scene + wrist cameras publishing compressed images.
  * jaka_lerobot_bridge.bag_recorder_node.

If you do not want the cameras started from this file (e.g. you launch
them in a separate terminal), pass `cameras:=false`.

Usage:
    ros2 launch jaka_lerobot_bridge record.launch.py \
        out_dir:=$HOME/jaka_data/raw
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    out_dir_arg = DeclareLaunchArgument(
        "out_dir",
        default_value=os.path.expanduser("~/jaka_data/raw"),
        description="Where to write episode_NNNN bag dirs",
    )
    storage_arg = DeclareLaunchArgument(
        "storage", default_value="mcap",
        description="rosbag2 storage plugin (mcap recommended)",
    )
    cameras_arg = DeclareLaunchArgument(
        "cameras", default_value="true",
        description="Whether to launch realsense cameras here.",
    )

    pkg_share = get_package_share_directory("jaka_lerobot_bridge")

    # ----- include your existing servo launch (teleop + arm controllers) -----
    # NOTE: requires the launch file you already have. Adjust path if your
    # package name is different.
    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("jaka_driver"),
                "launch", "servo.launch.py",
            )
        ),
    )

    # ----- DH AG95 driver -----
    gripper_driver = Node(
        package="dh_gripper_driver",
        executable="dh_ag95_driver",
        name="dh_ag95_driver",
        parameters=[{
            "device_port": "/dev/ttyUSB0",
            "baudrate": 115200,
            "gripper_id": 1,
            "max_position": 100.0,
            "max_force": 100.0,
        }],
        output="screen",
    )

    # ----- cameras -----
    cameras = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, "config", "camera_realsense.launch.py")
            ),
        ),
    ], condition=IfCondition(LaunchConfiguration("cameras")))

    # ----- bag recorder -----
    bag_recorder = Node(
        package="jaka_lerobot_bridge",
        executable="bag_recorder_node",
        name="bag_recorder",
        output="screen",
        parameters=[{
            "out_dir": LaunchConfiguration("out_dir"),
            "storage": LaunchConfiguration("storage"),
        }],
    )

    return LaunchDescription([
        out_dir_arg,
        storage_arg,
        cameras_arg,
        servo_launch,
        # gripper_driver,
        # cameras,
        bag_recorder,
    ])
