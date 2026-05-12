"""
inference.launch.py
-------------------

Brings up the *real* robot for closed-loop policy inference. Compared with
record.launch.py, the joy/servo bridge is NOT started — the arm is driven
directly by policy_bridge_node, which talks ZMQ to a separately-running
LeRobot policy_server.

Make sure to start the policy server first:
    # in your LeRobot venv (Py3.12), maybe on a remote GPU box:
    python offline/policy_server.py --ckpt <path> --device cuda \
        --endpoint tcp://0.0.0.0:5555

Then on the robot machine:
    ros2 launch jaka_lerobot_bridge inference.launch.py \
        endpoint:=tcp://<gpu-host>:5555
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
    endpoint_arg = DeclareLaunchArgument(
        "endpoint", default_value="tcp://127.0.0.1:5555",
        description="ZMQ endpoint of policy_server.py",
    )
    rate_arg = DeclareLaunchArgument(
        "rate_hz", default_value="30.0",
        description="Inference rate.",
    )
    cameras_arg = DeclareLaunchArgument(
        "cameras", default_value="true",
        description="Whether to launch realsense cameras here.",
    )
    gripper_arg = DeclareLaunchArgument(
        "publish_gripper", default_value="true",
        description="Publish gripper /gripper/ctrl actions.",
    )

    pkg_share = get_package_share_directory("jaka_lerobot_bridge")

    # --- ros2_control + JTC, RViz, MoveIt — borrow your existing launch ---
    # If your `servo_launch.py` always starts the joy + servo composable
    # container, set `start_joy:=false` on it or replace this include with
    # a minimal launch that only brings up controller_manager + the arm
    # controllers + dh_ag95_driver.
    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("jaka_driver"),
                "launch", "servo_launch.py",
            )
        ),
    )

    gripper_driver = Node(
        package="dh_gripper_driver",
        executable="dh_ag95_driver",
        name="dh_ag95_driver",
        parameters=[{
            "device_port": "/dev/ttyUSB0",
            "baudrate": 115200,
            "gripper_id": 1,
            "max_position": 100.0,
        }],
        output="screen",
    )

    cameras = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, "config", "camera_realsense.launch.py")
            ),
        ),
    ], condition=IfCondition(LaunchConfiguration("cameras")))

    policy_bridge = Node(
        package="jaka_lerobot_bridge",
        executable="policy_bridge_node",
        name="policy_bridge",
        output="screen",
        parameters=[{
            "endpoint": LaunchConfiguration("endpoint"),
            "rate_hz": LaunchConfiguration("rate_hz"),
            "publish_gripper": LaunchConfiguration("publish_gripper"),
        }],
    )

    return LaunchDescription([
        endpoint_arg,
        rate_arg,
        cameras_arg,
        gripper_arg,
        servo_launch,
        gripper_driver,
        cameras,
        policy_bridge,
    ])
