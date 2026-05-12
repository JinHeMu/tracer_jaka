"""
camera_realsense.launch.py
---------------------------

Bring up two RealSense cameras (scene + wrist) at 320x240@30fps with
compressed image transport. Both images are republished to
    /camera_scene/color/image_raw/compressed
    /camera_wrist/color/image_raw/compressed

To use a different camera (USB webcam, gscam, etc.), replace this file —
record.launch.py and inference.launch.py only depend on the topic names
above, not on RealSense specifically.

To run without cameras (e.g. you start them elsewhere), launch the parent
file with `cameras:=false`.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def _rs_node(ns: str, serial: str = ""):
    """RealSense camera node in namespace `ns`.

    Set `serial` (parameter `serial_no`) to disambiguate which physical
    camera is the scene vs the wrist cam.
    """
    params = {
        "camera_name": ns,
        "camera_namespace": "",
        "enable_color": True,
        "enable_depth": False,
        "enable_infra1": False,
        "enable_infra2": False,
        "rgb_camera.color_profile": "320x240x30",
        "color_qos": "SENSOR_DATA",
    }
    if serial:
        params["serial_no"] = serial

    return Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace=ns,
        name="realsense2_camera",
        parameters=[params],
        output="screen",
    )


def generate_launch_description():
    scene_serial_arg = DeclareLaunchArgument(
        "scene_cam_serial", default_value="",
        description="USB serial of the scene RealSense camera.",
    )
    wrist_serial_arg = DeclareLaunchArgument(
        "wrist_cam_serial", default_value="",
        description="USB serial of the wrist RealSense camera.",
    )

    # NOTE: when using realsense2_camera, the raw color image is
    # /camera_<ns>/color/image_raw and image_transport automatically
    # publishes /camera_<ns>/color/image_raw/compressed alongside it.
    return LaunchDescription([
        scene_serial_arg,
        wrist_serial_arg,
        _rs_node("camera_scene"),
        _rs_node("camera_wrist"),
    ])
