# =============================================================================
#  jaka_pick_place_demo launch
#  会依次启动：
#    1. 你 moveit_config 包中的 demo.launch.py (move_group + rviz + fake controllers)
#    2. 延迟若干秒后启动 pick_place_demo 节点
#  如果你的 moveit_config 包不是 tracer_jaka_zu5_moveit_config，请修改下面的
#  MOVEIT_CONFIG_PKG 变量。
# =============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


MOVEIT_CONFIG_PKG = "tracer_jaka_moveit_config"   # ← 按需修改
ROBOT_NAME        = "tracer_jaka"


def generate_launch_description():
    # -------- 1. 构建 moveit_config（把 URDF/SRDF/运动学/规划管线参数集中注入）--------
    moveit_config = (
        MoveItConfigsBuilder(ROBOT_NAME, package_name=MOVEIT_CONFIG_PKG)
        .to_moveit_configs()
    )

    # -------- 2. 包含 moveit_config 的 demo.launch.py --------
    moveit_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(MOVEIT_CONFIG_PKG),
                "launch",
                "demo.launch.py",
            )
        )
    )

    # -------- 3. 我们的 pick & place 节点 --------
    pick_place_node = Node(
        package="jaka_pick_place_demo",
        executable="pick_place_demo",
        name="pick_place_demo",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            # 某些发行版还需要下面这个；若编译/运行报缺参数可启用
            # moveit_config.joint_limits,
        ],
    )

    # move_group 启动需要若干秒，延迟后再跑 demo
    delayed_node = TimerAction(period=6.0, actions=[pick_place_node])

    return LaunchDescription([
        moveit_demo_launch,
        delayed_node,
    ])
