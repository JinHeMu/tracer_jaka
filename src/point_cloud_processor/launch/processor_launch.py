"""
processor_launch.py
====================
启动 point_cloud_processor_node，并加载 config/params.yaml 参数文件。

用法
----
# 使用默认参数文件
ros2 launch point_cloud_processor processor_launch.py

# 覆盖单个参数
ros2 launch point_cloud_processor processor_launch.py \
    params_file:=/path/to/my_params.yaml

# 启用自动流水线
ros2 launch point_cloud_processor processor_launch.py \
    auto_pipeline:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── 参数声明 ──────────────────────────────────────────────
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('point_cloud_processor'),
            'config',
            'params.yaml'
        ]),
        description='参数配置文件路径（YAML）'
    )

    auto_pipeline_arg = DeclareLaunchArgument(
        'auto_pipeline',
        default_value='false',
        description='启动后自动执行完整流水线（true/false）'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='日志级别：DEBUG / INFO / WARN / ERROR'
    )

    # ── 节点定义 ──────────────────────────────────────────────
    processor_node = Node(
        package='point_cloud_processor',
        executable='point_cloud_processor_node',
        name='point_cloud_processor',
        output='screen',
        # 加载 YAML 参数文件
        parameters=[
            LaunchConfiguration('params_file'),
            # 命令行覆盖（优先级高于 YAML）
            {
                'node.auto_pipeline': LaunchConfiguration('auto_pipeline'),
                'node.log_level':     LaunchConfiguration('log_level'),
            }
        ],
        # 将 ROS 日志级别映射到 launch 参数
        arguments=['--ros-args', '--log-level',
                   LaunchConfiguration('log_level')],
        emulate_tty=True,
    )

    return LaunchDescription([
        params_file_arg,
        auto_pipeline_arg,
        log_level_arg,
        LogInfo(msg=">>> 启动 PointCloudProcessorNode <<<"),
        processor_node,
    ])
