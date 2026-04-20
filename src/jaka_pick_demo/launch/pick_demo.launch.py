from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="tracer_jaka",
            package_name="tracer_jaka_moveit_config",
        )
        .robot_description(file_path="config/tracer_jaka_zu5.urdf.xacro")
        .robot_description_semantic(file_path="config/tracer_jaka_zu5.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    jaka_pick_demo_node = Node(
        package="jaka_pick_demo",
        executable="jaka_pick_demo_node",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription([
        jaka_pick_demo_node,
    ])
