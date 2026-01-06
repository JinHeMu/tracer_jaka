#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/Constraints.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "keep_level_with_constraints");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("arm");

    // 当前姿态 → 端平水
    geometry_msgs::Pose start_pose = arm.getCurrentPose().pose;

    // 1. 设置最终移动
    geometry_msgs::Pose target_pose = start_pose;
    target_pose.position.x -= 0.5;
    target_pose.position.z += 0.0;

    arm.setPoseTarget(target_pose);

    // 2. 创建姿态约束
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = arm.getEndEffectorLink();
    ocm.header.frame_id = "base_footprint";
    ocm.orientation = start_pose.orientation;   // 端平水姿态
    ocm.absolute_x_axis_tolerance = 0.01;  // Roll 平水
    ocm.absolute_y_axis_tolerance = 3.14;  // Pitch 平水
    ocm.absolute_z_axis_tolerance = 0.01;  // 允许 yaw 旋转
    ocm.weight = 1.0;

    // 3. 添加到 Constraints 中
    moveit_msgs::Constraints constraints;
    constraints.orientation_constraints.push_back(ocm);

    // 4. 设置 MoveGroup 的路径约束
    arm.setPathConstraints(constraints);

    // 5. 规划
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (arm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if(success) arm.execute(plan);

    // 清除约束
    arm.clearPathConstraints();

    ros::shutdown();
    return 0;
}
