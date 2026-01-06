#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char  *argv[])
{
    ros::init(argc, argv, "jaka_pose_demo");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 创建对象arm连接到xarm规划组
    moveit::planning_interface::MoveGroupInterface arm("arm");
    std::string planning_frame = arm.getPlanningFrame();
    ROS_INFO_STREAM("Planning frame : "<< planning_frame);
    std::string eef_link = arm.getEndEffectorLink();
    ROS_INFO_STREAM("End effector link : "<< eef_link);

    arm.allowReplanning(true);
    arm.setGoalPositionTolerance(0.02);
    arm.setGoalOrientationTolerance(0.03);
    arm.setMaxVelocityScalingFactor(0.1);
    arm.setMaxAccelerationScalingFactor(0.1);


    arm.setStartStateToCurrentState();


    geometry_msgs::PoseStamped current_pose = arm.getCurrentPose();

    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = "world";
    target_pose.header.stamp = ros::Time::now();
    target_pose.pose.position.x = current_pose.pose.position.x;
    target_pose.pose.position.y = current_pose.pose.position.y;
    target_pose.pose.position.z = current_pose.pose.position.z-0.1;
    target_pose.pose.orientation.x = current_pose.pose.orientation.x;
    target_pose.pose.orientation.y = current_pose.pose.orientation.y;
    target_pose.pose.orientation.z = current_pose.pose.orientation.z;
    target_pose.pose.orientation.w = current_pose.pose.orientation.w;

    ROS_INFO("Moving to target_pose ...");
    arm.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success = (arm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("moveit_pose_demo", "Visualizing plan 1 (joint space goal) %s", success ? "" : "FAILED");

    if (success)
    {
        arm.execute(plan);
    
    }

    // geometry_msgs::PoseStamped current_pose = arm.getCurrentPose();
    // std::vector<double> current_joint_positions = arm.getCurrentJointValues();
    // ROS_INFO("Move forward 5 cm  ...");
    // target_pose = current_pose;
    // target_pose.pose.position.x += 0.2;
    // arm.setPoseTarget(target_pose);


    ros::shutdown();
    

    return 0;
}
