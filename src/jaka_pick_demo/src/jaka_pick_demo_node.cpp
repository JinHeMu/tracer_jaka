#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // MoveIt 2 官方教程推荐这样创建 node
    auto node = std::make_shared<rclcpp::Node>(
        "moveit2_demo_node",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto logger = rclcpp::get_logger("moveit2_demo_node");
    RCLCPP_INFO(logger, "MoveIt2 Demo Node Started");

    // 用单独线程 spin，保证 MoveGroupInterface/action/service 回调能正常处理
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    using moveit::planning_interface::MoveGroupInterface;

    // 创建 MoveGroupInterface
    // "arm" 改成你自己 MoveIt 配置里的 planning group
    MoveGroupInterface move_group(node, "jaka_arm");

    // 一些常用参数
    move_group.setPlanningTime(5.0);
    move_group.setNumPlanningAttempts(5);
    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.setMaxAccelerationScalingFactor(0.1);

    RCLCPP_INFO(logger, "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(logger, "End effector link: %s", move_group.getEndEffectorLink().c_str());

    geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();


    // 目标位姿
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = current_pose.pose.orientation.w;
    target_pose.orientation.x = current_pose.pose.orientation.x;
    target_pose.orientation.y = current_pose.pose.orientation.y;
    target_pose.orientation.z = current_pose.pose.orientation.z;
    target_pose.position.x = current_pose.pose.position.x + 0.1;
    target_pose.position.y = current_pose.pose.position.y;
    target_pose.position.z = current_pose.pose.position.z;


    move_group.setPoseTarget(target_pose);

     move_group.setNamedTarget("up");

    // 规划
    MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group.plan(plan));

    if (success)
    {
        RCLCPP_INFO(logger, "Planning successful, executing...");
        auto result = move_group.execute(plan);

        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(logger, "Execution succeeded.");
        }
        else
        {
            RCLCPP_ERROR(logger, "Execution failed.");
        }
    }
    else
    {
        RCLCPP_ERROR(logger, "Planning failed!");
    }

    // 清理目标，避免影响后续动作
    move_group.clearPoseTargets();

    executor.cancel();
    if (spinner.joinable())
    {
        spinner.join();
    }

    rclcpp::shutdown();
    return 0;
}
