// Copyright 2024, Force Admittance Servo Node
// SPDX-License-Identifier: Apache-2.0
//
// 恒力/导纳控制节点：按轴可选控制模式，输出末端速度给 MoveIt Servo
//
// 控制律：
//   导纳模式：M_i * x_ddot_i = F_i - D_i * x_dot_i - K_i * x_i
//   z轴恒力PI：v_z = Kp * force_error + Ki * integral(force_error)
//
// 符号约定：
//   F_i         — 传感器测量到的外力/环境反力
//   F_target_i — 期望施加力
//   平衡条件：F_target + F_measured = 0

#pragma once

#include <array>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Dense>

namespace force_admittance_servo
{

enum class AxisMode : int
{
  DISABLED       = 0,
  ADMITTANCE     = 1,
  FORCE_CONTROL  = 2,
};

struct AxisState
{
  double position = 0.0;
  double velocity = 0.0;
  double accel    = 0.0;
};

struct ControlParams
{
  std::array<AxisMode, 6> mode = {AxisMode::DISABLED};

  std::array<double, 6> mass = {
    1.0, 1.0, 1.0,
    0.1, 0.1, 0.1
  };

  std::array<double, 6> damping = {
    50.0, 50.0, 50.0,
    5.0, 5.0, 5.0
  };

  std::array<double, 6> stiffness = {
    200.0, 200.0, 200.0,
    20.0, 20.0, 20.0
  };

  std::array<double, 6> target_wrench = {
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0
  };

  double max_linear_vel    = 0.1;
  double max_angular_vel   = 0.5;
  double max_linear_accel  = 0.5;
  double max_angular_accel = 1.0;

  // z轴恒力 PI 控制参数
  double z_force_kp = 0.003;              // [m/(s*N)]
  double z_force_ki = 0.0000;             // [m/(s*N*s)]
  double z_force_integral_limit = 10.0;   // [N*s]
};

class ForceAdmittanceServoNode : public rclcpp::Node
{
public:
  explicit ForceAdmittanceServoNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  bool getToolRotation(Eigen::Matrix3d & R_base_tool);

  void declareParameters();
  void loadParameters();
  void setupTopics();

  void wrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  void joyTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void trackerTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void enableCallback(const std_msgs::msg::Bool::SharedPtr msg);

  void controlLoop();

  double computeAxisAccel(
    int axis,
    double force_ctrl,
    const AxisState & state,
    const ControlParams & p) const;

  void publishTwist(const Eigen::Matrix<double, 6, 1> & vel);
  void resetState();

  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & params);

private:
  ControlParams params_;
  std::array<AxisState, 6> axis_state_;

  Eigen::Matrix<double, 6, 1> prev_vel_out_ =
    Eigen::Matrix<double, 6, 1>::Zero();

  // z轴恒力 PI 积分项
  double z_force_error_integral_ = 0.0;

  std::mutex wrench_mutex_;
  std::mutex joy_mutex_;
  std::mutex tracker_mutex_;

  geometry_msgs::msg::Wrench latest_wrench_{};
  geometry_msgs::msg::Twist latest_joy_twist_{};
  geometry_msgs::msg::Twist latest_tracker_twist_{};

  bool wrench_received_ = false;
  bool joy_received_ = false;
  bool tracker_received_ = false;

  bool enabled_ = true;

  double dt_ = 0.008;
  double ctrl_rate_ = 125.0;

  std::string control_frame_id_ = "base_link";
  std::string ee_frame_id_ = "gripper_center_link";

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr joy_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr tracker_sub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::TimerBase::SharedPtr ctrl_timer_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace force_admittance_servo
