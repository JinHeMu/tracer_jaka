// Copyright 2024, Force Admittance Servo Node
// SPDX-License-Identifier: Apache-2.0

#include "force_admittance_servo/force_admittance_servo_node.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace force_admittance_servo
{

// ─── 辅助：从参数名字中解析轴索引的字符串向量 ───────────────────────────────────
static std::array<AxisMode, 6> parseModeArray(const std::vector<int64_t> & v)
{
  if (v.size() != 6) {
    throw std::runtime_error("mode array must have exactly 6 elements");
  }
  std::array<AxisMode, 6> out;
  for (int i = 0; i < 6; ++i) {
    switch (v[i]) {
      case 0: out[i] = AxisMode::DISABLED;      break;
      case 1: out[i] = AxisMode::ADMITTANCE;    break;
      case 2: out[i] = AxisMode::FORCE_CONTROL; break;
      default:
        throw std::runtime_error("invalid axis mode: " + std::to_string(v[i]));
    }
  }
  return out;
}

static std::array<double, 6> toArray6(const std::vector<double> & v)
{
  if (v.size() != 6) {
    throw std::runtime_error("expected array of length 6, got " + std::to_string(v.size()));
  }
  std::array<double, 6> out;
  std::copy(v.begin(), v.end(), out.begin());
  return out;
}

// ─────────────────────────────────────────────────────────────────────────────
//  构造 & 初始化
// ─────────────────────────────────────────────────────────────────────────────

ForceAdmittanceServoNode::ForceAdmittanceServoNode(const rclcpp::NodeOptions & options)
: Node("force_admittance_servo", options)
{
  declareParameters();
  loadParameters();
  setupTopics();
  resetState();

  // 注册动态参数回调
  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ForceAdmittanceServoNode::onParameterChange, this, std::placeholders::_1));

  // 控制定时器
  dt_ = 1.0 / ctrl_rate_;
  ctrl_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(dt_),
    std::bind(&ForceAdmittanceServoNode::controlLoop, this));

  RCLCPP_INFO(this->get_logger(),
    "ForceAdmittanceServoNode 启动，控制频率 %.1f Hz，坐标系: %s",
    ctrl_rate_, control_frame_id_.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
//  参数声明
// ─────────────────────────────────────────────────────────────────────────────

void ForceAdmittanceServoNode::declareParameters()
{
  // ── 话题 ──
  this->declare_parameter<std::string>(
    "wrench_topic", "/tcp_fts_sensor/wrench");
  this->declare_parameter<std::string>(
    "servo_twist_topic", "/servo_node/delta_twist_cmds");
  this->declare_parameter<std::string>(
    "enable_topic", "~/enable");
  this->declare_parameter<std::string>(
    "control_frame_id", "base_link");

  // ── 频率 ──
  this->declare_parameter<double>("control_rate", 125.0);

  // ── 每轴模式
  //   0 = DISABLED, 1 = ADMITTANCE, 2 = FORCE_CONTROL
  //   顺序: [x, y, z, rx, ry, rz]（控制坐标系）
  this->declare_parameter<std::vector<int64_t>>(
    "axis_mode", {1, 1, 2, 0, 0, 0});

  // ── 动力学参数（每轴）──
  this->declare_parameter<std::vector<double>>(
    "mass",       {1.0, 1.0, 1.0, 0.1, 0.1, 0.1});
  this->declare_parameter<std::vector<double>>(
    "damping",    {50.0, 50.0, 50.0, 5.0, 5.0, 5.0});
  this->declare_parameter<std::vector<double>>(
    "stiffness",  {200.0, 200.0, 200.0, 20.0, 20.0, 20.0});

  // ── 恒力目标（仅 FORCE_CONTROL 轴生效）──
  this->declare_parameter<std::vector<double>>(
    "target_wrench", {0.0, 0.0, -5.0, 0.0, 0.0, 0.0});

  // ── 速度限幅 ──
  this->declare_parameter<double>("max_linear_vel",  0.1);
  this->declare_parameter<double>("max_angular_vel", 0.5);
}

// ─────────────────────────────────────────────────────────────────────────────
//  参数加载
// ─────────────────────────────────────────────────────────────────────────────

void ForceAdmittanceServoNode::loadParameters()
{
  ctrl_rate_        = this->get_parameter("control_rate").as_double();
  control_frame_id_ = this->get_parameter("control_frame_id").as_string();

  auto mode_vec = this->get_parameter("axis_mode").as_integer_array();
  params_.mode = parseModeArray(mode_vec);

  params_.mass          = toArray6(this->get_parameter("mass").as_double_array());
  params_.damping       = toArray6(this->get_parameter("damping").as_double_array());
  params_.stiffness     = toArray6(this->get_parameter("stiffness").as_double_array());
  params_.target_wrench = toArray6(this->get_parameter("target_wrench").as_double_array());

  params_.max_linear_vel  = this->get_parameter("max_linear_vel").as_double();
  params_.max_angular_vel = this->get_parameter("max_angular_vel").as_double();
}

// ─────────────────────────────────────────────────────────────────────────────
//  话题注册
// ─────────────────────────────────────────────────────────────────────────────

void ForceAdmittanceServoNode::setupTopics()
{
  const auto wrench_topic = this->get_parameter("wrench_topic").as_string();
  const auto twist_topic  = this->get_parameter("servo_twist_topic").as_string();
  const auto enable_topic = this->get_parameter("enable_topic").as_string();

  wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    wrench_topic, rclcpp::SensorDataQoS(),
    std::bind(&ForceAdmittanceServoNode::wrenchCallback, this, std::placeholders::_1));

  enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    enable_topic, 10,
    std::bind(&ForceAdmittanceServoNode::enableCallback, this, std::placeholders::_1));

  twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    twist_topic, rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(this->get_logger(), "订阅力矩话题: %s", wrench_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "发布 Servo 速度话题: %s", twist_topic.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
//  状态重置
// ─────────────────────────────────────────────────────────────────────────────

void ForceAdmittanceServoNode::resetState()
{
  for (auto & s : axis_state_) {
    s.position = 0.0;
    s.velocity = 0.0;
    s.accel    = 0.0;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  传感器回调
// ─────────────────────────────────────────────────────────────────────────────

void ForceAdmittanceServoNode::wrenchCallback(
  const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(wrench_mutex_);
  latest_wrench_  = msg->wrench;
  wrench_received_ = true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  使能话题回调
// ─────────────────────────────────────────────────────────────────────────────

void ForceAdmittanceServoNode::enableCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data && enabled_) {
    RCLCPP_INFO(this->get_logger(), "控制器已禁用，重置积分状态");
    resetState();
  } else if (msg->data && !enabled_) {
    RCLCPP_INFO(this->get_logger(), "控制器已启用");
    resetState();
  }
  enabled_ = msg->data;
}

// ─────────────────────────────────────────────────────────────────────────────
//  单轴控制律
//
//  参数说明：
//   axis       — 轴索引 [0,5]
//   force_ctrl — 该轴在控制坐标系下的测量力（已补偿，纯净值）
//   state      — 该轴当前积分状态
//   p          — 控制参数
//
//  返回：该轴加速度 x_ddot
// ─────────────────────────────────────────────────────────────────────────────

double ForceAdmittanceServoNode::computeAxisAccel(
  int axis, double force_ctrl, const AxisState & state, const ControlParams & p) const
{
  const double m_inv = (p.mass[axis] > 1e-9) ? (1.0 / p.mass[axis]) : 0.0;
  const double d     = p.damping[axis];
  const double k     = p.stiffness[axis];

  switch (p.mode[axis]) {
    case AxisMode::DISABLED:
      return 0.0;

    case AxisMode::ADMITTANCE:
      // 导纳：跟随外力，有刚度回弹
      // M * x_ddot = F_ext - D * x_dot - K * x
      return m_inv * (force_ctrl - d * state.velocity - k * state.position);

    case AxisMode::FORCE_CONTROL:
      // 恒力：维持目标接触力
      // 符号约定：F_ext 为环境对机器人的反力，接触时与施加方向相反
      // M * x_ddot = (F_target + F_ext) - D * x_dot
      //   → 当 |F_ext| < |F_target| 时，残差驱动机器人继续施力
      return m_inv * (p.target_wrench[axis] + force_ctrl - d * state.velocity);

    default:
      return 0.0;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  主控制循环
// ─────────────────────────────────────────────────────────────────────────────

void ForceAdmittanceServoNode::controlLoop()
{
  if (!enabled_) {
    // 使能关闭时持续发布零速，防止 Servo 超时报警
    Eigen::Matrix<double, 6, 1> zero_vel = Eigen::Matrix<double, 6, 1>::Zero();
    publishTwist(zero_vel);
    return;
  }

  if (!wrench_received_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "尚未收到力矩传感器数据，等待中...");
    return;
  }

  // ── 取最新力矩数据（线程安全）──────────────────────────────────────────────
  geometry_msgs::msg::Wrench wrench;
  {
    std::lock_guard<std::mutex> lock(wrench_mutex_);
    wrench = latest_wrench_;
  }

  // 将力矩数据打包为 6 维向量（控制坐标系，这里假设传感器坐标系已与控制坐标系对齐）
  // 如需坐标变换，在此处旋转该向量
  const std::array<double, 6> F = {
    wrench.force.x,  wrench.force.y,  wrench.force.z,
    wrench.torque.x, wrench.torque.y, wrench.torque.z
  };

  // const std::array<double, 6> F = {
  //   0.0, 0.0, 0.0,
  //   0.0, 0.0, 0.0
  // };


  // ── 逐轴计算并积分 ──────────────────────────────────────────────────────────
  Eigen::Matrix<double, 6, 1> vel_out;

  for (int i = 0; i < 6; ++i) {
    if (params_.mode[i] == AxisMode::DISABLED) {
      axis_state_[i] = {0.0, 0.0, 0.0};
      vel_out[i]     = 0.0;
      continue;
    }

    // 控制律 → 加速度
    const double accel = computeAxisAccel(i, F[i], axis_state_[i], params_);

    // 欧拉积分
    axis_state_[i].velocity += accel * dt_;
    axis_state_[i].position += axis_state_[i].velocity * dt_;
    axis_state_[i].accel     = accel;

    vel_out[i] = axis_state_[i].velocity;
  }

  // ── 限幅 & 发布 ─────────────────────────────────────────────────────────────
  publishTwist(vel_out);
}

// ─────────────────────────────────────────────────────────────────────────────
//  发布 TwistStamped → MoveIt Servo
// ─────────────────────────────────────────────────────────────────────────────

void ForceAdmittanceServoNode::publishTwist(const Eigen::Matrix<double, 6, 1> & vel)
{
  // 线速度限幅
  double vx = std::clamp(vel[0], -params_.max_linear_vel,  params_.max_linear_vel);
  double vy = std::clamp(vel[1], -params_.max_linear_vel,  params_.max_linear_vel);
  double vz = std::clamp(vel[2], -params_.max_linear_vel,  params_.max_linear_vel);
  // 角速度限幅
  double wx = std::clamp(vel[3], -params_.max_angular_vel, params_.max_angular_vel);
  double wy = std::clamp(vel[4], -params_.max_angular_vel, params_.max_angular_vel);
  double wz = std::clamp(vel[5], -params_.max_angular_vel, params_.max_angular_vel);

  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp    = this->now();
  msg.header.frame_id = control_frame_id_;   // MoveIt Servo 按此 frame 解释速度
  msg.twist.linear.x  = vx;
  msg.twist.linear.y  = vy;
  msg.twist.linear.z  = vz;
  msg.twist.angular.x = wx;
  msg.twist.angular.y = wy;
  msg.twist.angular.z = wz;

  twist_pub_->publish(msg);
}

// ─────────────────────────────────────────────────────────────────────────────
//  动态参数回调（运行时修改 target_wrench / mode 等，无需重启）
// ─────────────────────────────────────────────────────────────────────────────

rcl_interfaces::msg::SetParametersResult
ForceAdmittanceServoNode::onParameterChange(const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & p : params) {
    try {
      if (p.get_name() == "axis_mode") {
        params_.mode = parseModeArray(p.as_integer_array());
        RCLCPP_INFO(this->get_logger(), "axis_mode 已更新");
        resetState();
      } else if (p.get_name() == "mass") {
        params_.mass = toArray6(p.as_double_array());
      } else if (p.get_name() == "damping") {
        params_.damping = toArray6(p.as_double_array());
      } else if (p.get_name() == "stiffness") {
        params_.stiffness = toArray6(p.as_double_array());
      } else if (p.get_name() == "target_wrench") {
        params_.target_wrench = toArray6(p.as_double_array());
        RCLCPP_INFO(this->get_logger(), "target_wrench 已更新: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
          params_.target_wrench[0], params_.target_wrench[1], params_.target_wrench[2],
          params_.target_wrench[3], params_.target_wrench[4], params_.target_wrench[5]);
      } else if (p.get_name() == "max_linear_vel") {
        params_.max_linear_vel = p.as_double();
      } else if (p.get_name() == "max_angular_vel") {
        params_.max_angular_vel = p.as_double();
      }
    } catch (const std::exception & e) {
      result.successful = false;
      result.reason     = e.what();
      RCLCPP_ERROR(this->get_logger(), "参数更新失败: %s", e.what());
    }
  }
  return result;
}

}  // namespace force_admittance_servo

