// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <thread>

#include "jaka_hardware_interface/jaka_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std;

namespace jaka_hardware_interface
{
hardware_interface::CallbackReturn JakaHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // 1. 获取参数: Robot IP
  // 在 URDF 的 <hardware> 标签下配置: <param name="robot_ip">192.168.1.100</param>
  auto it = info_.hardware_parameters.find("robot_ip");
  if (it != info_.hardware_parameters.end())
  {
    robot_ip_ = it->second;
  }
  else
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("JakaHardwareInterface"),
      "Parameter'robot_ip' not set in URDF/ros2_control tag");
    return CallbackReturn::ERROR;
  }


  // TODO(anyone): read parameters and initialize the hardware
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  
  RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"), "Initialized Jaka Interface with IP: %s", robot_ip_.c_str());

  return CallbackReturn::SUCCESS;
}


// 初始化实际机器人
hardware_interface::CallbackReturn JakaHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to be ready for read calls and write calls of some interfaces

  // 1. 登录机器人
  if (robot_.login_in(robot_ip_.c_str()) != ERR_SUCC) {
    RCLCPP_ERROR(rclcpp::get_logger("JakaHardwareInterface"), "Failed to login to robot at %s", robot_ip_.c_str());
    return CallbackReturn::ERROR;
  }
  // // // Turn off servo at startup
  // robot_.servo_move_enable(false);
  // std::this_thread::sleep_for(chrono::milliseconds(500));

  // // // Filter param
  // robot_.servo_move_use_joint_LPF(5);

  // // // Power on + enable
  // RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"), "Powering on...");
  // robot_.power_on();
  // std::this_thread::sleep_for(std::chrono::seconds(8)); 
  // RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"), "Enabling robot...");
  // robot_.enable_robot();
  // std::this_thread::sleep_for(std::chrono::seconds(4)); 

  // 读取初始位置以同步 Command 和 State
  // 这一点至关重要：防止在控制器启动瞬间，command为0导致机器人猛冲
  if (robot_.get_joint_position(&joint_position_fb_) != ERR_SUCC) {
    RCLCPP_ERROR(rclcpp::get_logger("JakaHardwareInterface"), "Failed to get initial joint position");
    return CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < info_.joints.size() && i < 6; ++i) {
    hw_states_[i] = joint_position_fb_.jVal[i];
    hw_commands_[i] = hw_states_[i]; 
    RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"), "Joint %zu initial pos: %f", i, hw_states_[i]);
  }
  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> JakaHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> JakaHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;


  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      // TODO(anyone): insert correct interfaces
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn JakaHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
 RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"), "Activating Servo Mode...");

  // 再次同步位置，确保安全
  robot_.get_joint_position(&joint_position_fb_);
  for (size_t i = 0; i < info_.joints.size() && i < 6; ++i) {
     hw_states_[i] = joint_position_fb_.jVal[i];
     hw_commands_[i] = hw_states_[i];
  }

  // 开启 Servo 模式 (实时控制模式)
  if (robot_.servo_move_enable(true) != ERR_SUCC) {
    RCLCPP_ERROR(rclcpp::get_logger("JakaHardwareInterface"), "Failed to enable servo mode");
    return CallbackReturn::ERROR;
  }
  
  // 等待伺服模式稳定
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"), "System Activated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JakaHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to stop receiving commands

  // errno_t ret = robot_.servo_move_enable(FALSE);
  RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"), "Deactivating Servo Mode...");

  // 关闭 Servo 模式
  robot_.servo_move_enable(false);
  // robot_.disable_robot();
  // robot_.power_off();


  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type JakaHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(anyone): read robot states
  //printf("get joint value!\n");



  if (robot_.get_joint_position(&joint_position_fb_) == ERR_SUCC) {
    for (size_t i = 0; i < info_.joints.size() && i < 6; ++i) {
      hw_states_[i] = joint_position_fb_.jVal[i];
    }
  } else {
    // 读取失败处理，通常打印 Warning 并保持旧值
    // RCLCPP_WARN_THROTTLE(rclcpp::get_logger("JakaHardwareInterface"), *this->get_clock(), 1000, "Failed to read joint state");
  }

  // for (size_t i = 0; i < hw_commands_.size(); ++i) {
  //   printf("j%zu:%.2f\t", i + 1, hw_states_[i]);
  // }
  // printf("\n");


  return hardware_interface::return_type::OK;
}

hardware_interface::return_type JakaHardwareInterface::write(
  const rclcpp::Time & , const rclcpp::Duration & /*period*/)
{

  // for (size_t i = 0; i < hw_commands_.size(); ++i) {
  //   printf("j%zu:%.10f\t", i + 1, hw_commands_[i]);
  // }
  // printf("\n");

   // 将 ROS 的 Command 填入 SDK 的结构体
  for (size_t i = 0; i < info_.joints.size() && i < 6; ++i) {
    // 安全检查：如果命令是 NaN，则不发送（或保持当前位置）
    if (std::isnan(hw_commands_[i])) {
        joint_position_cmd_.jVal[i] = hw_states_[i];
    } else {
        joint_position_cmd_.jVal[i] = hw_commands_[i];
    }
  }

  // 发送伺服控制指令
  // servo_j: 关节空间伺服运动
  // ABS: 绝对位置模式 (Increment 为相对模式)
  // step_num: 1 (倍周期，通常填1)
  errno_t ret = robot_.servo_j(&joint_position_cmd_, MoveMode::ABS, 1);

  if (ret != ERR_SUCC) {
    // 写入失败通常意味着网络抖动或指令不平滑（速度/加速度超限）
    // 可以在这里做错误计数
  }

  return hardware_interface::return_type::OK;
}

}

  // namespace jaka_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  jaka_hardware_interface::JakaHardwareInterface, hardware_interface::SystemInterface)

