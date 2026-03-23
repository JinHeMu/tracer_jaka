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


#include "jaka_hardware_interface/jaka_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <thread>

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

 // 1. 获取 Robot IP
  auto it = info_.hardware_parameters.find("robot_ip");
  if (it != info_.hardware_parameters.end())
  {
    robot_ip_ = it->second;
  }
  else
  {
    RCLCPP_FATAL(rclcpp::get_logger("JakaHardwareInterface"), "Parameter'robot_ip' not set");
    return CallbackReturn::ERROR;
  }

  // 2. 获取 Local IP (EDG 必需)
  auto it_local = info_.hardware_parameters.find("local_ip");
  if (it_local != info_.hardware_parameters.end())
  {
    local_ip_ = it_local->second;
  }
  else
  {
    RCLCPP_FATAL(rclcpp::get_logger("JakaHardwareInterface"), "Parameter'local_ip' not set (Required for EDG)");
    return CallbackReturn::ERROR;
  }

    // 3. 获取力传感器 TCP 偏移参数与旋转参数 (带有默认值的安全解析)
  auto it_off_x = info_.hardware_parameters.find("sensor_to_tcp_offset_x");
  sensor_to_tcp_offset_[0] = (it_off_x != info_.hardware_parameters.end()) ? std::stod(it_off_x->second) : 0.0;

  auto it_off_y = info_.hardware_parameters.find("sensor_to_tcp_offset_y");
  sensor_to_tcp_offset_[1] = (it_off_y != info_.hardware_parameters.end()) ? std::stod(it_off_y->second) : 0.0;

  auto it_off_z = info_.hardware_parameters.find("sensor_to_tcp_offset_z");
  sensor_to_tcp_offset_[2] = (it_off_z != info_.hardware_parameters.end()) ? std::stod(it_off_z->second) : 0.18; // 默认 0.18m

  auto it_angle = info_.hardware_parameters.find("sensor_z_angle");
  sensor_z_angle_ = (it_angle != info_.hardware_parameters.end()) ? std::stod(it_angle->second) : (M_PI / 4.0); // 默认 45 度

  RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"), 
    "FTS Config - Offset: [%.3f, %.3f, %.3f], Z-Angle: %.3f rad", 
    sensor_to_tcp_offset_[0], sensor_to_tcp_offset_[1], sensor_to_tcp_offset_[2], sensor_z_angle_);



  // 3. 初始化存储向量
  hw_position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocity_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_position_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  
  // === 修改：FTS 相关初始化 ===
  hw_fts_states_.resize(6, 0.0);    // 输出给ROS的数据 (初始为0)
  hw_fts_raw_.resize(6, 0.0);       // 原始数据容器
  ft_bias_.resize(6, 0.0);          // 偏置容器
  bias_initialized_ = false;        // 重置标志位
  

  RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"), 
    "Jaka EDG Interface Init: Robot=%s, Local=%s", robot_ip_.c_str(), local_ip_.c_str());

  return CallbackReturn::SUCCESS;
}


// 初始化实际机器人
hardware_interface::CallbackReturn JakaHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to be ready for read calls and write calls of some interfaces

RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"), "Connecting to robot...");

  // 1. 登录
  if (robot_.login_in(robot_ip_.c_str()) != ERR_SUCC) {
    RCLCPP_ERROR(rclcpp::get_logger("JakaHardwareInterface"), "Login failed!");
    return CallbackReturn::ERROR;
  }

  // 2. 上电与使能 (建议在外部或脚本中完成，也可在此处取消注释)
  // robot_.power_on();
  // std::this_thread::sleep_for(std::chrono::seconds(5));
  // robot_.enable_robot();
  // std::this_thread::sleep_for(std::chrono::seconds(5));

  // 3. 初始化 EDG 模式
  // 参数: true(开启), 本机IP, 端口(默认10010), 模式(0:全功能)
  RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"), "Initializing EDG UDP Stream...");
  if (robot_.edg_init(true, local_ip_.c_str(), 10010, 0) != ERR_SUCC) {
     RCLCPP_ERROR(rclcpp::get_logger("JakaHardwareInterface"), "Failed to init EDG! Check firewall/IP.");
     return CallbackReturn::ERROR;
  }
  
  // 给一点时间让 UDP 数据流建立
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // 4. 读取初始状态 (使用 EDG 接口)
  // 尝试读取几次直到成功，确保数据流正常
  int retry_count = 0;
  while(true) {
      if (robot_.edg_get_stat(&edg_state_) == ERR_SUCC) {
          break;
      }
      retry_count++;
      if(retry_count > 20) {
          RCLCPP_ERROR(rclcpp::get_logger("JakaHardwareInterface"), "Timeout waiting for initial EDG data.");
          return CallbackReturn::ERROR;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // 5. 同步 Command 和 State，防止启动飞车
  for (size_t i = 0; i < info_.joints.size() && i < 6; ++i) {
    hw_position_states_[i] = edg_state_.jointVal.jVal[i];
    hw_velocity_states_[i] = edg_state_.jointVel.jVel[i]; // rad/s
    
    hw_position_commands_[i] = hw_position_states_[i]; // 初始指令 = 当前位置
    
    RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"), "Joint %zu init pos: %.4f", i, hw_position_states_[i]);
  }


  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> JakaHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    // 导出 Position
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_states_[i]));
    
    // 导出 Velocity (你的 XML 中有定义)
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_states_[i]));

  }

  if (info_.sensors.size() > 0) {
      const auto& sensor = info_.sensors[0]; // 假设只有一个力传感器
      
      // 注意：这里的顺序必须和 hw_fts_states_ 的存储顺序一致
      // 通常是 Fx, Fy, Fz, Tx, Ty, Tz
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        sensor.name, "force.x", &hw_fts_states_[0]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        sensor.name, "force.y", &hw_fts_states_[1]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        sensor.name, "force.z", &hw_fts_states_[2]));
      
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        sensor.name, "torque.x", &hw_fts_states_[3]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        sensor.name, "torque.y", &hw_fts_states_[4]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        sensor.name, "torque.z", &hw_fts_states_[5]));
        
     RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"), 
        "Exported FT Sensor interfaces for: %s", sensor.name.c_str());
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> JakaHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;


  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    // 只导出 Position Command
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn JakaHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"), "Activating... (Ensuring EDG is running)");
  robot_.servo_move_enable(true);

  // 建议：每次 Activate 都重新校准一次零点
  // 这样如果之前校准歪了，重启一下 Controller 就好了
  bias_initialized_ = false; 
  std::fill(hw_fts_states_.begin(), hw_fts_states_.end(), 0.0);


  // 可以在这里再次确保 EDG 开启，或者重置状态
  // 再次同步，因为从 Configure 到 Activate 可能有时间差
  robot_.edg_get_stat(&edg_state_);
  for (size_t i = 0; i < info_.joints.size() && i < 6; ++i) {
      hw_position_commands_[i] = edg_state_.jointVal.jVal[i];
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JakaHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to stop receiving commands

  RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"), "Deactivating... Stopping EDG");
  
  robot_.servo_move_enable(false);
  // 关闭 EDG 模式
  robot_.edg_init(false);

  

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type JakaHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 1. 使用 EDG 接口读取全量数据
  errno_t ret = robot_.edg_get_stat(&edg_state_);

  if (ret == ERR_SUCC) 
  {
    // A. 更新关节状态
    for (size_t i = 0; i < info_.joints.size() && i < 6; ++i) {
      hw_position_states_[i] = edg_state_.jointVal.jVal[i];
      hw_velocity_states_[i] = edg_state_.jointVel.jVel[i];
    }

    // B. 更新力传感器数据 (使用固定偏置补偿)
    if (hw_fts_states_.size() == 6) {
        
        // --- 1. 获取原始数据 ---
        hw_fts_raw_[0] = edg_state_.torqSensor.fx;
        hw_fts_raw_[1] = edg_state_.torqSensor.fy;
        hw_fts_raw_[2] = edg_state_.torqSensor.fz;
        hw_fts_raw_[3] = edg_state_.torqSensor.tx;
        hw_fts_raw_[4] = edg_state_.torqSensor.ty;
        hw_fts_raw_[5] = edg_state_.torqSensor.tz;

        // --- 2. 设置固定偏置 (Fixed Bias) ---
        // 注意：根据你的要求，这里直接硬编码提供的测量值
        // Force Bias [x,y,z]: [-9.80, -6.78, -6.00] (N)
        // Torque Bias [x,y,z]: [0.54, -0.63, 0.03] (Nm)
        ft_bias_[0] = -9.80;
        ft_bias_[1] = -6.78;
        ft_bias_[2] = -6.00;
        ft_bias_[3] = 0.54;
        ft_bias_[4] = -0.63;
        ft_bias_[5] = 0.03;

        // --- 3. 计算最终接触力 (带坐标系旋转与 TCP 杠杆补偿) ---
        // 3.1 提取纯净的传感器原生数据 (Raw - Bias)
        double fx_pure = hw_fts_raw_[0] - ft_bias_[0];
        double fy_pure = hw_fts_raw_[1] - ft_bias_[1];
        double fz_pure = hw_fts_raw_[2] - ft_bias_[2];
        double tx_pure = hw_fts_raw_[3] - ft_bias_[3];
        double ty_pure = hw_fts_raw_[4] - ft_bias_[4];
        double tz_pure = hw_fts_raw_[5] - ft_bias_[5];

        // 3.2 绕 Z 轴旋转对齐 (例如 45 度)
        const double c = std::cos(sensor_z_angle_);
        const double s = std::sin(sensor_z_angle_);

        double fx_rot = fx_pure * c - fy_pure * s;
        double fy_rot = fx_pure * s + fy_pure * c;
        double fz_rot = fz_pure; 

        double tx_rot = tx_pure * c - ty_pure * s;
        double ty_rot = tx_pure * s + ty_pure * c;
        double tz_rot = tz_pure; 

        // 3.3 TCP 杠杆补偿 
        // 补偿公式: M_tcp = M_rot - (R x F_rot)
        double rx = sensor_to_tcp_offset_[0];
        double ry = sensor_to_tcp_offset_[1];
        double rz = sensor_to_tcp_offset_[2];

        // 修正后的 TCP 力矩输出 (考虑了旋转后的力和力矩)
        double tcp_tx = tx_rot - (ry * fz_rot - rz * fy_rot);
        double tcp_ty = ty_rot - (rz * fx_rot - rx * fz_rot);
        double tcp_tz = tz_rot - (rx * fy_rot - ry * fx_rot);

        // 3.4 打包数据并进行后期处理
        std::array<double, 6> compensated_ft = {
            fx_rot, fy_rot, fz_rot, 
            tcp_tx, tcp_ty, tcp_tz
        };

        for (int i = 0; i < 6; ++i) {
            double raw_val = compensated_ft[i];
            
            // 1. 低通滤波
            hw_fts_states_[i] = filter_alpha_ * raw_val + (1.0 - filter_alpha_) * hw_fts_states_[i];

            // 2. 死区处理 (注意：你之前的代码阈值是0，建议根据实际抖动设置为如 0.5 或 1.0)
            const double deadband = (i < 3) ? 1.0 : 0.2; // 示例：力 0.2N, 力矩 0.02Nm
            if (std::abs(hw_fts_states_[i]) < deadband) {
                hw_fts_states_[i] = 0.0;
            }
        }
      }
  } else {
    // 读取失败处理：例如保持旧值或设为0
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type JakaHardwareInterface::write(
  const rclcpp::Time & , const rclcpp::Duration & /*period*/)
{

  // 准备指令数据
  for (size_t i = 0; i < info_.joints.size() && i < 6; ++i) {
    // NaN 检查
    if (std::isnan(hw_position_commands_[i])) {
        joint_cmd_.jVal[i] = hw_position_states_[i];
    } else {
        joint_cmd_.jVal[i] = hw_position_commands_[i];
    }
  }

  // printf("write joint value: %f, %f, %f, %f, %f, %f\n", 
  //   joint_cmd_.jVal[0], joint_cmd_.jVal[1], joint_cmd_.jVal[2], 
  //   joint_cmd_.jVal[3], joint_cmd_.jVal[4], joint_cmd_.jVal[5]);
  // 使用 EDG 伺服接口发送指令
  // MoveMode::ABS (绝对位置)
  // step_num = 1 (通常设为1，表示立即执行)
  
  static std::array<double,6> last_sent{};
  static bool inited=false;

  double eps = 1e-5; // 约0.0001 rad ≈ 0.0057°
  bool changed=false;
  for (int i=0;i<6;i++){
    if (!inited || std::fabs(joint_cmd_.jVal[i]-last_sent[i])>eps) { changed=true; }
  }

  if (changed) {
      robot_.edg_servo_j(&joint_cmd_, MoveMode::ABS, 1);
    // printf("write joint value: %f, %f, %f, %f, %f, %f\n", 
    //   joint_cmd_.jVal[0], joint_cmd_.jVal[1], joint_cmd_.jVal[2], 
    //   joint_cmd_.jVal[3], joint_cmd_.jVal[4], joint_cmd_.jVal[5]);
    for (int i=0;i<6;i++) last_sent[i]=joint_cmd_.jVal[i];
    inited=true;
  } else {
    // 不发 or 降频发（比如每 10 个周期发一次）
  }


  return hardware_interface::return_type::OK;
}

}

  // namespace jaka_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  jaka_hardware_interface::JakaHardwareInterface, hardware_interface::SystemInterface)
