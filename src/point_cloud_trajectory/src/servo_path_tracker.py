#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Path
from std_srvs.srv import Trigger

import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R
import csv


class PathToServoController(Node):
    """
    将 CSV 路径点序列转化为末端速度指令，
    通过比例控制器发布至 /servo_node/delta_twist_cmds 进行轨迹跟踪。
    """

    def __init__(self):
        super().__init__('path_to_servo_controller')

        # ── 参数声明 ──────────────────────────────────────────
        self.declare_parameter('csv_file',
            '/home/ras/tracer_jaka/src/point_cloud_trajectory/pcd/coverage_path.csv')
        self.declare_parameter('ee_frame',    'gripper_center_link')       # 末端执行器 TF 帧名
        self.declare_parameter('base_frame',  'world')       # 基坐标系 TF 帧名
        self.declare_parameter('kp_linear',   15)           # 线速度比例增益
        self.declare_parameter('kp_angular',  1.5)           # 角速度比例增益
        self.declare_parameter('max_linear',  0.4)          # 最大线速度 (m/s)
        self.declare_parameter('max_angular', 0.0)           # 最大角速度 (rad/s)
        self.declare_parameter('goal_tol_pos',  0.005)       # 到达判定: 位置容差 (m)
        self.declare_parameter('goal_tol_rot',  0.05)        # 到达判定: 姿态容差 (rad)
        self.declare_parameter('control_rate', 125.0)         # 控制频率 (Hz)

        self.ee_frame    = self.get_parameter('ee_frame').value
        self.base_frame  = self.get_parameter('base_frame').value
        self.kp_lin      = self.get_parameter('kp_linear').value
        self.kp_ang      = self.get_parameter('kp_angular').value
        self.max_lin     = self.get_parameter('max_linear').value
        self.max_ang     = self.get_parameter('max_angular').value
        self.tol_pos     = self.get_parameter('goal_tol_pos').value
        self.tol_rot     = self.get_parameter('goal_tol_rot').value
        control_rate     = self.get_parameter('control_rate').value

        # ── 发布者 ────────────────────────────────────────────
        # self.twist_pub = self.create_publisher(
        #     TwistStamped, '/servo_node/delta_twist_cmds', 10)
        # 改为
        self.twist_pub = self.create_publisher(
            TwistStamped, '/tracker_reference_twist', 10)

        # ── TF 监听器 ─────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── 加载路径点 ────────────────────────────────────────
        csv_file = self.get_parameter('csv_file').value
        self.waypoints = self._load_waypoints(csv_file)  # list of (pos, quat)
        self.current_idx = 0
        self.tracking_active = False

        if not self.waypoints:
            self.get_logger().error('未加载到任何路径点，节点退出。')
            return

        # ── 启动服务（手动触发开始跟踪）────────────────────────
        self.start_srv = self.create_service(
            Trigger, '/path_servo/start', self._start_tracking_cb)
        self.stop_srv  = self.create_service(
            Trigger, '/path_servo/stop',  self._stop_tracking_cb)

        # ── 控制定时器 ────────────────────────────────────────
        dt = 1.0 / control_rate
        self.timer = self.create_timer(dt, self._control_loop)

        self.get_logger().info(
            f'节点就绪：共 {len(self.waypoints)} 个路径点。'
            f'调用 /path_servo/start 开始跟踪。')

    # ─────────────────────────────────────────────────────────
    # 辅助函数
    # ─────────────────────────────────────────────────────────

    def _load_waypoints(self, filepath):
        """从 CSV 读取路径点，返回 [(position_array, quaternion_array), ...] 列表"""
        waypoints = []
        try:
            with open(filepath, 'r') as f:
                for row in csv.DictReader(f):
                    pos  = np.array([float(row['x']),
                                     float(row['y'])+0.3,
                                     float(row['z'])])
                    # 法向量 → 四元数
                    quat = self._normal_to_quat(
                        float(row['nx']),
                        float(row['ny']),
                        float(row['nz']))
                    waypoints.append((pos, quat))
            self.get_logger().info(f'成功加载 {len(waypoints)} 个路径点。')
        except Exception as e:
            self.get_logger().error(f'读取 CSV 失败: {e}')
        return waypoints

    def _normal_to_quat(self, nx, ny, nz):
        """将曲面法向量转换为末端坐标系，并附加自定义的局部旋转"""
        z_axis = np.array([nx, ny, nz])
        norm   = np.linalg.norm(z_axis)
        if norm < 1e-6:
            return np.array([0.0, 0.0, 0.0, 1.0])
        z_axis /= norm

        # 1. 构造初始旋转矩阵 (Z轴对齐法向量)
        ref = np.array([1.0, 0.0, 0.0]) if abs(z_axis[0]) < 0.9 \
              else np.array([0.0, 1.0, 0.0])
        x_axis = np.cross(ref, z_axis)
        x_axis /= np.linalg.norm(x_axis)
        y_axis  = np.cross(z_axis, x_axis)

        rot_mat = np.column_stack((x_axis, y_axis, z_axis))
        base_rot = R.from_matrix(rot_mat)

        # 2. 定义附加的局部旋转 (Intrinsic rotations)
        # 'zx' (小写) 表示内旋：先绕局部Z轴旋转，再绕新的局部X轴旋转
        # 顺时针通常遵循右手定则为负，即 -45 度
        extra_rot = R.from_euler('zx', [-90, 180], degrees=True)

        # 3. 矩阵右乘表示局部旋转叠加: R_final = R_base * R_extra
        final_rot = base_rot * extra_rot

        return final_rot.as_quat()   # [x, y, z, w]

    def _get_current_ee_pose(self):
        """通过 TF 查询当前末端位姿，返回 (pos, quat) 或 None"""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
            t = tf.transform.translation
            r = tf.transform.rotation
            pos  = np.array([t.x, t.y, t.z])
            quat = np.array([r.x, r.y, r.z, r.w])
            return pos, quat
        except Exception as e:
            self.get_logger().warn(f'TF 查询失败: {e}', throttle_duration_sec=2.0)
            return None

    def _clamp(self, vec, max_norm):
        """将向量模限制在 max_norm 以内"""
        norm = np.linalg.norm(vec)
        return vec if norm <= max_norm else vec * (max_norm / norm)

    # ─────────────────────────────────────────────────────────
    # 控制回路
    # ─────────────────────────────────────────────────────────

    def _control_loop(self):
        if not self.tracking_active:
            return
        if self.current_idx >= len(self.waypoints):
            self.get_logger().info('✅ 所有路径点已完成！')
            self.tracking_active = False
            self._publish_zero_twist()
            return

        # 1. 获取当前末端位姿
        result = self._get_current_ee_pose()
        if result is None:
            return
        cur_pos, cur_quat = result

        # 2. 取目标路径点
        tgt_pos, tgt_quat = self.waypoints[self.current_idx]

        # 3. 计算位置误差 → 线速度
        pos_err_base = tgt_pos - cur_pos
        
        # ★ 关键：将位置误差投影到工具坐标系，清零力控轴 ★
        R_base_tool = R.from_quat(cur_quat).as_matrix()  # 3x3
        R_tool_base = R_base_tool.T
       
        pos_err_tool = R_tool_base @ pos_err_base
        
            # 力控轴（工具Z）不由路径跟踪器修正
        pos_err_tool[2] = 0.0
        
            # 变换回 base frame 计算速度
        pos_err_filtered = R_base_tool @ pos_err_tool

        
        linear_vel = self._clamp(self.kp_lin * pos_err_filtered, self.max_lin)

        # 4. 计算姿态误差 → 角速度
        #    q_err = q_target ⊗ q_current^{-1}
        q_cur = R.from_quat(cur_quat)
        q_tgt = R.from_quat(tgt_quat)
        q_err = q_tgt * q_cur.inv()
        q_err_vec = q_err.as_quat()                          # [x, y, z, w]
        # 保证最短路径（w >= 0）
        if q_err_vec[3] < 0:
            q_err_vec = -q_err_vec
        ang_err = q_err_vec[:3]                              # 取虚部
        angular_vel = self._clamp(2.0 * self.kp_ang * ang_err, self.max_ang)

        # 5. 判断是否到达当前路径点
        pos_dist = np.linalg.norm(pos_err_filtered)  # 而非 pos_err_base
        rot_dist = 2.0 * np.linalg.norm(ang_err)            # 近似轴角误差 (rad)

        # if pos_dist < self.tol_pos and rot_dist < self.tol_rot:
        #     self.get_logger().info(
        #         f'✔ 到达路径点 {self.current_idx + 1}/{len(self.waypoints)}'
        #         f'  pos_err={pos_dist*1000:.1f}mm  rot_err={np.degrees(rot_dist):.1f}°')
        #     self.current_idx += 1
        #     return                                           # 下一周期切换新目标

        if pos_dist < self.tol_pos:
            self.get_logger().info(
                f'✔ 到达路径点 {self.current_idx + 1}/{len(self.waypoints)}'
                f'  pos_err={pos_dist*1000:.1f}mm  rot_err={np.degrees(rot_dist):.1f}°')
            self.current_idx += 1
            return           

        # 6. 封装并发布 TwistStamped
        twist_msg = TwistStamped()
        twist_msg.header.stamp    = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.base_frame          # 在基坐标系下表达速度

        twist_msg.twist.linear.x  = float(linear_vel[0])
        twist_msg.twist.linear.y  = float(linear_vel[1])
        twist_msg.twist.linear.z  = float(linear_vel[2])
        twist_msg.twist.angular.x = float(angular_vel[0])
        twist_msg.twist.angular.y = float(angular_vel[1])
        twist_msg.twist.angular.z = float(angular_vel[2])

        self.twist_pub.publish(twist_msg)

    def _publish_zero_twist(self):
        """发布零速度以停止机械臂"""
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        self.twist_pub.publish(msg)

    # ─────────────────────────────────────────────────────────
    # 服务回调
    # ─────────────────────────────────────────────────────────

    def _start_tracking_cb(self, request, response):
        self.current_idx = 0
        self.tracking_active = True
        self.get_logger().info('▶ 路径跟踪已启动。')
        response.success = True
        response.message = f'开始跟踪，共 {len(self.waypoints)} 个路径点。'
        return response

    def _stop_tracking_cb(self, request, response):
        self.tracking_active = False
        self._publish_zero_twist()
        self.get_logger().info('⏹ 路径跟踪已停止。')
        response.success = True
        response.message = '跟踪已中止，已发送零速度指令。'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PathToServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点被用户中断。')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
