#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger

import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R
import csv


class PathToServoController(Node):
    """
    将 CSV 路径点序列转化为末端速度指令，通过比例控制器发布进行轨迹跟踪。
    【深度解耦版】：
      - 位置：仅在工具 X-Y 平面移动，忽略 Z 轴（法向）误差。
      - 姿态：仅控制绕工具 Z 轴的旋转以对齐前进方向，忽略 X/Y 轴旋转（交由力控贴合表面）。
    """

    def __init__(self):
        super().__init__('path_to_servo_controller')

        # ── 参数声明 ──────────────────────────────────────────
        self.declare_parameter('csv_file', '/home/ras/tracer_jaka/outputs/coverage_path.csv')
        self.declare_parameter('ee_frame',    'tool0')       # 末端执行器 TF 帧名
        self.declare_parameter('base_frame',  'world')       # 基坐标系 TF 帧名
        self.declare_parameter('kp_linear',   5.0)           # 线速度比例增益
        self.declare_parameter('kp_angular',  2.0)           # 角速度比例增益 (转向可以适当给大一点)
        self.declare_parameter('max_linear',  0.8)           # 最大线速度 (m/s)
        self.declare_parameter('max_angular', 0.6)           # 最大角速度 (rad/s)
        self.declare_parameter('goal_tol_pos',  0.005)       # 到达判定: 平面位置容差 (m)
        self.declare_parameter('goal_tol_rot',  0.08)        # 到达判定: 偏航姿态容差 (rad)
        self.declare_parameter('control_rate', 125.0)        # 控制频率 (Hz)

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
        #     TwistStamped, '/tracker_reference_twist', 10)

        self.twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)


        # ── TF 监听器 ─────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── 加载路径点 ────────────────────────────────────────
        csv_file = self.get_parameter('csv_file').value
        self.waypoints = self._load_waypoints(csv_file)
        self.current_idx = 0
        self.tracking_active = False

        if not self.waypoints:
            self.get_logger().error('未加载到任何路径点，节点退出。')
            return

        # ── 启动服务 ──────────────────────────────────────────
        self.start_srv = self.create_service(Trigger, '/path_servo/start', self._start_tracking_cb)
        self.stop_srv  = self.create_service(Trigger, '/path_servo/stop',  self._stop_tracking_cb)

        # ── 控制定时器 ────────────────────────────────────────
        dt = 1.0 / control_rate
        self.timer = self.create_timer(dt, self._control_loop)

        self.get_logger().info(f'节点就绪：共 {len(self.waypoints)} 个路径点。调用 /path_servo/start 开始跟踪。')

    def _load_waypoints(self, filepath):
        positions, normals, directions = [], [], []
        try:
            with open(filepath, 'r') as f:
                for row in csv.DictReader(f):
                    positions.append(np.array([float(row['x']), float(row['y']), float(row['z'])]))
                    normals.append(np.array([float(row['nx']), float(row['ny']), float(row['nz'])]))
                    directions.append(np.array([float(row['dx']), float(row['dy']), float(row['dz'])]))
        except Exception as e:
            self.get_logger().error(f'读取 CSV 失败: {e}')
            return []

        if not positions: return []

        waypoints = self._compute_orientations(positions, normals, directions)
        self.get_logger().info(f'成功加载并计算 {len(waypoints)} 个切向对齐路径点。')
        return waypoints

    def _compute_orientations(self, positions, normals, directions):
        """计算目标姿态: Z 平行法向, Y 平行前进方向"""
        waypoints = []
        for i in range(len(positions)):
            pos, n, d = positions[i], normals[i], directions[i]

            norm_n = np.linalg.norm(n)
            z_axis = n / norm_n if norm_n > 1e-6 else np.array([0.0, 0.0, 1.0])

            norm_d = np.linalg.norm(d)
            fw_dir = d / norm_d if norm_d > 1e-6 else np.array([0.0, 1.0, 0.0])

            y_axis = fw_dir - np.dot(fw_dir, z_axis) * z_axis
            norm_y = np.linalg.norm(y_axis)

            if norm_y < 1e-6:
                ref = np.array([1.0, 0.0, 0.0]) if abs(z_axis[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
                y_axis = np.cross(z_axis, ref)
                y_axis /= np.linalg.norm(y_axis)
            else:
                y_axis /= norm_y

            x_axis = np.cross(y_axis, z_axis)
            x_axis /= np.linalg.norm(x_axis)

            rot_mat = np.column_stack((x_axis, y_axis, z_axis))
            waypoints.append((pos, R.from_matrix(rot_mat).as_quat()))
        return waypoints

    def _get_current_ee_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.05))
            t, r = tf.transform.translation, tf.transform.rotation
            return np.array([t.x, t.y, t.z]), np.array([r.x, r.y, r.z, r.w])
        except Exception:
            return None

    def _clamp(self, vec, max_norm):
        norm = np.linalg.norm(vec)
        return vec if norm <= max_norm else vec * (max_norm / norm)

    # ─────────────────────────────────────────────────────────
    # ★ 核心控制回路 ★
    # ─────────────────────────────────────────────────────────
    def _control_loop(self):
        if not self.tracking_active: return
        if self.current_idx >= len(self.waypoints):
            self.get_logger().info('✅ 轨迹跟踪完成！')
            self.tracking_active = False
            self._publish_zero_twist()
            return

        result = self._get_current_ee_pose()
        if result is None: return
        cur_pos, cur_quat = result
        tgt_pos, tgt_quat = self.waypoints[self.current_idx]

        R_bt = R.from_quat(cur_quat).as_matrix()   # base ← tool
        R_tb = R_bt.T                               # tool ← base

        # ==========================================
        # 1. 位置误差 → 只保留工具 X、Y
        # ==========================================
        pos_err_base = tgt_pos - cur_pos
        pos_err_tool = R_tb @ pos_err_base

        # # ✅ 强制清零工具 Z（交给力控器）
        #pos_err_tool[2] = 0.0

        # 转回基坐标系发出
        pos_err_filtered = R_bt @ pos_err_tool
        linear_vel = self._clamp(self.kp_lin * pos_err_filtered, self.max_lin)

        # ==========================================
        # 2. 姿态误差 → 只保留工具 Rz（Yaw）
        #    使用 Y 轴投影法（上一轮修正的正确方法）
        # ==========================================
        y_cur_world = R_bt[:, 1]
        z_cur_world = R_bt[:, 2]
        y_tgt_world = R.from_quat(tgt_quat).as_matrix()[:, 1]

        y_cur_proj = y_cur_world - np.dot(y_cur_world, z_cur_world) * z_cur_world
        y_tgt_proj = y_tgt_world - np.dot(y_tgt_world, z_cur_world) * z_cur_world

        norm_c = np.linalg.norm(y_cur_proj)
        norm_t = np.linalg.norm(y_tgt_proj)

        if norm_c > 1e-6 and norm_t > 1e-6:
            y_cur_proj /= norm_c
            y_tgt_proj /= norm_t
            sin_yaw = np.dot(np.cross(y_cur_proj, y_tgt_proj), z_cur_world)
            cos_yaw = np.dot(y_cur_proj, y_tgt_proj)
            yaw_err = np.arctan2(sin_yaw, cos_yaw)
        else:
            yaw_err = 0.0

        # ✅ 角速度只绕工具 Z 轴（Rz），Rx/Ry 清零交给力控器
        angular_vel = self._clamp(self.kp_ang * yaw_err * z_cur_world, self.max_ang)

        # ==========================================
        # 3. 容差判断（只判 XY 平面位置 + Yaw）
        # ==========================================
        pos_dist = np.linalg.norm(pos_err_tool[:2])
        rot_dist  = abs(yaw_err)

        if pos_dist < self.tol_pos and rot_dist < self.tol_rot:
            self.current_idx += 1
            return

        # ==========================================
        # 4. 发布（基坐标系下，Z/Rx/Ry 已清零）
        # ==========================================
        twist_msg = TwistStamped()
        twist_msg.header.stamp    = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.base_frame
        twist_msg.twist.linear.x  = float(linear_vel[0])
        twist_msg.twist.linear.y  = float(linear_vel[1])
        twist_msg.twist.linear.z  = float(linear_vel[2])
        twist_msg.twist.angular.x = float(angular_vel[0])
        twist_msg.twist.angular.y = float(angular_vel[1])
        twist_msg.twist.angular.z = float(angular_vel[2])
        self.twist_pub.publish(twist_msg)


    def _publish_zero_twist(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        self.twist_pub.publish(msg)

    def _start_tracking_cb(self, req, res):
        self.current_idx = 0
        self.tracking_active = True
        res.success, res.message = True, 'Tracking Started'
        return res

    def _stop_tracking_cb(self, req, res):
        self.tracking_active = False
        self._publish_zero_twist()
        res.success, res.message = True, 'Tracking Stopped'
        return res

def main(args=None):
    rclpy.init(args=args)
    node = PathToServoController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()