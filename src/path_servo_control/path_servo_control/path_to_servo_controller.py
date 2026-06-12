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

    修改版：
      - 位置：仅在工具 X-Y 平面移动，忽略工具 Z 轴误差。
      - 姿态：不再逐点跟踪 CSV 中每个点的法向量。
      - 姿态：统计 CSV 中全部 nx/ny/nz，直接求平均，得到一个固定法向量。
      - 姿态：整条轨迹都让末端工具 Z 轴对齐该平均法向量。
      - 不进行法向量反向处理。
      - 绕工具 Z 轴的自旋 yaw 不控制。
    """

    def __init__(self):
        super().__init__('path_to_servo_controller')

        # ── 参数声明 ──────────────────────────────────────────
        self.declare_parameter(
            'csv_file',
            '/home/ras/tracer_jaka/outputs/coverage_path.csv'
        )
        self.declare_parameter('ee_frame', 'gripper_center_link')
        self.declare_parameter('base_frame', 'world')

        self.declare_parameter('kp_linear', 15.0)
        self.declare_parameter('kp_angular', 20.0)

        self.declare_parameter('max_linear', 1.2)
        self.declare_parameter('max_angular', 0.8)

        self.declare_parameter('goal_tol_pos', 0.005)
        self.declare_parameter('goal_tol_rot', 0.08)

        self.declare_parameter('control_rate', 125.0)

        # ── 读取参数 ──────────────────────────────────────────
        self.ee_frame = self.get_parameter('ee_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.kp_lin = self.get_parameter('kp_linear').value
        self.kp_ang = self.get_parameter('kp_angular').value

        self.max_lin = self.get_parameter('max_linear').value
        self.max_ang = self.get_parameter('max_angular').value

        self.tol_pos = self.get_parameter('goal_tol_pos').value
        self.tol_rot = self.get_parameter('goal_tol_rot').value

        control_rate = self.get_parameter('control_rate').value

        # 全局固定平均法向量
        self.fixed_normal = None

        # ── 发布者 ────────────────────────────────────────────
        # 如果你使用 MoveIt Servo，可以改回：
        # self.twist_pub = self.create_publisher(
        #     TwistStamped,
        #     '/servo_node/delta_twist_cmds',
        #     10
        # )

        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/tracker_reference_twist',
            10
        )

        # ── TF 监听器 ─────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # ── 加载路径点 ────────────────────────────────────────
        csv_file = self.get_parameter('csv_file').value
        self.waypoints = self._load_waypoints(csv_file)

        self.current_idx = 0
        self.tracking_active = False

        if not self.waypoints:
            self.get_logger().error('未加载到任何路径点，节点退出。')
            return

        if self.fixed_normal is None:
            self.get_logger().error('未成功计算固定平均法向量，节点退出。')
            return

        # ── 启动 / 停止服务 ───────────────────────────────────
        self.start_srv = self.create_service(
            Trigger,
            '/path_servo/start',
            self._start_tracking_cb
        )

        self.stop_srv = self.create_service(
            Trigger,
            '/path_servo/stop',
            self._stop_tracking_cb
        )

        # ── 控制定时器 ────────────────────────────────────────
        dt = 1.0 / control_rate
        self.timer = self.create_timer(dt, self._control_loop)

        self.get_logger().info(
            f'节点就绪：共 {len(self.waypoints)} 个路径点。'
            f'固定平均法向量 = '
            f'[{self.fixed_normal[0]:.4f}, '
            f'{self.fixed_normal[1]:.4f}, '
            f'{self.fixed_normal[2]:.4f}]。'
            f'调用 /path_servo/start 开始跟踪。'
        )

    def _load_waypoints(self, filepath):
        """
        从 CSV 中读取位置和法向量。

        CSV 需要包含：
          x, y, z, nx, ny, nz

        其中：
          x, y, z 用于路径点位置跟踪；
          nx, ny, nz 会被全部统计并求平均，得到一个固定法向量。
        """
        positions = []
        normals = []

        try:
            with open(filepath, 'r') as f:
                reader = csv.DictReader(f)

                for row in reader:
                    pos = np.array([
                        float(row['x']),
                        float(row['y']),
                        float(row['z'])
                    ], dtype=float)

                    normal = np.array([
                        float(row['nx']),
                        float(row['ny']),
                        float(row['nz'])
                    ], dtype=float)

                    positions.append(pos)
                    normals.append(normal)

        except Exception as e:
            self.get_logger().error(f'读取 CSV 失败: {e}')
            return []

        if not positions:
            self.get_logger().error('CSV 中没有有效路径点。')
            return []

        waypoints = self._compute_waypoints_and_fixed_normal(
            positions,
            normals
        )

        self.get_logger().info(
            f'成功加载 {len(waypoints)} 个路径点，并完成平均法向量计算。'
        )

        return waypoints

    def _compute_waypoints_and_fixed_normal(self, positions, normals):
        """
        计算路径点，并根据全部法向量得到一个固定平均法向量。

        注意：
          - 不做法向量反向翻转；
          - 每个有效法向量先归一化；
          - 然后直接求平均；
          - 平均结果再归一化，作为 self.fixed_normal。
        """
        waypoints = []
        valid_normals = []

        for pos, n in zip(positions, normals):
            norm_n = np.linalg.norm(n)

            if norm_n > 1e-6:
                n_axis = n / norm_n
            else:
                self.get_logger().warn(
                    '检测到零法向量，该点法向量不参与平均，位置仍保留。'
                )
                n_axis = None

            waypoints.append(pos)

            if n_axis is not None:
                valid_normals.append(n_axis)

        self.fixed_normal = self._compute_average_normal(valid_normals)

        self._print_normal_statistics(valid_normals, self.fixed_normal)

        return waypoints

    def _compute_average_normal(self, normals):
        """
        直接对全部归一化法向量求平均。

        不进行正反方向修正。
        """
        if not normals:
            self.get_logger().warn(
                '没有有效法向量，默认 fixed_normal = [0, 0, 1]'
            )
            return np.array([0.0, 0.0, 1.0], dtype=float)

        mean_normal = np.mean(normals, axis=0)
        norm_mean = np.linalg.norm(mean_normal)

        if norm_mean < 1e-6:
            self.get_logger().warn(
                '平均法向量接近零，可能是法向量方向分布相互抵消。'
                '默认使用 [0, 0, 1]。'
            )
            return np.array([0.0, 0.0, 1.0], dtype=float)

        fixed_normal = mean_normal / norm_mean

        self.get_logger().info(
            f'平均法向量计算完成：'
            f'raw_mean = '
            f'[{mean_normal[0]:.4f}, '
            f'{mean_normal[1]:.4f}, '
            f'{mean_normal[2]:.4f}], '
            f'normalized = '
            f'[{fixed_normal[0]:.4f}, '
            f'{fixed_normal[1]:.4f}, '
            f'{fixed_normal[2]:.4f}]'
        )

        return fixed_normal

    def _print_normal_statistics(self, normals, fixed_normal):
        """
        打印所有法向量相对于平均法向量的角度统计。

        这里只做统计，不影响控制。
        """
        if not normals or fixed_normal is None:
            return

        angles = []

        for n in normals:
            cos_angle = np.clip(
                np.dot(n, fixed_normal),
                -1.0,
                1.0
            )
            angle = np.arccos(cos_angle)
            angles.append(angle)

        angles = np.array(angles)

        mean_deg = np.degrees(np.mean(angles))
        max_deg = np.degrees(np.max(angles))
        std_deg = np.degrees(np.std(angles))

        self.get_logger().info(
            f'法向量统计：'
            f'mean_err = {mean_deg:.2f} deg, '
            f'max_err = {max_deg:.2f} deg, '
            f'std_err = {std_deg:.2f} deg'
        )

        if max_deg > 20.0:
            self.get_logger().warn(
                '法向量分散较大，使用固定平均姿态可能会产生较大局部姿态误差。'
            )

    def _get_current_ee_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )

            t = tf.transform.translation
            r = tf.transform.rotation

            cur_pos = np.array([t.x, t.y, t.z], dtype=float)
            cur_quat = np.array([r.x, r.y, r.z, r.w], dtype=float)

            return cur_pos, cur_quat

        except Exception:
            return None

    def _clamp(self, vec, max_norm):
        norm = np.linalg.norm(vec)

        if norm <= max_norm:
            return vec

        if norm < 1e-9:
            return vec

        return vec * (max_norm / norm)

    # ─────────────────────────────────────────────────────────
    # 核心控制回路
    # ─────────────────────────────────────────────────────────
    def _control_loop(self):
        if not self.tracking_active:
            return

        if self.current_idx >= len(self.waypoints):
            self.get_logger().info('✅ 轨迹跟踪完成！')
            self.tracking_active = False
            self._publish_zero_twist()
            return

        result = self._get_current_ee_pose()

        if result is None:
            return

        cur_pos, cur_quat = result

        # 当前目标路径点位置
        tgt_pos = self.waypoints[self.current_idx]

        # 整条轨迹统一使用固定平均法向量
        tgt_normal = self.fixed_normal

        R_bt = R.from_quat(cur_quat).as_matrix()   # base ← tool
        R_tb = R_bt.T                              # tool ← base

        # ==========================================
        # 1. 位置误差：只保留工具 X-Y 平面
        # ==========================================
        pos_err_base = tgt_pos - cur_pos

        # 把位置误差转到工具坐标系
        pos_err_tool = R_tb @ pos_err_base

        # 工具 Z 方向交给力控器，不由轨迹跟踪控制
        pos_err_tool[2] = 0.0

        # 再转回基坐标系发布
        pos_err_filtered = R_bt @ pos_err_tool

        linear_vel = self._clamp(
            self.kp_lin * pos_err_filtered,
            self.max_lin
        )

        # ==========================================
        # 2. 姿态误差：工具 Z 轴对齐固定平均法向量
        # ==========================================
        z_cur_world = R_bt[:, 2]          # 当前工具 Z 轴，基坐标系表达
        n_tgt_world = tgt_normal         # 固定平均法向量，基坐标系表达

        rot_axis = np.cross(z_cur_world, n_tgt_world)

        sin_angle = np.linalg.norm(rot_axis)
        cos_angle = np.clip(
            np.dot(z_cur_world, n_tgt_world),
            -1.0,
            1.0
        )

        normal_err = np.arctan2(sin_angle, cos_angle)

        if sin_angle > 1e-6:
            rot_axis /= sin_angle

            angular_vel = self._clamp(
                self.kp_ang * normal_err * rot_axis,
                self.max_ang
            )
        else:
            angular_vel = np.zeros(3, dtype=float)

        # ==========================================
        # 3. 到达判断：工具 X-Y 平面位置 + 固定法向角度
        # ==========================================
        pos_dist = np.linalg.norm(pos_err_tool[:2])
        rot_dist = abs(normal_err)

        if pos_dist < self.tol_pos and rot_dist < self.tol_rot:
            self.current_idx += 1
            return

        # ==========================================
        # 4. 发布 TwistStamped
        # ==========================================
        twist_msg = TwistStamped()

        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.base_frame

        twist_msg.twist.linear.x = float(linear_vel[0])
        twist_msg.twist.linear.y = float(linear_vel[1])
        twist_msg.twist.linear.z = float(linear_vel[2])

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

        res.success = True
        res.message = 'Tracking Started'

        self.get_logger().info('轨迹跟踪开始。')

        return res

    def _stop_tracking_cb(self, req, res):
        self.tracking_active = False
        self._publish_zero_twist()

        res.success = True
        res.message = 'Tracking Stopped'

        self.get_logger().info('轨迹跟踪停止。')

        return res


def main(args=None):
    rclpy.init(args=args)

    node = PathToServoController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
