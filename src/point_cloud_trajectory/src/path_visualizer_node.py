#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker
import csv
import numpy as np
from scipy.spatial.transform import Rotation as R
import open3d as o3d

class CoverageAndPcdVisualizer(Node):
    def __init__(self):
        super().__init__('coverage_and_pcd_visualizer')
        
        # --- 1. 参数与发布者初始化 ---
        self.path_pub = self.create_publisher(Path, '/coverage_path', 10)
        self.pose_pub = self.create_publisher(PoseArray, '/coverage_poses', 10)
        self.pcd_pub = self.create_publisher(PointCloud2, '/filtered_cloud', 10)
        
        # ★ 发布青色法线段 (Normal Z-axis)
        self.normal_marker_pub = self.create_publisher(Marker, '/coverage_normals_marker', 10)
        # ★ 发布黄色方向段 (Direction X-axis)
        self.direction_marker_pub = self.create_publisher(Marker, '/coverage_directions_marker', 10)
        
        self.frame_id = "world"
        
        # --- 2. 准备消息对象 ---
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id
        
        self.pose_array_msg = PoseArray()
        self.pose_array_msg.header.frame_id = self.frame_id

        # --- 初始化法线 Marker (青色) ---
        self.normals_marker = Marker()
        self.normals_marker.header.frame_id = self.frame_id
        self.normals_marker.ns = "normals"
        self.normals_marker.id = 0
        self.normals_marker.type = Marker.LINE_LIST
        self.normals_marker.action = Marker.ADD
        self.normals_marker.scale.x = 0.0015         # 线宽
        self.normals_marker.color.r = 0.0            # 青色
        self.normals_marker.color.g = 1.0
        self.normals_marker.color.b = 1.0
        self.normals_marker.color.a = 1.0

        # --- 初始化方向 Marker (黄色) ---
        self.directions_marker = Marker()
        self.directions_marker.header.frame_id = self.frame_id
        self.directions_marker.ns = "directions"
        self.directions_marker.id = 1                # ID 必须不同
        self.directions_marker.type = Marker.LINE_LIST
        self.directions_marker.action = Marker.ADD
        self.directions_marker.scale.x = 0.0015      # 线宽
        self.directions_marker.color.r = 1.0         # 黄色
        self.directions_marker.color.g = 1.0
        self.directions_marker.color.b = 0.0
        self.directions_marker.color.a = 1.0
        
        # --- 3. 读取 CSV 路径数据 ---
        csv_file = '/home/ras/tracer_jaka/src/point_cloud_trajectory/pcd/coverage_path.csv'
        self.load_csv_data(csv_file)
        
        # --- 4. 读取 PCD 点云数据并预处理 ---
        pcd_file = '/home/ras/tracer_jaka/src/point_cloud_trajectory/pcd/cropped_cloud.pcd'
        self.pcd_msg = self.load_and_transform_pcd(pcd_file, offset_x=0.0, offset_y=0.31, offset_z=0.0)
        
        # --- 5. 设置定时器 (1Hz 同步发布) ---
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('节点已启动！正在发布双 Marker、路径、位姿及点云...')

    def compute_6dof_quaternion(self, nx, ny, nz, dx, dy, dz):
        """
        利用法向量(n)和前进方向(d)，计算严格正交的 6-DoF 四元数
        工具坐标系定义: Z 轴 = 表面法向，X 轴 = 轨迹前进方向
        """
        # 1. Z 轴 = 表面法向 (严格遵循表面)
        z_axis = np.array([nx, ny, nz])
        norm_z = np.linalg.norm(z_axis)
        if norm_z < 1e-6:
            z_axis = np.array([0.0, 0.0, 1.0])
        else:
            z_axis = z_axis / norm_z

        # 2. 初步 X 轴 = 前进方向
        x_init = np.array([dx, dy, dz])
        norm_x = np.linalg.norm(x_init)
        if norm_x < 1e-6:
            x_init = np.array([1.0, 0.0, 0.0])
        else:
            x_init = x_init / norm_x

        # 3. Y 轴 = Z 叉乘 初始X (垂直于法面和前进方向)
        y_axis = np.cross(z_axis, x_init)
        norm_y = np.linalg.norm(y_axis)
        
        if norm_y < 1e-6:
            # 万一前进方向和法向量平行（极小概率），使用默认 Y 轴
            fallback_axis = np.array([0, 1, 0]) if abs(z_axis[2]) < 0.9 else np.array([1, 0, 0])
            y_axis = np.cross(z_axis, fallback_axis)
            y_axis = y_axis / np.linalg.norm(y_axis)
        else:
            y_axis = y_axis / norm_y

        # 4. 严格正交的最终 X 轴 = Y 叉乘 Z
        x_axis = np.cross(y_axis, z_axis)

        # 5. 组合旋转矩阵并转换为四元数
        rot_mat = np.column_stack((x_axis, y_axis, z_axis))
        return R.from_matrix(rot_mat).as_quat()

    def load_csv_data(self, filepath):
        """加载 CSV 并构建 Path, PoseArray 以及双 Normal Marker"""
        try:
            with open(filepath, 'r') as f:
                reader = csv.DictReader(f)
                all_rows = list(reader)
                
            total_points = len(all_rows)
            if total_points == 0:
                return

            # 降采样步长，如果点太密可以改成 2 或 3
            step = 1  
            sliced_rows = all_rows[::step]
            
            display_length = 0.02 # 线段显示长度 (2cm)

            for row in sliced_rows:
                # 坐标
                x, y, z = float(row['x']), float(row['y']) + 0.3, float(row['z'])
                # 法向量
                nx, ny, nz = float(row['nx']), float(row['ny']), float(row['nz'])
                # 前进方向 (如果 CSV 里没有会报错，请确保 Python 规划脚本成功导出了 dx,dy,dz)
                dx, dy, dz = float(row['dx']), float(row['dy']), float(row['dz'])
                
                # 1. Path 消息
                pose_stamped = PoseStamped()
                pose_stamped.pose.position.x = x
                pose_stamped.pose.position.y = y
                pose_stamped.pose.position.z = z
                pose_stamped.pose.orientation.w = 1.0
                self.path_msg.poses.append(pose_stamped)
                
                # 2. PoseArray 消息 (使用最新 6DoF 姿态计算)
                q_vals = self.compute_6dof_quaternion(nx, ny, nz, dx, dy, dz)
                pose_msg = Pose()
                pose_msg.position.x = x
                pose_msg.position.y = y
                pose_msg.position.z = z
                pose_msg.orientation.x = q_vals[0]
                pose_msg.orientation.y = q_vals[1]
                pose_msg.orientation.z = q_vals[2]
                pose_msg.orientation.w = q_vals[3]
                self.pose_array_msg.poses.append(pose_msg)
                
                # 3. Marker 共同的起点
                p_start = Point(x=x, y=y, z=z)
                
                # ★ 法向 Marker (青色)
                p_end_normal = Point()
                p_end_normal.x = x + nx * display_length
                p_end_normal.y = y + ny * display_length
                p_end_normal.z = z + nz * display_length
                self.normals_marker.points.extend([p_start, p_end_normal])

                # ★ 方向 Marker (黄色) - 可以比法向稍微短一点，以免视觉混淆
                dir_len = display_length * 0.8 
                p_end_dir = Point()
                p_end_dir.x = x + dx * dir_len
                p_end_dir.y = y + dy * dir_len
                p_end_dir.z = z + dz * dir_len
                self.directions_marker.points.extend([p_start, p_end_dir])
                    
            self.get_logger().info(f'成功加载 {len(self.path_msg.poses)} 个轨迹点及其法向/方向数据。')
        except KeyError as e:
            self.get_logger().error(f'CSV 缺失列: {e}。请确认已运行最新的 Python 规划代码并生成了 dx, dy, dz。')
        except Exception as e:
            self.get_logger().error(f'读取 CSV 失败: {e}')

    def load_and_transform_pcd(self, filepath, offset_x, offset_y, offset_z):
        """使用 Open3D 读取点云，应用平移，并转换为 PointCloud2 消息"""
        try:
            pcd = o3d.io.read_point_cloud(filepath)
            if pcd.is_empty():
                self.get_logger().error(f'PCD file is empty or not found: {filepath}')
                return PointCloud2()

            transform = np.eye(4)
            transform[0, 3] = offset_x
            transform[1, 3] = offset_y
            transform[2, 3] = offset_z
            pcd.transform(transform)

            points = np.asarray(pcd.points, dtype=np.float32)

            msg = PointCloud2()
            msg.header.frame_id = self.frame_id
            msg.height = 1
            msg.width = len(points)
            msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            ]
            msg.is_bigendian = False
            msg.point_step = 12
            msg.row_step = 12 * points.shape[0]
            msg.is_dense = True
            msg.data = points.tobytes()
            
            return msg
        except Exception as e:
            self.get_logger().error(f'Failed to load PCD: {e}')
            return PointCloud2()

    def timer_callback(self):
        current_time = self.get_clock().now().to_msg()
        
        # 更新时间戳
        self.path_msg.header.stamp = current_time
        self.pose_array_msg.header.stamp = current_time
        self.normals_marker.header.stamp = current_time
        self.directions_marker.header.stamp = current_time
        self.pcd_msg.header.stamp = current_time
        
        # 发布
        self.path_pub.publish(self.path_msg)
        self.pose_pub.publish(self.pose_array_msg)
        self.normal_marker_pub.publish(self.normals_marker)
        self.direction_marker_pub.publish(self.directions_marker)
        
        if len(self.pcd_msg.data) > 0:
            self.pcd_pub.publish(self.pcd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CoverageAndPcdVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly.')
    except Exception as e:
        node.get_logger().error(f'Exception in node: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
