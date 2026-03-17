#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from sensor_msgs.msg import PointCloud2, PointField
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
        
        self.frame_id = "world"
        
        # --- 2. 准备消息对象 ---
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id
        
        self.pose_array_msg = PoseArray()
        self.pose_array_msg.header.frame_id = self.frame_id
        
        # --- 3. 读取 CSV 路径数据 ---
        csv_file = '/home/ras/tracer_jaka/src/point_cloud_trajectory/pcd/coverage_path.csv'
        self.load_csv_data(csv_file)
        
        # --- 4. 读取 PCD 点云数据并预处理 ---
        pcd_file = '/home/ras/tracer_jaka/src/point_cloud_trajectory/pcd/filtered.pcd'
        self.pcd_msg = self.load_and_transform_pcd(pcd_file, offset_x=0.0, offset_y=0.0, offset_z=0.0)
        
        # --- 5. 设置定时器 (1Hz 同步发布) ---
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Unified visualizer node started. Publishing Path, Poses, and PointCloud to RViz2...')

    def normal_to_quaternion(self, nx, ny, nz):
        """将法向量转换为四元数 [x, y, z, w]"""
        z_axis = np.array([nx, ny, nz])
        norm_z = np.linalg.norm(z_axis)
        if norm_z < 1e-6:
            return [0.0, 0.0, 0.0, 1.0]
        z_axis = z_axis / norm_z

        if abs(z_axis[0]) < 0.9:
            x_axis = np.cross(np.array([1, 0, 0]), z_axis)
        else:
            x_axis = np.cross(np.array([0, 1, 0]), z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)

        rot_mat = np.column_stack((x_axis, y_axis, z_axis))
        return R.from_matrix(rot_mat).as_quat()

    def load_csv_data(self, filepath):
        """加载 CSV 并构建 Path 和 PoseArray 消息"""
        try:
            with open(filepath, 'r') as f:
                reader = csv.DictReader(f)
                all_rows = list(reader)
                
            total_points = len(all_rows)
            if total_points == 0:
                return

            start_idx = int(total_points * 0)
            end_idx = int(total_points * 1)
            sliced_rows = all_rows[start_idx:end_idx]

            for row in sliced_rows:
                x, y, z = float(row['x']), float(row['y']) + 0.3, float(row['z'])
                nx, ny, nz = float(row['nx']), float(row['ny']), float(row['nz'])
                
                # Path
                pose_stamped = PoseStamped()
                pose_stamped.pose.position.x = x
                pose_stamped.pose.position.y = y
                pose_stamped.pose.position.z = z
                pose_stamped.pose.orientation.w = 1.0
                self.path_msg.poses.append(pose_stamped)
                
                # PoseArray
                q_vals = self.normal_to_quaternion(nx, ny, nz)
                pose_msg = Pose()
                pose_msg.position.x = x
                pose_msg.position.y = y
                pose_msg.position.z = z
                pose_msg.orientation.x = q_vals[0]
                pose_msg.orientation.y = q_vals[1]
                pose_msg.orientation.z = q_vals[2]
                pose_msg.orientation.w = q_vals[3]
                self.pose_array_msg.poses.append(pose_msg)
                    
            self.get_logger().info(f'Loaded {len(self.path_msg.poses)} path points.')
        except Exception as e:
            self.get_logger().error(f'Failed to read CSV: {e}')

    def load_and_transform_pcd(self, filepath, offset_x, offset_y, offset_z):
        """使用 Open3D 读取点云，应用平移，并转换为 PointCloud2 消息"""
        try:
            pcd = o3d.io.read_point_cloud(filepath)
            if pcd.is_empty():
                self.get_logger().error(f'PCD file is empty or not found: {filepath}')
                return PointCloud2()

            # 应用平移变换 (对应 C++ 中的 Eigen::Matrix4f 变换)
            transform = np.eye(4)
            transform[0, 3] = offset_x
            transform[1, 3] = offset_y
            transform[2, 3] = offset_z
            pcd.transform(transform)

            points = np.asarray(pcd.points, dtype=np.float32)
            self.get_logger().info(f'Loaded {len(points)} points from PCD and applied offset.')

            # 将 Numpy 数组打包为 ROS 2 的 PointCloud2 消息格式
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
        # 统一获取当前时间戳
        current_time = self.get_clock().now().to_msg()
        
        # 更新时间戳并发布
        self.path_msg.header.stamp = current_time
        self.pose_array_msg.header.stamp = current_time
        self.pcd_msg.header.stamp = current_time
        
        self.path_pub.publish(self.path_msg)
        self.pose_pub.publish(self.pose_array_msg)
        
        # 如果点云数据不为空，则发布
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
