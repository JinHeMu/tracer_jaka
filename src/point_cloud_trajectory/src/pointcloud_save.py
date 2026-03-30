import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

import open3d as o3d
import numpy as np
import time
import os

class PointCloudColorSaver(Node):
    def __init__(self):
        super().__init__('pcd_color_saver')
        
        # 订阅包含颜色信息的点云话题
        self.topic_name = '/camera/camera/depth/color/points'
        self.subscription = self.create_subscription(
            PointCloud2,
            self.topic_name,
            self.listener_callback,
            10 # QoS
        )
        
        # 保存相关的控制变量
        self.save_interval = 0.5  # 0.5秒保存一次
        self.last_save_time = time.time()
        self.save_dir = "saved_pcds_color"
        self.save_count = 0
        
        # 创建保存目录
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            self.get_logger().info(f"Created directory: {self.save_dir}")

    def listener_callback(self, msg):
        current_time = time.time()
        
        # 性能优化：如果时间不到 0.5 秒，直接跳过
        if (current_time - self.last_save_time) < self.save_interval:
            return

        self.last_save_time = current_time

        try:
            # 提取 x, y, z 和 rgb 字段
            gen = pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)
            points_data = np.array(list(gen))
        except Exception as e:
            self.get_logger().error(f"解析点云失败: {e}。请确认相机发布的点云包含 'rgb' 字段。")
            return
        
        if points_data.shape[0] == 0:
            return

        # ================= 核心修复区 =================
        # 兼容处理：检查它是 1D 结构化数组 还是 2D 普通数组
        if points_data.ndim == 1 and points_data.dtype.names is not None:
            # ROS 2 Humble 等版本：生成的是带有字段名的 1D 结构化数组
            x = points_data['x']
            y = points_data['y']
            z = points_data['z']
            xyz = np.column_stack((x, y, z))  # 组合成 (N, 3) 的二维矩阵
            rgb_float = points_data['rgb']
        else:
            # 其他较老版本：生成的是标准的 2D 数组 (N, 4)
            xyz = points_data[:, :3]
            rgb_float = points_data[:, 3]
        # ==============================================

        # 2. 提取并解包 RGB 颜色
        # ROS 默认将 RGB 压缩在一个 float32 中，我们需要以 uint32 的视角读取其二进制位
        rgb_uint32 = np.asarray(rgb_float, dtype=np.float32).view(np.uint32)
        
        # 通过位移运算 (Bitwise shift) 分离 R, G, B 通道 (0-255)
        r = np.bitwise_and(np.right_shift(rgb_uint32, 16), 255)
        g = np.bitwise_and(np.right_shift(rgb_uint32, 8), 255)
        b = np.bitwise_and(rgb_uint32, 255)
        
        # Open3D 要求的颜色格式是范围在 [0.0, 1.0] 之间的浮点数
        colors = np.vstack((r, g, b)).T / 255.0

        # 3. 构建 Open3D 格式并保存
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        pcd.colors = o3d.utility.Vector3dVector(colors) # 赋给点云颜色属性
        
        filename = os.path.join(self.save_dir, f"color_frame_{self.save_count:05d}.pcd")
        o3d.io.write_point_cloud(filename, pcd)
        self.get_logger().info(f"Saved: {filename} (包含 {len(xyz)} 个点)")
        self.save_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudColorSaver()
    
    print("后台彩色点云保存节点已启动。按 Ctrl+C 停止。")
    print("正在等待点云数据...")

    try:
        # 纯后台模式，无需多线程，直接阻塞运行
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboard Interrupt, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
