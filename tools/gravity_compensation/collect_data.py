#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from tf2_ros import Buffer, TransformListener
import numpy as np
from scipy.spatial.transform import Rotation as R
import os

class AutoDataCollector(Node):
    def __init__(self):
        super().__init__('auto_data_collector')
        
        # 参数配置
        self.declare_parameter('sensor_frame', 'jk_se_vi_200_link')
        self.declare_parameter('gravity_frame', 'base_link') 
        self.sensor_frame = self.get_parameter('sensor_frame').value
        self.gravity_frame = self.get_parameter('gravity_frame').value
        
        # TF 监听
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 数据存储
        self.rot_data = []
        self.f_data = []
        self.t_data = []
        
        # 订阅力传感器
        self.sub = self.create_subscription(
            WrenchStamped,
            'jaka_fts_broadcaster/wrench', 
            self.wrench_callback,
            10
        )
        
        self.get_logger().info("--- 自动采集模式已开启 ---")
        self.get_logger().info(f"正在从 {self.gravity_frame} 采集到 {self.sensor_frame} 的变换...")
        self.get_logger().info("请控制机械臂缓慢变换姿态。完成后请按 [Ctrl+C] 停止并保存。")

    def wrench_callback(self, msg):
        try:
            # 获取旋转矩阵 (已包含 URDF 中的 0.02rad 倾斜)
            trans = self.tf_buffer.lookup_transform(
                self.gravity_frame, 
                self.sensor_frame, 
                rclpy.time.Time()
            )
            
            q = trans.transform.rotation
            rot = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            
            f_meas = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
            t_meas = np.array([msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])
            
            # 简单去重：如果旋转矩阵变化很小，则不保存（防止原地静止产生过多冗余数据）
            if len(self.rot_data) > 0:
                dot_prod = np.sum(self.rot_data[-1] * rot) # 矩阵点积近似判断变化
                if dot_prod > 2.9999: # 姿态几乎没变
                    return

            self.rot_data.append(rot)
            self.f_data.append(f_meas)
            self.t_data.append(t_meas)
            
            # 实时打印样本数
            print(f"\r[采集进度] 已获取样本数: {len(self.f_data)}", end='')
                
        except Exception:
            # 忽略 TF 尚未就绪时的错误
            pass

    def save_and_exit(self):
        if len(self.f_data) < 10:
            self.get_logger().warn("\n数据量太少，未保存文件。")
            return
            
        filename = "calibration_data.npz"
        np.savez(filename, 
                 rot=np.array(self.rot_data), 
                 force=np.array(self.f_data), 
                 torque=np.array(self.t_data))
        print(f"\n\n[保存成功] 数据已存至: {os.path.abspath(filename)}")
        print(f"共计 {len(self.f_data)} 组同步数据。")

def main():
    rclpy.init()
    node = AutoDataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 捕捉 Ctrl+C 信号并执行保存
        node.save_and_exit()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
