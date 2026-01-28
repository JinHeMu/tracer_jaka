#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from tf2_ros import Buffer, TransformListener
import numpy as np
from scipy.spatial.transform import Rotation as R

# ==========================================
# 1. 请在此处填入计算出的参数
# ==========================================
MASS = 1.31566       # 替换为你的计算结果 (kg)
COM  = [-0.0032298, -0.0049969, 0.0639627] # 替换为你的计算结果 (m) [x, y, z]
F_BIAS = [-9.3077, -8.9436, 0.8071]     # 替换为你的计算结果 [Fx, Fy, Fz]
T_BIAS = [0.5867, -0.5734, -0.0368]      # 替换为你的计算结果 [Tx, Ty, Tz]
# ==========================================

class GravityCompensatorNode(Node):
    def __init__(self):
        super().__init__('gravity_verification_node')
        
        self.g = 9.81
        
        # TF 监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 订阅原始力数据 (来自你的 C++ 驱动)
        # 假设通过 force_torque_sensor_broadcaster 发布的话题是 /force_torque_sensor_broadcaster/wrench
        # 或者你上面脚本里用的 /jaka_tfs_broadcast/wrench
        self.sub = self.create_subscription(
            WrenchStamped,
            'jaka_fts_broadcaster/wrench', 
            self.callback,
            10
        )
        
        # 发布补偿后的力 (用于绘图验证)
        self.pub_comp = self.create_publisher(WrenchStamped, '/compensated_wrench', 10)
        
        print("验证节点已启动。请在 Rviz 或 RQT Plot 中观察 /compensated_wrench")

    def callback(self, msg):
        try:
            # 1. 获取当前姿态 (Base -> Sensor)
            # 注意：使用了 msg.header.stamp 以保证时间和数据同步
            # 如果报错 extrapolation，可以改用 rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'base_link', 
                'jk_se_vi_200_link', 
                rclpy.time.Time() # 获取最新变换
            )
            
            # 将四元数转为旋转矩阵
            q = trans.transform.rotation
            r = R.from_quat([q.x, q.y, q.z, q.w])
            R_mat = r.as_matrix() # Base 到 Sensor 的旋转矩阵
            
            # 2. 获取原始数据
            F_raw = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
            T_raw = np.array([msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])
            
            # 3. 计算重力补偿
            # 理论重力向量在传感器系下的投影: g_sensor = R^T * [0, 0, -1]
            g_vec_sensor = R_mat.T @ np.array([0.0, 0.0, -1.0])
            F_gravity = (MASS * self.g) * g_vec_sensor
            
            # 纯接触力 = 原始读数 - 重力分量 - 零点偏置
            F_contact = F_raw - F_gravity - np.array(F_BIAS)
            
            # 力矩补偿 (简单模型)
            # T_gravity = CoM x F_gravity
            T_gravity = np.cross(np.array(COM), F_gravity)
            T_contact = T_raw - T_gravity - np.array(T_BIAS)
            
            # 4. 发布结果
            out_msg = WrenchStamped()
            out_msg.header = msg.header
            out_msg.header.frame_id = "jk_se_vi_200_link"
            
            out_msg.wrench.force.x = F_contact[0]
            out_msg.wrench.force.y = F_contact[1]
            out_msg.wrench.force.z = F_contact[2]
            out_msg.wrench.torque.x = T_contact[0]
            out_msg.wrench.torque.y = T_contact[1]
            out_msg.wrench.torque.z = T_contact[2]
            
            self.pub_comp.publish(out_msg)
            
            #终端打印 (可选)
            print(f"Compensated Force: {F_contact}", end='\r')
            
        except Exception as e:
            # print(f"TF Error: {e}", end='\r')
            pass

def main(args=None):
    rclpy.init(args=args)
    node = GravityCompensatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
