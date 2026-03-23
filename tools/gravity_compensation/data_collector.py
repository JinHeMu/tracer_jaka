#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from tf2_ros import Buffer, TransformListener
import numpy as np
import csv

class PayloadDataCollector(Node):
    def __init__(self):
        super().__init__('payload_data_collector')

        self.base_frame  = 'base_link'
        self.sensor_frame = 'jk_se_vi_200_link'
        self.csv_filename = 'payload_data.csv'

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            WrenchStamped,
            '/jaka_fts_broadcaster/wrench',
            self.wrench_callback,
            10)

        self.current_wrench = None
        self.last_quat      = None
        self.record_count   = 0

        self.csv_file   = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['qx', 'qy', 'qz', 'qw', 'fx', 'fy', 'fz', 'tx', 'ty', 'tz'])

        # 采集频率提高至 50 Hz（原为 10 Hz）
        self.timer = self.create_timer(0.02, self.record_data)

        self.get_logger().info(f"Data Collector Started. Saving to {self.csv_filename}")
        self.get_logger().info("Jog the robot around. Data will be recorded automatically when pose changes.")
        self.get_logger().info("Press Ctrl+C to stop recording.")

    def wrench_callback(self, msg):
        self.current_wrench = msg.wrench

    def record_data(self):
        if self.current_wrench is None:
            return

        try:
            trans = self.tf_buffer.lookup_transform(
                self.sensor_frame,
                self.base_frame,
                rclpy.time.Time())

            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w
            current_quat = np.array([qx, qy, qz, qw])

            # 姿态阈值放宽：0.9995（约 ±1.6°，原为 0.999 约 ±2.5°）
            # 点乘越接近 1 表示姿态越相似；阈值越大，更小的角度变化就能触发记录
            if self.last_quat is not None:
                dot_product = np.abs(np.dot(current_quat, self.last_quat))
                if dot_product > 0.99995:
                    return

            self.last_quat = current_quat

            fx = self.current_wrench.force.x
            fy = self.current_wrench.force.y
            fz = self.current_wrench.force.z
            tx = self.current_wrench.torque.x
            ty = self.current_wrench.torque.y
            tz = self.current_wrench.torque.z

            self.csv_writer.writerow([qx, qy, qz, qw, fx, fy, fz, tx, ty, tz])
            self.csv_file.flush()

            self.record_count += 1
            self.get_logger().info(f"Recorded point {self.record_count} (Fz: {fz:.2f} N)")

        except Exception:
            pass

    def destroy_node(self):
        self.csv_file.close()
        self.get_logger().info(f"File closed. Total points recorded: {self.record_count}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PayloadDataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
