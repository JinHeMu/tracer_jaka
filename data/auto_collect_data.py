#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import WrenchStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import csv
import time
import os
import math

# ================= 配置区域 =================
# 采集间隔 (秒)
COLLECT_INTERVAL = 0.5
# 目标采集数量 (组)
TARGET_SAMPLES = 100
# 保存文件名
OUTPUT_FILE = 'ft_calibration_data.csv'

# TF Frame 定义
# TARGET: 基坐标系 (父)
FRAME_TARGET = 'base_link'
# SOURCE: 传感器/末端坐标系 (子)
FRAME_SOURCE = 'jk_se_vi_200_link'
# 话题名称
TOPIC_WRENCH = 'jaka_fts_broadcaster/wrench'
# ===========================================

class AutoCollector(Node):
    def __init__(self):
        super().__init__('jaka_auto_collector')

        # 1. 初始化 CSV 文件
        self.csv_file = open(OUTPUT_FILE, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        # 写入表头
        self.csv_writer.writerow([
            'timestamp_sec', 'timestamp_nanosec', # 时间戳
            'trans_x', 'trans_y', 'trans_z',      # 平移
            'quat_x', 'quat_y', 'quat_z', 'quat_w', # 旋转(四元数)
            'force_x', 'force_y', 'force_z',      # 力
            'torque_x', 'torque_y', 'torque_z'    # 力矩
        ])
        
        # 2. 初始化 ROS 接口
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.wrench_sub = self.create_subscription(
            WrenchStamped,
            TOPIC_WRENCH,
            self.wrench_callback,
            10
        )

        # 3. 状态变量
        self.latest_wrench_msg = None
        self.last_saved_stamp = None # 用于去重
        self.sample_count = 0

        # 4. 定时器
        self.timer = self.create_timer(COLLECT_INTERVAL, self.collect_loop)

        print(f"=== 开始采集任务 ===")
        print(f"目标: {TARGET_SAMPLES} 组数据")
        print(f"保存至: {os.path.abspath(OUTPUT_FILE)}")
        print(f"请移动机器人到不同姿态，保持每个姿态静止约 1 秒...\n")

    def wrench_callback(self, msg):
        self.latest_wrench_msg = msg

    def collect_loop(self):
        # 检查是否完成
        if self.sample_count >= TARGET_SAMPLES:
            print(f"\n采集完成！已保存 {self.sample_count} 组数据。")
            self.destroy_node()
            rclpy.shutdown()
            return

        # 检查是否有数据
        if self.latest_wrench_msg is None:
            print("等待传感器数据...", end='\r')
            return

        # === 核心：去重检查 ===
        # 比较当前消息的时间戳和上次保存的时间戳
        current_stamp = (self.latest_wrench_msg.header.stamp.sec, self.latest_wrench_msg.header.stamp.nanosec)
        if self.last_saved_stamp == current_stamp:
            # 数据未刷新，跳过
            return
        
        # === 获取 TF ===
        try:
            # 获取最新的变换
            trans = self.tf_buffer.lookup_transform(
                FRAME_TARGET,
                FRAME_SOURCE,
                rclpy.time.Time()
            )

            # === 保存数据 ===
            self.save_data(trans, self.latest_wrench_msg)
            
            # 更新状态
            self.last_saved_stamp = current_stamp
            self.sample_count += 1
            
            # 打印进度
            fx = self.latest_wrench_msg.wrench.force.x
            fz = self.latest_wrench_msg.wrench.force.z
            print(f"[进度 {self.sample_count}/{TARGET_SAMPLES}] "
                  f"Fz: {fz:6.2f} | 姿态已记录")

        except (LookupException, ConnectivityException, ExtrapolationException):
            print("警告: 无法获取 TF 变换，跳过此帧。")

    def save_data(self, tf_msg, wrench_msg):
        # 提取数据
        t = tf_msg.transform.translation
        r = tf_msg.transform.rotation
        f = wrench_msg.wrench.force
        tor = wrench_msg.wrench.torque
        
        row = [
            wrench_msg.header.stamp.sec,
            wrench_msg.header.stamp.nanosec,
            f"{t.x:.5f}", f"{t.y:.5f}", f"{t.z:.5f}",
            f"{r.x:.5f}", f"{r.y:.5f}", f"{r.z:.5f}", f"{r.w:.5f}",
            f"{f.x:.5f}", f"{f.y:.5f}", f"{f.z:.5f}",
            f"{tor.x:.5f}", f"{tor.y:.5f}", f"{tor.z:.5f}"
        ]
        
        self.csv_writer.writerow(row)
        self.csv_file.flush() # 确保立即写入硬盘

def main(args=None):
    rclpy.init(args=args)
    node = AutoCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.csv_file.close()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
