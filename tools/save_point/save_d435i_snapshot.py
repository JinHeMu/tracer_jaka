#!/usr/bin/env python3
"""
d435i_saver.py
保存 RealSense D435i 的 RGB 图像 + 同步彩色点云。
用法：
    python3 save_d435i_snapshot.py                 # 默认 RELIABLE，适合 D435i ROS2 节点
    python3 save_d435i_snapshot.py --qos best_effort
    python3 save_d435i_snapshot.py --save-dir ./out
交互：按 Enter 保存一帧，输入 q + Enter 退出。
"""
import argparse
import os
import sys
import threading

import cv2
import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import message_filters
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge


class D435iSaver(Node):
    def __init__(self, save_dir='./d435i_data', reliability='reliable'):
        super().__init__('d435i_saver')

        self.rgb_dir = os.path.join(save_dir, 'rgb')
        self.pcd_dir = os.path.join(save_dir, 'pointcloud')
        os.makedirs(self.rgb_dir, exist_ok=True)
        os.makedirs(self.pcd_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.capture_flag = False
        self.frame_count = 0

        self._rgb_cnt = 0
        self._pc_cnt = 0
        self._sync_cnt = 0

        rel = (QoSReliabilityPolicy.RELIABLE
               if reliability == 'reliable'
               else QoSReliabilityPolicy.BEST_EFFORT)

        qos_rgb = QoSProfile(
            reliability=rel,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        qos_pc = QoSProfile(
            reliability=rel,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.get_logger().info(f'QoS reliability = {reliability.upper()}')

        rgb_topic = '/camera/camera/color/image_raw'
        pc_topic = '/camera/camera/depth/color/points'

        rgb_sub = message_filters.Subscriber(
            self, Image, rgb_topic, qos_profile=qos_rgb
        )
        pc_sub = message_filters.Subscriber(
            self, PointCloud2, pc_topic, qos_profile=qos_pc
        )

        rgb_sub.registerCallback(lambda msg: self._bump('rgb'))
        pc_sub.registerCallback(lambda msg: self._bump('pc'))

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, pc_sub], queue_size=30, slop=0.3
        )
        self.ts.registerCallback(self.sync_callback)

        self._stats_timer = self.create_timer(2.0, self._print_stats)

        self.get_logger().info(f'保存目录：{os.path.abspath(save_dir)}')
        self.get_logger().info(f'订阅话题：\n  {rgb_topic}\n  {pc_topic}')
        self.get_logger().info('按 Enter 保存一帧，输入 q + Enter 退出。')

        self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kb_thread.start()

    def _keyboard_loop(self):
        while rclpy.ok():
            try:
                line = sys.stdin.readline()
            except Exception:
                break
            if not line:
                continue
            cmd = line.strip().lower()
            if cmd == 'q':
                self.get_logger().info('收到退出指令。')
                rclpy.shutdown()
                break
            self.capture_flag = True
            self.get_logger().info('>>> capture_flag = True，等待下一帧同步消息…')

    def _bump(self, which: str):
        if which == 'rgb':
            self._rgb_cnt += 1
        else:
            self._pc_cnt += 1

    def _print_stats(self):
        self.get_logger().info(
            f'[stats] rgb={self._rgb_cnt}  pc={self._pc_cnt}  '
            f'synced={self._sync_cnt}  saved={self.frame_count}  '
            f'flag={self.capture_flag}'
        )

    def sync_callback(self, rgb_msg: Image, pc_msg: PointCloud2):
        self._sync_cnt += 1
        if not self.capture_flag:
            return
        self.capture_flag = False

        sec = rgb_msg.header.stamp.sec
        nsec = rgb_msg.header.stamp.nanosec
        stamp = f'{sec}_{nsec:09d}'

        try:
            cv_img = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            rgb_path = os.path.join(self.rgb_dir, f'{stamp}.png')
            cv2.imwrite(rgb_path, cv_img)
            self.get_logger().info(
                f'[RGB] {rgb_path}  ({cv_img.shape[1]}x{cv_img.shape[0]})'
            )
        except Exception as e:
            self.get_logger().error(f'保存 RGB 失败：{e}')
            return

        try:
            pcd_path = os.path.join(self.pcd_dir, f'{stamp}.pcd')
            n = self._save_pointcloud(pc_msg, pcd_path)
            self.get_logger().info(f'[PCD] {pcd_path}  ({n} pts)')
            self.frame_count += 1
            self.get_logger().info(f'总计已保存：{self.frame_count} 帧')
        except Exception as e:
            self.get_logger().error(f'保存点云失败：{e}')

    @staticmethod
    def _save_pointcloud(pc_msg: PointCloud2, path: str) -> int:
        pts = pc2.read_points(
            pc_msg,
            field_names=('x', 'y', 'z', 'rgb'),
            skip_nans=True,
            reshape_organized_cloud=False,
        )
        pts = np.asarray(pts)
        if pts.size == 0:
            raise RuntimeError('点云为空。')

        if pts.dtype.names is not None:
            xyz = np.stack(
                [pts['x'], pts['y'], pts['z']], axis=1
            ).astype(np.float64)
            rgb_field = np.ascontiguousarray(pts['rgb'])
        else:
            xyz = pts[:, :3].astype(np.float64)
            rgb_field = np.ascontiguousarray(pts[:, 3])

        rgb_float = rgb_field.astype(np.float32, copy=False)
        rgb_uint = rgb_float.view(np.uint32)
        r = ((rgb_uint >> 16) & 0xFF).astype(np.float64) / 255.0
        g = ((rgb_uint >> 8) & 0xFF).astype(np.float64) / 255.0
        b = (rgb_uint & 0xFF).astype(np.float64) / 255.0
        colors = np.stack([r, g, b], axis=1)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        o3d.io.write_point_cloud(path, pcd, write_ascii=False)
        return xyz.shape[0]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--save-dir', default='./d435i_data')
    parser.add_argument('--qos', choices=['reliable', 'best_effort'],
                        default='reliable',
                        help='订阅 QoS，需与发布端匹配；D435i 默认发布为 RELIABLE')
    args = parser.parse_args()

    rclpy.init()
    node = D435iSaver(save_dir=args.save_dir, reliability=args.qos)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._stats_timer.cancel()
        except Exception:
            pass
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
