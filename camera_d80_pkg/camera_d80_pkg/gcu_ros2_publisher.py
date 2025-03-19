#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
File   : gcu_ros2_publisher.py
Author : FantasyWilly
Email  : bc697522h04@gmail.com

相機型號 : D-80 Pro
檔案大綱 :
    A. 接收解碼數據 & 發布至ROS2
"""

# ROS2
import rclpy
from rclpy.node import Node

# ROS2 自定義消息包
from camera_msg_pkg.msg import Camera, CameraData

# ---------- [CUPublisher] 初始化[Node], 宣告參數, 發布相機回傳資訊 ----------
class GCUPublisher(Node):
    def __init__(self):
        super().__init__('gcu_ros2_publisher_node')
        self.publisher_camera = self.create_publisher(Camera, '/camera_data_pub', 10)

    # ----------------------- (publish_camera_data) 接收 Camera 資料 & 發布至ROS2 -----------------------
    def publish_camera_data(self, roll: float, pitch: float, yaw: float):
        camera_data = CameraData()
        camera_data.rollangle = roll
        camera_data.yawangle = yaw
        camera_data.pitchangle = pitch

        camera_msg = Camera()
        camera_msg.data = [camera_data]

        self.publisher_camera.publish(camera_msg)
        # self.get_logger().info(f"[發布] Camera 資料: [ROLL]={camera_data.rollangle}, [YAW]={camera_data.yawangle}, [PITCH]={camera_data.pitchangle}")
        
# ---------- [main] 主要執行序 ----------
def main(args=None):
    rclpy.init(args=args)
    node = GCUPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
