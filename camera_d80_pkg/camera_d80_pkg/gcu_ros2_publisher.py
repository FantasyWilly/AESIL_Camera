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
from geometry_msgs.msg import Vector3

# ---------- [CUPublisher] 初始化[Node], 宣告參數, 發布相機回傳資訊 ----------
class GCUPublisher(Node):
    def __init__(self):
        super().__init__('gcu_publisher_node')
        self.publisher_ = self.create_publisher(Vector3, 'gcu_response', 10)

    # ----------------------- (publish_data) 接收資料 & 發布至ROS2 -----------------------
    def publish_data(self, roll: float, pitch: float, yaw: float):
        msg = Vector3()
        msg.x = roll
        msg.y = pitch
        msg.z = yaw

        self.publisher_.publish(msg)
        # self.get_logger().info(f"已發布資料: roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}")

# ---------- [main] 主要執行序 ----------
def main(args=None):
    rclpy.init(args=args)
    node = GCUPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
