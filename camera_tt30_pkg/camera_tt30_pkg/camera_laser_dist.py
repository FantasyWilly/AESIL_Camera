#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
File   : camera_laser_dist.py
author : FantasyWilly
email  : bc697522h04@gmail.com

相機型號 : KTG-TT30
檔案大綱 :
    A. 接收 - 相機 (roll, yaw, pitch) 與雲台數據
    B. 接收 - [GPS] 定位(lat, lon)
    C. 接收 - [Laser] 雷射測距距離 (target distance)
    D. 計算 - 目標物經緯度
    E. 發布 - 目標物經緯度資訊至ROS2
'''

# Python
import math

# ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# ROS2 自定義消息包
from camera_msg_pkg.msg import Camera
from camera_msg_pkg.msg import Laser

# ---------- 基本參數 (全域) ----------
Earth_radius = 6371000.0    # 定義地球半徑（單位：公尺）

# ---------------------- 目標物計算公式 ----------------------
def destination_point(lat, lon, bearing, distance):
    # 地球半徑
    R = Earth_radius

    # 角度轉弧度
    lat_rad     = math.radians(lat)      # 緯度轉換成弧度
    lon_rad     = math.radians(lon)      # 經度轉換成弧度
    bearing_rad = math.radians(bearing)  # 方位角轉換成弧度

    # 使用球面三角學公式計算目標位置
    target_lat = math.asin(math.sin(lat_rad) * math.cos(distance / R) +
                           math.cos(lat_rad) * math.sin(distance / R) * math.cos(bearing_rad))
    target_lon = lon_rad + math.atan2(math.sin(bearing_rad) * math.sin(distance / R) * math.cos(lat_rad),
                                      math.cos(distance / R) - math.sin(lat_rad) * math.sin(target_lat))
    
    return math.degrees(target_lat), math.degrees(target_lon)  # 將結果轉換成角度返回

# ----------------------- ROS2 節點設定 -----------------------
class TargetPositionNode(Node):
    """
    定義 ROS2 Node 節點：
        1. 接收 - 相機 (roll, yaw, pitch) 與雷射測距資訊
        2. 接收 - GPS 定位 (lat, lon)
        3. 接收 - 飛機航向 (heading)
        4. 計算 - 目標物經緯度
        5. 發布 - 目標物資訊至 ROS2
    """
    def __init__(self):
        super().__init__('target_position_node')

        # QoS 定義
        qos = QoSProfile(
            reliability=ReliabilityPolicy(0),                # 使用 BEST EFFORT 模式
            durability=DurabilityPolicy.VOLATILE,              # 設定耐久性為 VOLATILE
            history=HistoryPolicy.KEEP_ALL,                    # 保留所有訊息
        )
        
        # 訂閱相機數據（接收 roll, yaw, pitch）
        self.create_subscription(
            Camera,
            '/camera_data_pub',
            self.camera_callback,
            qos_profile=qos)
        
        # 訂閱雷射數據（接收雷射測距距離）
        self.create_subscription(
            Laser,
            '/laser_data_pub',
            self.laser_callback,
            qos_profile=qos)
        
        # 訂閱 GPS 定位數據
        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/raw/fix',
            self.gps_callback,
            qos_profile=qos)
        
        # 訂閱飛機航向數據
        self.create_subscription(
            Float64,
            '/mavros/global_position/compass_hdg',
            self.heading_callback,
            qos_profile=qos)
        
        # 初始化變數
        self.uav_lat = None         # UAV 的緯度
        self.uav_lon = None         # UAV 的經度
        self.uav_heading = None     # UAV 的方位角（度）
        
        self.laser_distance = None  # 雷射測距距離（公尺）
        self.camera_roll = None     # 相機 roll 值
        self.camera_yaw = None      # 相機 yaw 值
        self.camera_pitch = None    # 相機 pitch 值
        
        # 建立發布目標位置的 Publisher
        self.target_pub = self.create_publisher(
            NavSatFix,
            '/target_position',
            qos_profile=qos)

    # GPS 訂閱回調函數
    def gps_callback(self, msg: NavSatFix):
        self.uav_lat = msg.latitude   # 更新 UAV 緯度
        self.uav_lon = msg.longitude   # 更新 UAV 經度
        # self.get_logger().info(f'[GPS] lat: {self.uav_lat}, lon: {self.uav_lon}')

    # Heading 訂閱回調函數
    def heading_callback(self, msg: Float64):
        self.uav_heading = msg.data   # 更新 UAV 航向
        # self.get_logger().info(f'[Heading] {self.uav_heading} 度')

    # 相機數據訂閱回調函數，只接收 roll, yaw, pitch
    def camera_callback(self, msg: Camera):
        if not msg.data:
            self.get_logger().warning('沒有接收到相機數據！')
            return
        
        # 取出最新一筆相機數據
        camera_data = msg.data[0]
        self.camera_roll = camera_data.rollangle      # 更新相機的 roll 值
        self.camera_yaw = camera_data.yawangle        # 更新相機的 yaw 值
        self.camera_pitch = camera_data.pitchangle    # 更新相機的 pitch 值
        
        # 更新目標位置計算
        self.update_target_position()

    # 雷射數據訂閱回調函數，接收雷射測距距離
    def laser_callback(self, msg: Laser):
        if not msg.data:
            self.get_logger().warning('沒有接收到雷射數據！')
            return
        
        # 取出最新一筆雷射數據
        laser_data = msg.data[0]
        self.laser_distance = laser_data.targetdist  # 更新雷射測距距離
        
        # 更新目標位置計算
        self.update_target_position()

    # 計算並發布目標物經緯度的函數
    def update_target_position(self):
        
        # 檢查是否已收到 GPS 定位資料
        if self.uav_lat is None or self.uav_lon is None or self.uav_heading is None:
            self.get_logger().warning('等待 UAV 傳回資料 - GPS...')
            return
        
        # 檢查是否已收到雷射數據
        if self.laser_distance is None:
            self.get_logger().warning('等待雷射數據...')
            return
        
        # 檢查是否已收到雲台數據
        if self.camera_pitch is None or self.camera_yaw is None:
            self.get_logger().warning('等待雲台數據...')
            return
        
        # 計算目標物的水平距離（根據雷射距離與相機 pitch 角）
        pitch_rad = math.radians(self.camera_pitch)  # pitch 角度轉換成弧度
        horizontal_distance = self.laser_distance * math.cos(pitch_rad)  # 計算水平距離
        
        # 計算目標物的絕對方位角（結合 UAV 航向與相機 yaw）
        camera_yaw_adjusted = (self.camera_yaw + 360) % 360  # 調整相機 yaw 值確保在 0~359 度
        total_bearing = (self.uav_heading + camera_yaw_adjusted) % 360  # 計算總方位角
        
        # 計算目標物的經緯度
        target_lat, target_lon = destination_point(self.uav_lat, self.uav_lon, total_bearing, horizontal_distance)
        self.get_logger().info(f'[結果] 目標緯度: {target_lat:.7f}, 目標經度: {target_lon:.7f}')
        
        # 建立並填充 NavSatFix 訊息，發布目標位置
        target_msg = NavSatFix()
        target_msg.latitude = round(target_lat, 7)   # 設定目標緯度
        target_msg.longitude = round(target_lon, 7)  # 設定目標經度
        self.target_pub.publish(target_msg)          # 發布目標位置訊息

def main(args=None):
    rclpy.init(args=args)
    node = TargetPositionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
