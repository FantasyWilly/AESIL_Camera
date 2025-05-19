#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
File   : vio_laser_target_gps
author : FantasyWilly
email  : FantasyWilly@gmail.com

檔案大綱 :
    A. 接收 - 相機＆雲台回傳資料
    B. 接收 - [vio] 相機定位 (x,y,z) & 計算 [vio] 航向
    C. 計算 - 目標物經緯度
    D. 發布 - 目標物經緯度資訊至ROS2
'''

# Python
import math

# ROS2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# ROS2 自訂義消息包
from camera_msg_pkg.msg import Camera
from camera_msg_pkg.msg import Laser

# 全域參數：定義地球半徑 (單位：公尺)
Earth_radius = 6371000.0

# 定義根據起始經緯度、方位角與距離計算目標經緯度的函數
def destination_point(lat, lon, bearing, distance):
    # R 為地球半徑
    R = Earth_radius

    # 將起始緯度、經度與方位角轉換成弧度
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    bearing_rad = math.radians(bearing)

    # 利用球面三角學公式計算目標位置的緯度與經度（以弧度表示）
    target_lat = math.asin(math.sin(lat_rad) * math.cos(distance / R) +
                           math.cos(lat_rad) * math.sin(distance / R) * math.cos(bearing_rad))
    target_lon = lon_rad + math.atan2(math.sin(bearing_rad) * math.sin(distance / R) * math.cos(lat_rad),
                                      math.cos(distance / R) - math.sin(lat_rad) * math.sin(target_lat))

    # 將計算結果轉換成角度返回
    return math.degrees(target_lat), math.degrees(target_lon)

# 定義 ROS2 節點
class TargetPositionNode(Node):
    def __init__(self):
        super().__init__('vio_laser_target_gps_node')

        # 宣告並讀取初始全域座標與初始航向參數
        self.declare_parameter('initial_lat', 23.4515234)                     # 初始緯度
        self.declare_parameter('initial_lon', 120.2862747)                    # 初始經度
        self.declare_parameter('vio_pose_topic', '/mavros/vision_pose/pose')  # VIO 定位話題名稱
         
        self.initial_lat = self.get_parameter('initial_lat').get_parameter_value().double_value
        self.initial_lon = self.get_parameter('initial_lon').get_parameter_value().double_value
        self.vio_pose_topic = self.get_parameter('vio_pose_topic').get_parameter_value().string_value

        # 設定 QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy(0),  # BEST EFFORT 模式
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_ALL,
        )

        # 訂閱相機數據（接收雲台的 roll、yaw、pitch 資料）
        self.create_subscription(
            Camera,
            '/camera_data_pub',
            self.camera_callback,
            qos_profile=qos)
        
        # 訂閱雷射數據（接收雷射測距距離，將不再從相機消息中取得）
        self.create_subscription(
            Laser,
            '/laser_data_pub',
            self.laser_callback,
            qos_profile=qos)

        # 訂閱 VIO 系統傳回的 UAV 局部座標 (x, y, z)
        self.create_subscription(
            PoseStamped,
            self.vio_pose_topic,
            self.pose_callback,
            qos_profile=qos)

        # 初始全域經緯度與航向設定 (若無 VIO 資料則使用初始參數)
        self.uav_lat = self.initial_lat
        self.uav_lon = self.initial_lon
        self.uav_heading = 0.0
        self.relative_yaw = 0.0
        
        # 儲存從 VIO 得到的局部座標 (單位：公尺)
        self.local_x = None
        self.local_y = None
        self.local_z = None

        # 建立 Publisher 以發布目標物經緯度 (NavSatFix)
        self.target_pub = self.create_publisher(
            NavSatFix,
            '/target_position',
            qos_profile=qos)
        
        # 儲存最新的雷射測距距離
        self.laser_distance = None

    # 接收 VIO 定位資料的回調函數
    def pose_callback(self, msg: PoseStamped):
        # 從訊息中取得局部座標
        self.local_x = msg.pose.position.x  # 局部 x
        self.local_y = msg.pose.position.y  # 局部 y
        self.local_z = msg.pose.position.z  # 局部 z

        # 從四元數轉換出 yaw，更新 UAV 當前的航向
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        yaw_degrees = math.degrees(yaw)

        # VIO  YAW 值轉換
        self.uav_heading = 90 - yaw_degrees
        self.get_logger().info(f'[VIO航向角] 當前航向 (yaw): {self.uav_heading:.2f}°')

        # 利用全域位移計算緯度與經度的變化量
        delta_lat = (self.local_y / Earth_radius) * (180 / math.pi)
        delta_lon = (self.local_x / (Earth_radius * math.cos(math.radians(self.initial_lat)))) * (180 / math.pi)

        # 更新 UAV 當前全域經緯度
        self.uav_lat = self.initial_lat + delta_lat
        self.uav_lon = self.initial_lon + delta_lon

        self.get_logger().info(f'[VIO定位] 局部 x: {self.local_x:.2f}, y: {self.local_y:.2f}, z: {self.local_z:.2f}')
        self.get_logger().info(f'[GPS計算] 當前緯度: {self.uav_lat:.7f}, 當前經度: {self.uav_lon:.7f}')


    # 接收相機（雲台）數據的回調函數，僅接收 roll、yaw、pitch
    def camera_callback(self, msg: Camera):
        if not msg.data:
            self.get_logger().warning('沒有接收到雲台數據！')
            return
        
        # 取出最新一筆雲台數據，並儲存供後續計算使用
        camera_data = msg.data[0]
        self.camera_roll = camera_data.rollangle      # 更新相機的 roll 值
        self.camera_yaw = camera_data.yawangle        # 更新相機的 yaw 值
        self.camera_pitch = camera_data.pitchangle    # 更新相機的 pitch 值
        
        # 如果雷射數據已經存在，則利用最新的雲台與雷射數據進行目標位置計算
        if self.laser_distance is not None:
            self.update_target_position_with_laser()

    # 接收雷射測距數據的回調函數
    def laser_callback(self, msg: Laser):
        if not msg.data:
            self.get_logger().warning('沒有接收到雷射數據！')
            return
        
        # 取出最新一筆雷射數據，更新雷射測距距離
        laser_data = msg.data[0]
        self.laser_distance = laser_data.targetdist
        
        # 利用最新的雷射數據進行目標位置計算
        self.update_target_position_with_laser()

    # 利用最新的 VIO、雲台與雷射數據計算目標物經緯度
    def update_target_position_with_laser(self):
        # 檢查是否已收到 VIO 定位資料
        if self.local_x is None or self.local_y is None or self.local_z is None:
            self.get_logger().warning('等待 VIO 傳回資料[x,y,z]...')
            return
        
        # 檢查是否已收到雷射數據
        if self.laser_distance is None:
            self.get_logger().warning('等待雷射數據...')
            return
        
        # 檢查是否已收到雲台數據
        if self.camera_pitch is None or self.camera_yaw is None:
            self.get_logger().warning('等待雲台數據...')
            return
        
        # 計算水平距離：利用雷射測距距離與雲台 pitch 角 (以弧度計算)
        pitch_rad = math.radians(self.camera_pitch)
        horizontal_distance = self.laser_distance * math.cos(pitch_rad)
        
        # 計算絕對方位角：結合 UAV 航向與調整後的雲台 yaw
        gimbal_yaw_adjusted = (self.camera_yaw + 360) % 360
        total_bearing = (self.uav_heading + gimbal_yaw_adjusted) % 360
        
        # 利用當前 UAV 全域經緯度、總方位角與水平距離計算目標物經緯度
        target_lat, target_lon = destination_point(self.uav_lat, self.uav_lon, total_bearing, horizontal_distance)
        self.get_logger().info(f'[雷射計算] 目標緯度: {target_lat:.7f}, 目標經度: {target_lon:.7f}')
        
        # 建立 NavSatFix 訊息並發布目標物經緯度資訊
        target_msg = NavSatFix()
        target_msg.latitude = round(target_lat, 7)
        target_msg.longitude = round(target_lon, 7)
        self.target_pub.publish(target_msg)

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
