#!/usr/bin/env python3

'''
File   : camera_laser_dist.py
author : FantasyWilly
email  : bc697522h04@gmail.com

相機型號 : KTG-TT30
檔案大綱 :
    1. 接收 - 相機＆雲台回傳資料
    2. 接收 - [GPS] 定位(lat, lon)
    2. 計算 - 目標物經緯度
    3. 發布 - 目標物經緯度資訊至ROS2
'''

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from camera_msg_pkg.msg import Camera
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

# ----------------------- 基本參數設定 -----------------------

# 定義地球半徑（單位：公尺）
Earth_radius = 6371000.0

# ---------------------- 目標物計算公式 ----------------------
def destination_point(lat, lon, bearing, distance):

    # 地球半徑
    R = Earth_radius

    # 弧度轉化
    lat_rad     =   math.radians(lat)
    lon_rad     =   math.radians(lon)
    bearing_rad =   math.radians(bearing)

    # 使用球面三角學公式
    target_lat = math.asin(math.sin(lat_rad) * math.cos(distance / R) +
                     math.cos(lat_rad) * math.sin(distance / R) * math.cos(bearing_rad))
    target_lon = lon_rad + math.atan2(math.sin(bearing_rad) * math.sin(distance / R) * math.cos(lat_rad),
                                math.cos(distance / R) - math.sin(lat_rad) * math.sin(target_lat))
    
    return math.degrees(target_lat), math.degrees(target_lon)

# ----------------------- ROS2 節點設定 -----------------------
class TargetPositionNode(Node):
    """
    定義 ROS2 Node節點:
        1. 接收 - Camera, GPS, Heading, Alt 資訊
        2. 計算 - 目標物 經緯度
        3. 發布 - 目標物 資訊至 ROS2
    """
    # ------------------------ 初始定義 ---------------------------------
    def __init__(self):
        super().__init__('target_position_node')

        # Qos 定義
        qos = QoSProfile(
            reliability=ReliabilityPolicy(0),                # 使用 BEST EFFORT 模式
            durability=DurabilityPolicy.VOLATILE,            # 設定耐久性為 VOLATILE
            history=HistoryPolicy.KEEP_ALL,                  # 保留所有訊息
        )
        
        # 訂閱 UAV - Camera(自定義) [接收] - 相機數據（包含雷射測距、雲台角度等)
        self.create_subscription(
            Camera,
            '/camera_data_pub',
            self.camera_callback,
            qos_profile=qos)
        
        # 訂閱 UAV - GPS            [接收] - GPS (經緯度)
        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/raw/fix',
            self.gps_callback,
            qos_profile=qos)
        
        # 訂閱 UAV - Heading        [接收] - 飛機航向
        self.create_subscription(
            Float64,
            '/mavros/global_position/compass_hdg',
            self.heading_callback,
            qos_profile=qos)
        
        # 初始化變數
        self.uav_lat = None         # UAV 的緯度
        self.uav_lon = None         # UAV 的經度
        self.uav_rel_alt = None     # UAV 的相對高度（公尺）
        self.uav_heading = None     # UAV 的方位角（度）
        
        # 發布 target position      [
        self.target_pub = self.create_publisher(
            NavSatFix,
            '/target_position',
            qos_profile=qos)

    # GPS
    def gps_callback(self, msg: NavSatFix):
        self.uav_lat = msg.latitude
        self.uav_lon = msg.longitude
        # self.get_logger().info(f'[GPS] lat: {self.uav_lat}, lon: {self.uav_lon}')

    
    # Heading
    def heading_callback(self, msg: Float64):
        self.uav_heading = msg.data
        # self.get_logger().info(f'[Heading] {self.uav_heading} 度')
    
    # ------------------------ 計算目標物 經緯度函數 ---------------------------------
    def camera_callback(self, msg: Camera):
        if not msg.data:
            self.get_logger().warning('沒有接收到雲台數據！')
            return
        
        # 取出最新一筆雲台數據
        camera_data = msg.data[0]                   
        target_distance = camera_data.targetdist    # 雷射測距距離（公尺）
        pitch_angle = camera_data.pitchangle        # 雲台的 pitch 角度（度）
        gimbal_yaw = camera_data.yawangle           # 雲台的 yaw 角度（度）
        
        # self.get_logger().info(f'[Camera] target_distance: {target_distance}, pitch: {pitch_angle}, yaw: {gimbal_yaw}')
        
        # 檢查是否已接收到 UAV - GPS, rel_alt, Heading
        if self.uav_lat is None or self.uav_lon is None or self.uav_heading is None or self.uav_rel_alt is None:
            self.get_logger().warning('等待 UAV - GPS,Heading,rel_alt 資料...')
            return
        
        # 計算 目標物 - 水平分量
        pitch_rad = math.radians(pitch_angle)
        horizontal_distance = target_distance * math.cos(pitch_rad)
        
        # 計算 目標物 - 絕對方位角
        gimbal_yaw_adjusted = (gimbal_yaw + 360) % 360
        total_bearing = (self.uav_heading + gimbal_yaw_adjusted) % 360  # 結合 UAV 航向與雲台 yaw 後，使用模運算確保結果介於 0~359 度之間

        # self.get_logger().info(f'[計算] 水平距離: {horizontal_distance:.2f} 公尺, 絕對方位角: {total_bearing:.2f} 度')

        # 計算 目標物 - 經緯度
        target_lat, target_lon = destination_point(self.uav_lat, self.uav_lon, total_bearing, horizontal_distance)
        self.get_logger().info(f'[結果] 目標緯度: {target_lat:.7f}, 目標經度: {target_lon:.7f}')
        
        # 建立並填充 NavSatFix 訊息，用於發布目標的經緯度資訊
        target_msg = NavSatFix()
        target_msg.latitude = round(target_lat, 7)     # 設定目標緯度
        target_msg.longitude = round(target_lon, 7)    # 設定目標經度
        
        # 將目標位置訊息發布到 /target_position 話題
        self.target_pub.publish(target_msg)
        # self.get_logger().info(f'[發布] 已發布目標位置至 /target_position')
    
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
