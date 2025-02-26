#!/usr/bin/env python3

'''
File   : camera_vio_laser_dist.py
author : FantasyWilly
email  : bc697522h04@gmail.com

相機型號 : KTG-TT30
檔案大綱 :
    1. 接收 - 相機＆雲台回傳資料
    2. 接收 - [vio] 相機定位 (x,y,z)
    2. 計算 - 目標物經緯度
    3. 發布 - 目標物經緯度資訊至ROS2
'''

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from camera_msg_pkg.msg import Camera
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped

# ---------- 基本參數(全域參數) ----------

Earth_radius = 6371000.0    # 定義地球半徑（單位：公尺）

# ---------- (gps_callback) 訂閱ROS2 GPS的相關訊息 ----------
def destination_point(lat, lon, bearing, distance):
    
    # 地球半徑
    R = Earth_radius

    # 將起始緯度、經度與方位角轉換成弧度
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    bearing_rad = math.radians(bearing)

    # 使用球面三角學公式
    target_lat = math.asin(math.sin(lat_rad) * math.cos(distance / R) +
                           math.cos(lat_rad) * math.sin(distance / R) * math.cos(bearing_rad))
    target_lon = lon_rad + math.atan2(math.sin(bearing_rad) * math.sin(distance / R) * math.cos(lat_rad),
                                      math.cos(distance / R) - math.sin(lat_rad) * math.sin(target_lat))

    return math.degrees(target_lat), math.degrees(target_lon)

class TargetPositionNode(Node):
    def __init__(self):
        super().__init__('vio_target_position_node')

        # 宣告初始全域座標與初始航向，可從參數設定或直接指定
        self.declare_parameter('initial_lat', 23.4507301)                                           # 初始緯度
        self.declare_parameter('initial_lon', 120.2861433)                                          # 初始經度
        self.declare_parameter('initial_heading', 0)                                                # 初始航向 (度)
        self.declare_parameter('vio_pose_topic', '/mavros/vision_pose/pose')                        # vio_pose_topic
         
        self.initial_lat = self.get_parameter('initial_lat').get_parameter_value().double_value
        self.initial_lon = self.get_parameter('initial_lon').get_parameter_value().double_value
        self.initial_heading = self.get_parameter('initial_heading').get_parameter_value().integer_value
        self.vio_pose_topic = self.get_parameter('vio_pose_topic').get_parameter_value().string_value

        # QoS 設定
        qos = QoSProfile(
            reliability=ReliabilityPolicy(0),  # BEST EFFORT 模式
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_ALL,
        )

        # 訂閱 UAV - Camera   [接收] - 相機數據（包含雷射測距、雲台角度等）
        self.create_subscription(
            Camera,
            '/camera_data_pub',
            self.camera_callback,
            qos_profile=qos)

        # 訂閱 UAV - Position [接收] - UAV 局部座標 (x, y, z)
        self.create_subscription(
            PoseStamped,
            self.vio_pose_topic,
            self.pose_callback,
            qos_profile=qos)

        # UAV 的全域經緯度 - 定義初始值
        self.uav_lat = self.initial_lat
        self.uav_lon = self.initial_lon
        self.uav_heading = self.initial_heading

        # 用來儲存從 /mavros/local_position/pose 得到的局部位置（單位：公尺）
        self.local_x = None
        self.local_y = None
        self.local_z = None

        # Publisher 用來發布計算後的目標物經緯度 (NavSatFix)
        self.target_pub = self.create_publisher(
            NavSatFix,
            '/target_position',
            qos_profile=qos)

    # pose
    def pose_callback(self, msg: PoseStamped):
        self.local_x = msg.pose.position.x  # 局部 x 值 (代表東向)
        self.local_y = msg.pose.position.y  # 局部 y 值 (代表北向)
        self.local_z = msg.pose.position.z  # 局部 z 值 (代表向上)
        
        # 計算 移動多少 經緯度
        delta_lat = (self.local_y / Earth_radius) * (180 / math.pi)
        delta_lon = (self.local_x / (Earth_radius * math.cos(math.radians(self.initial_lat)))) * (180 / math.pi)

        # 計算 當前經緯度
        self.uav_lat = self.initial_lat + delta_lat
        self.uav_lon = self.initial_lon + delta_lon
        
        self.get_logger().info(f'[Pose] 局部 x: {self.local_x:.2f}, y: {self.local_y:.2f}, z: {self.local_z:.2f}')
        self.get_logger().info(f'[GPS計算] 當前緯度: {self.uav_lat:.7f}, 當前經度: {self.uav_lon:.7f}')

    # ------------------------ 計算目標物 經緯度函數 ---------------------------------
    def camera_callback(self, msg: Camera):
        if not msg.data:
            self.get_logger().warning('沒有接收到雲台數據！')
            return
        
        # 取出最新一筆雲台數據
        camera_data = msg.data[0]
        target_distance = camera_data.targetdist      # 雷射測距距離（公尺）
        pitch_angle = camera_data.pitchangle          # 雲台的 pitch 角度（度）
        gimbal_yaw = camera_data.yawangle             # 雲台的 yaw 角度（度）
        
        # 檢查是否已接收到 UAV - VIO系統
        if self.local_x is None or self.local_y is None or self.local_z is None:
            self.get_logger().warning('等待 VIO 傳回資料[x,y,z]...')
            return
        
        # 計算 目標物 - 水平分量
        pitch_rad = math.radians(pitch_angle)
        horizontal_distance = target_distance * math.cos(pitch_rad)
        
        # 計算 目標物 - 絕對方位角
        gimbal_yaw_adjusted = (gimbal_yaw + 360) % 360
        total_bearing = (self.uav_heading + gimbal_yaw_adjusted) % 360
        
        # self.get_logger().info(f'[計算] 水平距離: {horizontal_distance:.2f} 公尺, 絕對方位角: {total_bearing:.2f} 度')
        
        # 利用起始計算得到的 UAV 當前全域經緯度與目標物的水平距離、方位角，計算目標物經緯度
        target_lat, target_lon = destination_point(self.uav_lat, self.uav_lon, total_bearing, horizontal_distance)
        self.get_logger().info(f'[結果] 目標緯度: {target_lat:.7f}, 目標經度: {target_lon:.7f}')
        
        # 建立 NavSatFix 訊息並發布目標物經緯度
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
