#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Python3
import math
import cv2
import numpy as np

# ROS2 基本庫
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# ROS2 消息包
from std_msgs.msg import Float64 
from sensor_msgs.msg import NavSatFix
from camera_msg_pkg.msg import Camera
from yolo_msg_pkg.msg import CenterBox

# ====================== 1. 靜態配置區 ============================ #
class CameraConfig:
    def __init__(self):
        self.pixel_spacing_um = 1.45       # 像素間距 (µm)
        self.pixel_max_width  = 3840       # 感測器最大解析度（寬）
        self.pixel_max_height = 2160       # 感測器最大解析度（高）
        self.base_f_mm = 4.8               # 基本焦距 (mm)
        self.zoom_s    = 1.0               # 當前變焦倍率（動態更新）

# ====================== 2. 攝影機＋解析度讀取 ==================== #
class CameraReader:
    def __init__(self, source_url: str):
        self.cap = cv2.VideoCapture(source_url)
        if not self.cap.isOpened():
            raise RuntimeError(f"無法打開視訊串流: {source_url}")

    def get_frame_size(self):
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return w, h
    
# ====================== 3. 內參計算 ============================ #
class CameraIntrinsics:
    def __init__(self, cfg: CameraConfig):
        self.cfg = cfg
        self.P_spacing_m = cfg.pixel_spacing_um * 1e-6       # µm → m

    def compute_scaled(self, cur_width: int, cur_height: int):
        """
        根據當前解析度與 cfg.zoom_s 重新計算 fx, fy, u0, v0, 及 K
        公式：
          f_m = base_f_mm * zoom_s * 1e-3 (mm→m)
          fx = (f_m/p_spacing_m) * (cur_width  / pixel_max_width)
          fy = (f_m/p_spacing_m) * (cur_height / pixel_max_height)
          u0 = cur_width  / 2
          v0 = cur_height / 2
        """
        # 總焦距 (m)
        f_m = self.cfg.base_f_mm * self.cfg.zoom_s * 1e-3

        # 基準像素焦距
        fx_base = f_m / self.P_spacing_m
        fy_base = fx_base

        # 按解析度縮放
        scale_x = cur_width  / self.cfg.pixel_max_width
        scale_y = cur_height / self.cfg.pixel_max_height
        self.fx = fx_base * scale_x
        self.fy = fy_base * scale_y

        # 中心點
        self.u0 = cur_width  / 2.0
        self.v0 = cur_height / 2.0

        # 內參矩陣
        self.K = np.array([
            [self.fx, 0.0,    self.u0],
            [0.0,    self.fy,  self.v0],
            [0.0,    0.0,      1.0 ]
        ])
        return self.K

class TargetGPSNode(Node):
    def __init__(self):

        # 初始化節點
        super().__init__('target_gps_node')

        # 初始化 [相機硬體參數]config, [相機當前畫面]reader, [內參矩陣計算]intrinsics
        self.cfg    = CameraConfig()
        self.reader = CameraReader('rtsp://user:user@192.168.168.108:554/cam/realmonitor?channel=1&subtype=0')
        self.intr   = CameraIntrinsics(self.cfg)

        # 只讀一次解析度 內參僅初次或變焦時更新
        self.cur_width, self.cur_height = self.reader.get_frame_size()
        self.intr.compute_scaled(self.cur_width, self.cur_height)

        # 初始化 [訂閱資訊]
        self.uav_lat = self.uav_lon = self.uav_rel_alt = self.uav_heading = None # 飛機 [經度, 緯度, 高度]
        self.camera_roll = self.camera_pitch = self.camera_yaw = None            # 雲台 [roll, yaw, pitch]
        self.camera_zoom = None                                                  # 相機 [倍率]
        self.x_box_center = self.y_box_center = None                             # AI影像辨識 [方框中心點資訊]

        # QoS 定義
        qos = QoSProfile(
            reliability=ReliabilityPolicy(0),                # 使用 BEST EFFORT 模式
            durability=DurabilityPolicy.VOLATILE,            # 設定耐久性為 VOLATILE
            history=HistoryPolicy.KEEP_ALL,                  # 保留所有訊息
        )

        # ====================== 訂閱 ========================
        
        # 訂閱 飛機 [GPS] 數據
        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/raw/fix',
            self.gps_callback,
            qos_profile=qos)
        
        # 訂閱 飛機 [航向] 數據
        self.create_subscription(
            Float64,
            '/mavros/global_position/compass_hdg',
            self.heading_callback,
            qos_profile=qos)
        
        # 訂閱 飛機 [高度] 數據
        self.create_subscription(
            Float64,
            '/mavros/global_position/rel_alt',
            self.rel_alt_callback,
            qos_profile=qos)
        
        # 訂閱 相機 [roll, yaw, pitch, zoom] 數據
        self.create_subscription(
            Camera,
            '/camera_data_pub',
            self.camera_callback,
            qos_profile=qos)
        
        # 訂閱 AI影像辨識 [center_box] 數據
        self.create_subscription(
            CenterBox,
            '/box_center',
            self.center_box_callback,
            qos_profile=qos)
        
        # ====================== 發布 ========================
        self.pub_target_position = self.create_publisher(
            NavSatFix,
            '/target_position',
            10)
    
    # ====================== CallBack 回傳函數 ========================
    # 飛機 [GPS]
    def gps_callback(self, msg: NavSatFix):
        self.uav_lat = msg.latitude
        self.uav_lon = msg.longitude
        self.try_compute()

    # 飛機 [航向]
    def heading_callback(self, msg: Float64):
        self.uav_heading = msg.data
        self.try_compute()

    # 飛機 [相對高度]
    def rel_alt_callback(self, msg: Float64):
        self.uav_rel_alt = msg.data
        self.try_compute()

    # 相機 [(Angle度數 -> 弧度) & (相機倍率)]
    def camera_callback(self, msg: Camera):
        camera_data = msg.data[0]
        self.camera_roll    = math.radians(camera_data.rollangle)
        self.camera_yaw     = math.radians(camera_data.yawangle)
        self.camera_pitch   = math.radians(camera_data.pitchangle)
        self.camera_zoom    = round(camera_data.zoom_ratio, 1)
        if camera_data.zoom_ratio != self.cfg.zoom_s:
            self.cfg.zoom_s = camera_data.zoom_ratio
            self.intr.compute_scaled(self.cur_width, self.cur_height)
        self.try_compute()

    # AI影像辨識 [中心點方框資訊]
    def center_box_callback(self, msg: CenterBox):
        self.x_box_center = msg.x_center
        self.y_box_center = msg.y_center
        self.try_compute()

    # ====================== 目標經緯度回傳算法 ========================
    def try_compute(self):

        # 接收必要資訊計算
        if None in (
            self.uav_lat, self.uav_lon, self.uav_heading, self.uav_rel_alt,
            self.camera_roll, self.camera_pitch, self.camera_yaw, self.camera_zoom,
            self.x_box_center, self.y_box_center):
            return
        
        # 更新變焦倍率
        self.cfg.zoom_s = self.camera_zoom

        # 讀取當前解析度並計算內參
        w, h = self.reader.get_frame_size()
        self.intr.compute_scaled(w, h)
        fx, fy, u0, v0 = self.intr.fx, self.intr.fy, self.intr.u0, self.intr.v0

        # 入口像素
        u_px = self.x_box_center 
        v_px = self.y_box_center

        # Step1: 像素 -> 歸一化相機座標
        x = (u_px - u0) / fx
        y = (v_px - v0) / fy
        d_c = np.array([x, y, 1.0])

        # Step2. 計算世界系射線方向
        # 繞 x 軸旋轉
        def R_x(a): return np.array([[1,          0,           0],
                                     [0,math.cos(a),-math.sin(a)],
                                     [0,math.sin(a), math.cos(a)]])
        
        # 繞 y 軸旋轉
        def R_y(b): return np.array([[ math.cos(b),0, math.sin(b)],
                                     [0,           1,           0],
                                     [-math.sin(b),0, math.cos(b)]])
        
        # 繞 z 軸旋轉
        def R_z(c): return np.array([[ math.cos(c),-math.sin(c),0],
                                     [ math.sin(c), math.cos(c),0],
                                     [ 0,           0,          1]])
        
        R_WB = R_z(self.camera_yaw) @ R_y(self.camera_pitch) @ R_x(self.camera_roll)
        d_w_cam = R_WB @ d_c

        # Step3. 與地面 Z=0 求交 求出 z 軸方向向量 至 地面的 比例
        lam = - self.uav_rel_alt / d_w_cam[2]
        E_cam   = lam * d_w_cam[0]
        N_cam   = lam * d_w_cam[1]

        # Step4. 計算最終雲台絕對方位角 & 將計算二維向量 旋轉至 正東與正北 
        camera_yaw_adjustd = self.camera_yaw
        uav_heading_adjusted = math.radians(self.uav_heading)
        total_bearing_rad = (uav_heading_adjusted + camera_yaw_adjustd) % (2*math.pi)

        E =  E_cam * math.cos(total_bearing_rad) - N_cam * math.sin(total_bearing_rad)
        N =  E_cam * math.sin(total_bearing_rad) + N_cam * math.cos(total_bearing_rad)

        # Step5. ENU 經緯度
        R_M = 6378137.0
        R_P = R_M * math.cos(math.radians(self.uav_lat))
        target_lat = self.uav_lat + (N / R_M) * (180/math.pi)
        target_lon = self.uav_lon + (E / R_P) * (180/math.pi)

        # Step6. 發佈
        target_msg = NavSatFix()
        target_msg.latitude = round(target_lat, 7)   # 設定目標緯度
        target_msg.longitude = round(target_lon, 7)  # 設定目標經度
        self.pub_target_position.publish(target_msg) # 發布目標位置訊息

        print(target_msg.latitude,target_msg.longitude)

def main(args=None):
    rclpy.init(args=args)
    node = TargetGPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
