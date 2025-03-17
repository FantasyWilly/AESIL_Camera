#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
File   : camera_gui_ros2.py
Author : LYX(先驅), FantasyWilly
Email  : FantasyWilly - bc697522h04@gmail.com

相機型號 : KTG-TT30
檔案大綱 : 創建簡單 GUI 界面, 用於發送相機控制指令, 同時接收相機回傳資料發布至 ROS2
          
"""

# Python
import time
import threading
import tkinter as tk

# ROS2
import rclpy
from rclpy.node import Node

# ROS2 自定義消息包
from camera_msg_pkg.msg import Camera, CameraData
from camera_msg_pkg.msg import Laser, LaserData

# ROS2 引用 Python 檔 (mine)
import camera_tt30_pkg.camera_command as cm
import camera_tt30_pkg.camera_loop_command as loop_cm
from camera_tt30_pkg.camera_communication import CommunicationController
from camera_tt30_pkg.camera_decoder import ReceiveMsg

controller = CommunicationController("192.168.144.200", 2000)
gimbal_msg = ReceiveMsg()

# ----------------------- [CameraFeedbackPublisher] 初始化[Node], 宣告參數, 發布相機回傳資訊 -----------------------
class CameraFeedbackPublisher(Node):
    # Node       : camera_feedback_publisher_gui_node
    # Topic(PUB) : /camera_data_pub
    # Topic(PUB) : /laser_data_pub
    def __init__(self):
        super().__init__('camera_feedback_publisher_gui_node')

        self.declare_parameter('gimbal_step', 50)
        self.declare_parameter('zoom_duration', 0.3)
        self.declare_parameter('photo_continous_count', 3)

        self.gimbal_step = self.get_parameter('gimbal_step').get_parameter_value().integer_value
        self.zoom_duration = self.get_parameter('zoom_duration').get_parameter_value().double_value
        self.photo_continous_count = self.get_parameter('photo_continous_count').get_parameter_value().integer_value

        self.publisher_camera = self.create_publisher(Camera, '/camera_data_pub', 10)
        self.publisher_laser  = self.create_publisher(Laser, '/laser_data_pub', 10)

# ----------------------- [CameraControlGUI] 接收 [Node] 節點, 創建 [GUI] 界面 -----------------------
class CameraControlGUI:
    def __init__(self, master, ros_node):
        self.master = master
        self.ros_node = ros_node                    # 傳入 [CameraFeedbackPublisher] 的 [camera_feedback_publisher_gui_node] 節點
        self.master.title("Directional Buttons")
        self.create_widgets()

    # ----------------------- (create_widgets) 創建 [GUI] 容器 -----------------------
    def create_widgets(self):
        # 群組1: Common
        group1_frame = tk.Frame(self.master)
        group1_frame.pack(padx=10, pady=5, fill='x')
        group1_label = tk.Label(group1_frame, text="Common", font=("Helvetica", 12, "bold"),
                                 anchor="center", justify="center")
        group1_label.pack(side="top", fill="x", padx=5)
        group1_buttons = tk.Frame(group1_frame)
        group1_buttons.pack(fill='x', padx=5)
        btn_netural = tk.Button(group1_buttons, text="一鍵回中", command=self.on_netural)
        btn_netural.pack(side='left', padx=5, pady=5, expand=True, fill='x')
        btn_down = tk.Button(group1_buttons, text="一鍵向下", command=self.on_down)
        btn_down.pack(side='left', padx=5, pady=5, expand=True, fill='x')
        btn_follow = tk.Button(group1_buttons, text="跟隨機頭", command=self.on_follow_header)
        btn_follow.pack(side='left', padx=5, pady=5, expand=True, fill='x')

        # 群組2: Photo
        group2_frame = tk.Frame(self.master)
        group2_frame.pack(padx=10, pady=5, fill='x')
        group2_label = tk.Label(group2_frame, text="Photo", font=("Helvetica", 12, "bold"),
                                 anchor="center", justify="center")
        group2_label.pack(side="top", fill="x", padx=5)
        group2_buttons = tk.Frame(group2_frame)
        group2_buttons.pack(fill='x', padx=5)
        btn_take_photo = tk.Button(group2_buttons, text="拍一張照片", command=self.on_take_photo)
        btn_cont_shoot = tk.Button(group2_buttons, text="拍三張照片", command=self.on_continuous_shooting)
        btn_take_photo.pack(side='left', padx=5, pady=5, expand=True, fill='x')
        btn_cont_shoot.pack(side='left', padx=5, pady=5, expand=True, fill='x')

        # 群組3: Video
        group3_frame = tk.Frame(self.master)
        group3_frame.pack(padx=10, pady=5, fill='x')
        group3_label = tk.Label(group3_frame, text="Video", font=("Helvetica", 12, "bold"),
                                 anchor="center", justify="center")
        group3_label.pack(side="top", fill="x", padx=5)
        group3_buttons = tk.Frame(group3_frame)
        group3_buttons.pack(fill='x', padx=5)
        btn_start = tk.Button(group3_buttons, text="開始錄影", command=self.on_start_recording)
        btn_stop = tk.Button(group3_buttons, text="結束錄影", command=self.on_stop_recording)
        btn_start.grid(row=0, column=0, sticky="ew", padx=5, pady=5)
        btn_stop.grid(row=0, column=1, sticky="ew", padx=5, pady=5)
        group3_buttons.columnconfigure(0, weight=1)
        group3_buttons.columnconfigure(1, weight=1)

        # 群組4: Zoom
        group4_frame = tk.Frame(self.master)
        group4_frame.pack(padx=10, pady=5, fill='x')
        group4_label = tk.Label(group4_frame, text="Zoom", font=("Helvetica", 12, "bold"),
                                 anchor="center", justify="center")
        group4_label.pack(side="top", fill="x", padx=5)
        group4_buttons = tk.Frame(group4_frame)
        group4_buttons.pack(fill='x', padx=5)
        btn_reset = tk.Button(group4_buttons, text="恢復1倍", command=self.on_zoom_reset)
        btn_zoom_in = tk.Button(group4_buttons, text="放大2倍", command=self.on_zoom_in)
        btn_zoom_out = tk.Button(group4_buttons, text="縮小2倍", command=self.on_zoom_out)
        btn_zoom_keepin = tk.Button(group4_buttons, text="持續放大", command=self.on_zoom_keepin)
        btn_zoom_keepout = tk.Button(group4_buttons, text="持續縮小", command=self.on_zoom_keepout)
        btn_reset.pack(side='left', padx=5, pady=5, expand=True, fill='x')
        btn_zoom_in.pack(side='left', padx=5, pady=5, expand=True, fill='x')
        btn_zoom_out.pack(side='left', padx=5, pady=5, expand=True, fill='x')
        btn_zoom_keepin.pack(side='left', padx=5, pady=5, expand=True, fill='x')
        btn_zoom_keepout.pack(side='left', padx=5, pady=5, expand=True, fill='x')

        # 群組5: Laser
        group5_frame = tk.Frame(self.master)
        group5_frame.pack(padx=10, pady=5, fill='x')
        group5_label = tk.Label(group5_frame, text="Laser", font=("Helvetica", 12, "bold"),
                                 anchor="center", justify="center")
        group5_label.pack(side="top", fill="x", padx=5)
        group5_buttons = tk.Frame(group5_frame)
        group5_buttons.pack(fill='x', padx=5)
        btn_laser_open = tk.Button(group5_buttons, text="打開雷射", command=self.on_laser_open)
        btn_laser_close = tk.Button(group5_buttons, text="關閉雷射", command=self.on_laser_close)
        btn_laser_open.pack(side='left', padx=5, pady=5, expand=True, fill='x')
        btn_laser_close.pack(side='left', padx=5, pady=5, expand=True, fill='x')

        # 群組6: Direction
        group6_frame = tk.Frame(self.master)
        group6_frame.pack(padx=10, pady=5, fill='x')
        group6_label = tk.Label(group6_frame, text="Direction", font=("Helvetica", 12, "bold"),
                                anchor="center", justify="center")
        group6_label.pack(side="top", fill="x", padx=5)
        group6_buttons = tk.Frame(group6_frame)
        group6_buttons.pack(padx=5, pady=5)
        btn_up = tk.Button(group6_buttons, text="▲", width=3, height=2, command=self.on_indicator_up)
        btn_up.grid(row=0, column=1, padx=5, pady=5)
        btn_left = tk.Button(group6_buttons, text="◀", width=3, height=2, command=self.on_indicator_left)
        btn_left.grid(row=1, column=0, padx=5, pady=5)
        btn_right = tk.Button(group6_buttons, text="▶", width=3, height=2, command=self.on_indicator_right)
        btn_right.grid(row=1, column=2, padx=5, pady=5)
        btn_down = tk.Button(group6_buttons, text="▼", width=3, height=2, command=self.on_indicator_down)
        btn_down.grid(row=2, column=1, padx=5, pady=5)

    # ----------------- 各群組指令 -----------------
    def on_netural(self):
        print("[指令]: 一鍵回中", flush=True)
        cm.Command.Netural_command(controller)

    def on_down(self):
        print("[指令]: 一鍵向下", flush=True)
        cm.Command.Down_command(controller)
        
    def on_follow_header(self):
        print("[指令]: 跟隨機頭", flush=True)
        cm.Command.FollowHeader_command(controller)
        
    def on_take_photo(self):
        print("[指令]: 拍一張照片", flush=True)
        cm.Command.Photo_command(controller, 1, 0)
    
    def on_continuous_shooting(self):
        photo = self.ros_node.photo_continous_count
        print(f"[指令]: 拍 {photo} 張照片", flush=True)
        cm.Command.Photo_command(controller, 2, photo)

    def on_start_recording(self):
        print("[指令]: 開始錄影", flush=True)
        cm.Command.Video_command(controller, 1)
        
    def on_stop_recording(self):
        print("[指令]: 結束錄影", flush=True)
        cm.Command.Video_command(controller, 2)

    def on_zoom_keepin(self):
        zoom_duration = self.ros_node.zoom_duration
        print(f"[指令]: 持續放大 [控制時間]: {zoom_duration} 秒", flush=True)
        cm.Command.MachineZoom_command(controller, 1)
        time.sleep(zoom_duration)
        cm.Command.MachineZoom_command(controller, 3)
    
    def on_zoom_keepout(self):
        zoom_duration = self.ros_node.zoom_duration
        print(f"[指令]: 持續縮小 [控制時間]: {zoom_duration} 秒", flush=True)
        cm.Command.MachineZoom_command(controller, 2)
        time.sleep(zoom_duration)
        cm.Command.MachineZoom_command(controller, 3)
        
    def on_zoom_reset(self):
        print("[指令]: 恢復1倍", flush=True)
        cm.Command.MachineZoom_command(controller, 4)
        
    def on_zoom_in(self):
        print("[指令]: 放大2倍", flush=True)
        cm.Command.MachineZoom_command(controller, 5)

    def on_zoom_out(self):
        print("[指令]: 縮小2倍", flush=True)
        cm.Command.MachineZoom_command(controller, 6)
        
    def on_laser_close(self):
        print("[指令]: 關閉雷射", flush=True)
        cm.Command.Laser_command(controller, 0)
        
    def on_laser_open(self):
        print("[指令]: 打開雷射", flush=True)
        cm.Command.Laser_command(controller, 1)

    def on_indicator_up(self):
        step = self.ros_node.gimbal_step
        print(f"[指令]: 向上 [控制量]: {step/10}度", flush=True)
        cm.Command.GimbalControl_command(controller, 0, step)

    def on_indicator_down(self):
        step = self.ros_node.gimbal_step
        print(f"[指令]: 向下 [控制量]: {step/10}度", flush=True)
        cm.Command.GimbalControl_command(controller, 0, -step)

    def on_indicator_left(self):
        step = self.ros_node.gimbal_step
        print(f"[指令]: 向左 [控制量]: {step/10}度", flush=True)
        cm.Command.GimbalControl_command(controller, -step, 0)

    def on_indicator_right(self):
        step = self.ros_node.gimbal_step
        print(f"[指令]: 向右 [控制量]: {step/10}度", flush=True)
        cm.Command.GimbalControl_command(controller, step, 0)

# ----------------------- [BackgroundManager] 背景執行的多線程 -----------------------
class BackgroundManager:
    def __init__(self):
        # 初始化連線控制器與解析器
        self.controller = controller
        self.gimbal_msg = gimbal_msg
        # 初始化 ROS2 節點
        rclpy.init()
        self.ros_node = CameraFeedbackPublisher()
        self.stop_event = threading.Event()
        self.threads = []

    def start(self):
        # (1) 連線 & 發送雷射測距指令
        self.controller.connect()
        # print("[開啟]: 雷射測距 (等待 1 秒)")
        # cm.Command.Laser_command(self.controller, 1)
        time.sleep(1)

        # (2) 啟動 loop_in_background 線程（持續發送空命令）
        self.loop_thread = threading.Thread(
            target=loop_cm.loop_in_background,
            args=(self.controller, self.stop_event),
            daemon=True
        )
        self.loop_thread.start()
        self.threads.append(self.loop_thread)
        print("[開啟]: LOOP-CMD (等待 1 秒)")
        print("-----------------------------")
        time.sleep(1)

        # (3) 啟動 ROS2 的 spin 線程，讓節點持續運作
        self.ros_spin_thread = threading.Thread(
            target=lambda: rclpy.spin(self.ros_node),
            daemon=True
        )
        self.ros_spin_thread.start()
        self.threads.append(self.ros_spin_thread)
        print("[ROS2]: 節點啟動 (等待 1 秒)")
        print("-----------------------------")
        time.sleep(1)

        # (4) 啟動背景接收並發布資料到 ROS2 的線程
        self.ros_publish_thread = threading.Thread(
            target=self.receive_and_publish,
            daemon=True
        )
        self.ros_publish_thread.start()
        self.threads.append(self.ros_publish_thread)
        print("[背景線程] 開始接收並發布相機資料到 ROS2")

    # ----------------------- (receive_and_publish) 接收相機回傳資料並 傳至 ROS2 -----------------------
    def receive_and_publish(self):
        for packet in CommunicationController.recv_packets(self.controller.sock, packet_size=32, header=b'\x4B\x4B'):
            if self.stop_event.is_set():
                break
            if self.gimbal_msg.parse(packet, len(packet), self.gimbal_msg):
                camera_data = CameraData()
                camera_data.rollangle = float(self.gimbal_msg.rollAngle)
                camera_data.yawangle = float(self.gimbal_msg.yawAngle)
                camera_data.pitchangle = float(self.gimbal_msg.pitchAngle)

                camera_msg = Camera()
                camera_msg.data = [camera_data]
                self.ros_node.publisher_camera.publish(camera_msg)
                # self.ros_node.get_logger().info(
                #     f"[發布] Camera 資料: [ROLL]={camera_data.rollangle}, [YAW]={camera_data.yawangle}, [PITCH]={camera_data.pitchangle}"
                # )

                # -----------------------------------------------------------------------------------------

                target_distance = float(self.gimbal_msg.targetDist)
                if target_distance > 1500:
                    # print("超出範圍 (超過 1500) 的資料，忽略:", target_distance)
                    continue
                laser_data = LaserData()
                laser_data.targetdist = target_distance

                laser_msg = Laser()
                laser_msg.data = [laser_data]
                self.ros_node.publisher_laser.publish(laser_msg)
                # self.ros_node.get_logger().info(
                #     f"[發布] Laser 資料: [TARGET-DIST]={laser_data.targetdist}"
                # )

            else:
                print("無法解析資料")
            if self.stop_event.is_set():
                break
            time.sleep(0.05)
    
    # ---------- (shutdown) 通知所有背景線程停止並等待結束, 再關閉連線與 ROS2 ---------- 
    def shutdown(self):
        self.stop_event.set()
        for t in self.threads:
            t.join()
        self.controller.disconnect()
        rclpy.shutdown()

# ----------------------- [main] 主要執行序 -----------------------
def main():

    # 建立 AppManager 實例，並啟動所有背景線程
    try:
        background_manager = BackgroundManager()
        background_manager.start()
    except Exception as e:
        print("初始化錯誤:", e)
        return

    # 啟動 GUI 主迴圈 (root, ROS2節點)
    root = tk.Tk()
    gui = CameraControlGUI(root, background_manager.ros_node)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("使用者中斷程式執行")
    except Exception as e:
        print("發生例外:", e)
    finally:
        background_manager.shutdown()

if __name__ == "__main__":
    main()
