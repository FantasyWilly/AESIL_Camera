#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
File   : camera_gui.py
Author : LYX(先驅), FantasyWilly
Email  : FantasyWilly - bc697522h04@gmail.com

相機型號 : KTG-TT30
檔案大綱 : 
    A. 創建簡單 GUI 界面，用於發送相機控制指令
    B. 同時執行背景線程持續發送命令
    C. 整體框架採用 AppManager 封裝, 主程式只需呼叫即可
"""

# Python
import time
import threading
import tkinter as tk

# ROS2 引用 Python 檔 (mine)
import camera_tt30_pkg.camera_command as cm
import camera_tt30_pkg.camera_loop_command as loop_cm
from camera_tt30_pkg.camera_communication import CommunicationController
from camera_tt30_pkg.camera_decoder import ReceiveMsg

# ---------- 基本參數 (全域) ----------
controller = CommunicationController("192.168.144.200", 2000)
gimbal_msg = ReceiveMsg()

# ----------------- GUI 介面定義 -----------------
class CameraControlGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("Directional Buttons")
        self.create_widgets()

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
        print("[指令]: 一鍵回中")
        cm.Command.Netural_command(controller)

    def on_down(self):
        print("[指令]: 一鍵向下")
        cm.Command.Down_command(controller)

    def on_follow_header(self):
        print("[指令]: 跟隨機頭")
        cm.Command.FollowHeader_command(controller)

    def on_take_photo(self):
        print("[指令]: 拍一張照片")
        cm.Command.Photo_command(controller, 1, 0)

    def on_continuous_shooting(self):
        print("[指令]: 拍三張照片")
        cm.Command.Photo_command(controller, 2, 3)

    def on_start_recording(self):
        print("[指令]: 開始錄影")
        cm.Command.Video_command(controller, 1)

    def on_stop_recording(self):
        print("[指令]: 結束錄影")
        cm.Command.Video_command(controller, 2)

    def on_zoom_keepin(self):
        print("[指令]: 持續放大 [控制時間]: 0.25s")
        cm.Command.MachineZoom_command(controller, 1)
        time.sleep(0.25)
        cm.Command.MachineZoom_command(controller, 3)

    def on_zoom_keepout(self):
        print("[指令]: 持續縮小")
        cm.Command.MachineZoom_command(controller, 2)
        time.sleep(0.25)
        cm.Command.MachineZoom_command(controller, 3)

    def on_zoom_reset(self):
        print("[指令]: 恢復1倍")
        cm.Command.MachineZoom_command(controller, 4)

    def on_zoom_in(self):
        print("[指令]: 放大2倍")
        cm.Command.MachineZoom_command(controller, 5)

    def on_zoom_out(self):
        print("[指令]: 縮小2倍")
        cm.Command.MachineZoom_command(controller, 6)

    def on_laser_close(self):
        print("[指令]: 關閉雷射")
        cm.Command.Laser_command(controller, 0)

    def on_laser_open(self):
        print("[指令]: 打開雷射")
        cm.Command.Laser_command(controller, 1)

    def on_indicator_up(self):
        print("[指令]: 向上 [控制量]: 2.5度")
        cm.Command.GimbalControl_command(controller, 0, 25)

    def on_indicator_down(self):
        print("[指令]: 向下 [控制量]: 2.5度")
        cm.Command.GimbalControl_command(controller, 0, -25)

    def on_indicator_left(self):
        print("[指令]: 向左 [控制量]: 2.5度")
        cm.Command.GimbalControl_command(controller, -25, 0)

    def on_indicator_right(self):
        print("[指令]: 向右 [控制量]: 2.5度")
        cm.Command.GimbalControl_command(controller, 25, 0)

# ----------------- AppManager: 封裝所有背景線程 -----------------
class AppManager:
    def __init__(self, controller):
        self.controller = controller
        self.stop_event = threading.Event()
        self.threads = []

    def start(self):
        # (1) 連線與發送雷射測距指令
        self.controller.connect()
        print("[開啟]: 雷射測距 (等待 1 秒)")
        cm.Command.Laser_command(self.controller, 1)
        time.sleep(1)

        # (2) 啟動 loop_in_background 線程（持續發送空命令）
        self.loop_thread = threading.Thread(
            target=loop_cm.loop_in_background,
            args=(self.controller, self.stop_event),
            daemon=True
        )
        self.loop_thread.start()
        self.threads.append(self.loop_thread)
        print("[開啟]: LOOP-CMD")

        # (3) 啟動背景接收並接收資料線程
        self.receive_thread = threading.Thread(
            target=self.receive,
            daemon=True
        )
        self.receive_thread.start()
        self.threads.append(self.receive_thread)
        print("[背景線程] 開始接收並發布相機資料")

    def receive(self):
        for packet in CommunicationController.recv_packets(controller.sock, packet_size=32, header=b'\x4B\x4B'):
            if self.stop_event.is_set():
                break
            if gimbal_msg.parse(packet, len(packet), gimbal_msg):
                print(f"[ROLL]={gimbal_msg.rollAngle} 度, [YAW]={gimbal_msg.yawAngle} 度, [PITCH]={gimbal_msg.pitchAngle} 度")
                print(f"[TARGET-DIST]={gimbal_msg.targetDist} m")
            else:
                print("無法解析資料")

            time.sleep(0.05)

    def shutdown(self):
        # 通知所有背景線程停止，並等待結束，再關閉連線
        self.stop_event.set()
        for t in self.threads:
            t.join()
        self.controller.disconnect()

# ----------------- 主要執行序 -----------------
def main():
    try:
        # 建立 AppManager 實例，並啟動背景線程
        app_manager = AppManager(controller)
        app_manager.start()
    except Exception as e:
        print("初始化錯誤:", e)
        return

    # 啟動 GUI 主迴圈
    root = tk.Tk()
    gui = CameraControlGUI(root)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("使用者中斷程式執行")
    except Exception as e:
        print("發生例外:", e)
    finally:
        app_manager.shutdown()

if __name__ == "__main__":
    main()
