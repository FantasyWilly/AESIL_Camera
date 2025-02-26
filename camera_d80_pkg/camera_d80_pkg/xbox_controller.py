#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
檔案   : guc_ros2_main.py
作者   : FantasyWilly
Email  : bc697522h04@gmail.com

相機型號 : D-80 Pro
檔案大綱 : 
    A. 創建簡單 CMD 鍵盤輸入控制
    B. 發送相機控制指令
    C. 同時接收相機回傳資料發布至 ROS2
    D. 新增：透過 Xbox 控制器按鍵控制與使用 get_hat 控制雲台 (每次 2 度)
"""

# Python 標準函式庫
import time
import threading

# ROS2 與相機指令相關模組
import rclpy
import camera_d80_pkg.camera_command as cm
import camera_d80_pkg.gcu_loop as gcu_loop
from camera_d80_pkg.guc_ros2_controller import GCUController
from camera_d80_pkg.gcu_ros2_publisher import GCUPublisher

# 新增：導入 pygame 以處理 Xbox 控制器事件
import pygame

# ---------- 基本參數(全域參數) ----------
DEVICE_IP = "192.168.168.121"
DEVICE_PORT = 2332

CONTROL_INCREMENT = 5.0

def xbox_controller_loop(controller):
    """
    **初始化 Xbox 控制器並持續檢測按鈕與方向鍵 (hat) 事件**
    """
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("未找到 Xbox 控制器")
        return

    # **取得第一個連接的控制器並初始化**
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print("Xbox 控制器已啟動")

    # **持續監聽控制器事件**
    while True:
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                if joystick.get_button(0):
                    print("A 按鈕按下: 向下")
                    cm.down(controller)
                elif joystick.get_button(1):
                    print("B 按鈕按下: 拍照")
                    cm.photo(controller)
                elif joystick.get_button(2):
                    print("X 按鈕按下: 錄影")
                    cm.video(controller)
                elif joystick.get_button(3):
                    print("Y 按鈕按下: 回中")
                    cm.reset(controller)
                elif joystick.get_button(4):
                    print("L 按鈕按下: 鎖頭")
                    cm.lock(controller)
                elif joystick.get_button(5):
                    print("R 按鈕按下: 跟隨")
                    cm.follow(controller)
    
            elif event.type == pygame.JOYHATMOTION:
                hat = joystick.get_hat(0)
                pitch = hat[1] * CONTROL_INCREMENT
                yaw   = hat[0] * CONTROL_INCREMENT
                if hat != (0, 0):
                    print(f"Hat 更新: 發送雲台控制指令 -> pitch: {pitch}°, yaw: {yaw}°")
                    cm.control_gimbal(controller, pitch=pitch, yaw=yaw)

            elif event.type == pygame.JOYAXISMOTION:
            # 若事件來自右扳機 (RT)，軸號為 5
                if event.axis == 5:
                    rt_value = joystick.get_axis(5)
                    # 當 RT 按下 (值大於 0.5) 時，發送放大指令
                    if rt_value > 0.5:
                        print("RT 按下: zoom_in")
                        cm.zoom_in(controller)
                    else:
                        # 當 RT 回到未按下狀態時，發送停止指令
                        print("RT 釋放: zoom_stop")
                        cm.zoom_stop(controller)
                
                # 若事件來自左扳機 (LT)，軸號為 2
                elif event.axis == 2:
                    lt_value = joystick.get_axis(2)
                    # 當 LT 按下 (值大於 0.5) 時，發送縮小指令
                    if lt_value > 0.5:
                        print("LT 按下: zoom_out")
                        cm.zoom_out(controller)
                    else:
                        # 當 LT 回到未按下狀態時，發送停止指令
                        print("LT 釋放: zoom_stop")
                        cm.zoom_stop(controller)
        time.sleep(0.1)

def main():
    """
    **主要流程說明**：
    1. 初始化 ROS2 客戶端並創建發布節點。
    2. 建立 GCU 控制器連線。
    3. 啟動背景線程持續發送空命令。
    4. 同時啟動一個線程來監聽 Xbox 控制器事件（包含按鈕與 hat 控制）。
    5. （可選）保留命令列輸入控制作為備用。
    """
    # **初始化 ROS2 客戶端**
    rclpy.init()

    # **建立 ROS2 節點實例 (發布 gcu_response 話題)**
    ros2_node = GCUPublisher()

    # **啟動背景線程以持續旋轉 (spin) ROS2 節點**
    ros2_thread = threading.Thread(target=rclpy.spin, args=(ros2_node,), daemon=True)
    ros2_thread.start()

    print("[等待] ROS2 節點打開...")
    print("----------------------")
    time.sleep(1)

    # **建立 TCP 連線，並創建 GCU 控制器實例**
    controller = GCUController(DEVICE_IP, DEVICE_PORT, ros2_publisher=ros2_node)

    try:
        # **連線至 GCU 控制盒**
        controller.connect()

        # **建立 stop_event 以便通知背景線程結束**
        stop_event = threading.Event()

        # **啟動 LOOP 線程，不斷發送空命令**
        loop_thread = threading.Thread(
            target=gcu_loop.loop_in_background,
            args=(controller, stop_event),
            daemon=True
        )
        loop_thread.start()
        print("[LOOP] - 開始不斷發送空命令")

        # **啟動 Xbox 控制器監聽線程**
        xbox_thread = threading.Thread(target=xbox_controller_loop, args=(controller,), daemon=True)
        xbox_thread.start()
        print("[XBOX] - Xbox 控制器監聽線程已啟動")

        # **保留命令列輸入控制（可選）**
        while True:
            cmd = input("請輸入指令 (empty/ reset/ photo / video / quit): ").strip().lower()
            if cmd == 'empty':
                cm.empty(controller)
            elif cmd == 'reset':
                cm.reset(controller)
            elif cmd == "photo":
                cm.photo(controller)
            elif cmd == "video":
                cm.video(controller)
            elif cmd == "quit":
                print("已退出操作。")
                break
            else:
                print("無效指令，請重新輸入。")

    except Exception as e:
        print("[main] 出現錯誤:", e)
    finally:
        # **關閉連線與釋放資源**
        controller.disconnect()
        ros2_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
