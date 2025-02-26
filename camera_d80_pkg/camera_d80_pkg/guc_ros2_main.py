#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
File   : guc_ros2_main.py
Author : FantasyWilly
Email  : bc697522h04@gmail.com

相機型號 : D-80 Pro
檔案大綱 : 
    A. 創建簡單 CMD 鍵盤輸入控制
    B. 發送相機控制指令
    C. 同時接收相機回傳資料發布至 ROS2   
"""

# Python
import time
import threading

# ROS2
import rclpy
import camera_d80_pkg.camera_command as cm
import camera_d80_pkg.gcu_loop as gcu_loop
from camera_d80_pkg.guc_ros2_controller import GCUController
from camera_d80_pkg.gcu_ros2_publisher import GCUPublisher

# ---------- 基本參數(全域參數) ----------
DEVICE_IP = "192.168.168.121"
DEVICE_PORT = 2332

# -------------------- [main] 主要執行序 --------------------
def main():
    """
    - 說明 [main]
        1. 創建 [GCUController] 並 連線至 GCU控制盒
        2. 創建 [GCUPublisher] 並 創建 Node 節點 & 發布話題
        3. 連續發送空命令
        4. 讓使用者輸入指令 (reset/ photo / video / quit)
        5. 執行對應功能，直到輸入 quit 為止
    """

    # 初始化 ROS2 客戶端
    rclpy.init()

    # 建立 ROS2 節點實例 
    # 發布解碼資料 [Node]: gcu_publisher_node, [TOPIC]: gcu_response
    ros2_node = GCUPublisher()

    # 啟動背景執行緒以持續旋轉 (spin) ROS2 節點，確保其正常運作
    ros2_thread = threading.Thread(target=rclpy.spin, args=(ros2_node,), daemon=True)
    ros2_thread.start()

    print("[等待] ROS2 節點打開...")
    print("----------------------")
    time.sleep(1)

    # 建立TCP 連線與
    controller = GCUController(DEVICE_IP, DEVICE_PORT, ros2_publisher=ros2_node)

    try:

        # 正式開始連線
        controller.connect()

        # 建立 stop_event 讓背景線程知道什麼時候要結束
        stop_event = threading.Event()

        # 開啟 LOOP 線程，持續發送空命令
        loop_thread = threading.Thread(
            target=gcu_loop.loop_in_background,
            args=(controller, stop_event),
            daemon=True
        )
        loop_thread.start()
        print("[LOOP] - 開始不斷發送空命令")

        # 輸入控制指令
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
        controller.disconnect()
        ros2_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
