#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
File   : guc_main.py
Author : FantasyWilly
Email  : bc697522h04@gmail.com

相機型號 : D-80 Pro
檔案大綱 : 
    A. 創建簡單 CMD 鍵盤輸入控制
    B. 發送相機控制指令
    C. 接收相機回傳資料
"""

# Python
import threading

# ROS2
import camera_d80_pkg.gcu_loop as gcu_loop
import camera_d80_pkg.camera_command as cm
from camera_d80_pkg.gcu_controller import GCUController

# ---------- 基本參數(全域參數) ----------
DEVICE_IP = "192.168.168.121"
DEVICE_PORT = 2332

# -------------------- [main] 主要執行序 --------------------
def main():
    """
    - 說明 [main]
        1. 創建 [GCUController] 並 連線至 GCU控制盒
        2. 連續發送空命令
        3. 讓使用者輸入指令
        4. 執行對應功能, 直到輸入 quit 為止
    """

    # 建立 TCP 連線
    controller = GCUController(DEVICE_IP, DEVICE_PORT)

    try:
        # 建立 TCP 連線
        controller.connect()

        # 建立一個 stop_event 讓背景線程知道什麼時候要結束
        stop_event = threading.Event()

        # 開啟 [LOOP] 線程
        loop_thread = threading.Thread(
            target=gcu_loop.loop_in_background,
            args=(controller, stop_event),
            daemon=True
        )
        loop_thread.start()
        print("[LOOP] - 開始不斷發送空命令")

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

if __name__ == "__main__":
    main()
