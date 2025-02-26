#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
File   : guc_loop.py
Author : FantasyWilly
Email  : bc697522h04@gmail.com

相機型號 : D-80 Pro
檔案大綱 : 背景不斷發送空命令, 查詢雲台角度 
"""

# Python3
import time
import threading

# ROS2
from camera_d80_pkg.gcu_controller import GCUController

# ---------- (loop_send_command) 循環執行程式 ----------
def loop_in_background(controller: GCUController, stop_event: threading.Event):
    """
    - 說明 *(loop_in_background)*
        1. 後台執行緒函式 不斷查詢雲台姿態 (roll/pitch/yaw)
        2. 當 stop_event 被 set 時 -> 跳出迴圈結束

    Args:
        time.sleep (float [可動參數]) : 多少秒執行一次 (預設0.5s)
    """
    while not stop_event.is_set():
        try:
            controller.loop_send_command(
                command=0x00,
                parameters=b'',
                include_empty_command=False,
                enable_request=True
            )
        except Exception as e:
            print("無法送出資料", e)

        time.sleep(0.5)
