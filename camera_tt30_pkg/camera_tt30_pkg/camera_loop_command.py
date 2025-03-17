#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
File   : camera_loop_command.py
author : FantasyWilly
email  : bc697522h04@gmail.com

相機型號 : KTG-TT30
檔案大綱 :
    A. 持續發送 回傳命令
'''

# Python
import time
import threading

# ROS2 引用 Python 檔 (mine)
from camera_tt30_pkg.camera_communication import CommunicationController
from camera_tt30_pkg.camera_decoder import ReceiveMsg

gimbal_msg=ReceiveMsg()

def loop_in_background(controller: CommunicationController, stop_event: threading.Event):
    while not stop_event.is_set():
        try:

            controller.loop_send_command(b'\x4B\x4B\x01\x97')

        except Exception as e:
            print("無法送出資料", e)

        time.sleep(0.05)
