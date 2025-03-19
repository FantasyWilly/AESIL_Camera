#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
檔案   : xbox_air.py
作者   : FantasyWilly
Email  : bc697522h04@gmail.com

相機型號 : KTG-TT30
檔案大綱 : 
    A. 地面端透過 Xbox 控制器發送控制命令
"""

# Python
import time
import sys
import pygame

# 引用 Python 檔 (mine)
import camera_command as cm
from camera_communication import CommunicationController

# ---------- 基本參數 (全域) ----------
DEVICE_IP = "192.168.0.230"     # Server IP
DEVICE_PORT = 9999              # Server Port

CONTROL_INCREMENT = 5.0         # 每次雲台調整的角度增量（單位：度）

def xbox_controller_loop(controller: CommunicationController) -> None:

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("未找到 Xbox 控制器")
        sys.exit(1)

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print("Xbox 控制器已啟動")

    # 進入事件循環
    while True:
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == 11:
                    print("按下按鈕 11, 程式將結束")
                    pygame.quit()
                    return

                # 處理按鈕 (button) 事件, 用以 [自定義功能]
                if joystick.get_button(0):
                    print("A 按鈕按下：向下")
                    cm.Command.Down_command(controller)
                elif joystick.get_button(1):
                    print("B 按鈕按下：拍照")
                    cm.Command.Photo_command(controller)
                elif joystick.get_button(2):
                    print("X 按鈕按下：跟隨機頭")
                    cm.Command.FollowHeader_command(controller)
                elif joystick.get_button(3):
                    print("Y 按鈕按下：回中")
                    cm.Command.Netural_command(controller)
                elif joystick.get_button(4):
                    print("L 按鈕按下：開始錄影")
                    cm.Command.Video_command(controller, 1)
                elif joystick.get_button(5):
                    print("R 按鈕按下：結束錄影")
                    cm.Command.Video_command(controller, 2)
                elif joystick.get_button(6):
                    print("按鈕按下：開啟雷射測距")
                    cm.Command.Laser_command(controller, 1)
                elif joystick.get_button(7):
                    print("按鈕按下：關閉雷射測距")
                    cm.Command.Laser_command(controller, 0)

            # 處理方向鍵 (hat) 事件, 用以 [控制雲台角度]
            elif event.type == pygame.JOYHATMOTION:
                hat = joystick.get_hat(0)
                if hat != (0, 0):
                    pitch = hat[1] * CONTROL_INCREMENT
                    yaw   = hat[0] * CONTROL_INCREMENT
                    print(f"Hat 更新：發送雲台控制指令 -> pitch: {pitch}°, yaw: {yaw}°")
                    cm.Command.GimbalControl_command(controller, pitch_speed=pitch * 10, yaw_speed=yaw * 10)
            
            # 處理觸發器 (axis) 事件，用以控制 [放大縮小]
            elif event.type == pygame.JOYAXISMOTION:
                if event.axis == 5:
                    rt_value = joystick.get_axis(5)
                    if rt_value > 0.5:
                        print("RT 按下：放大")
                        cm.Command.MachineZoom_command(controller, 1)
                    else:
                        print("RT 釋放：停止放大縮小")
                        cm.Command.MachineZoom_command(controller, 3)

                elif event.axis == 4:
                    lt_value = joystick.get_axis(4)
                    if lt_value > 0.5:
                        print("LT 按下：縮小")
                        cm.Command.MachineZoom_command(controller, 2)
                    else:
                        print("LT 釋放：停止放大縮小")
                        cm.Command.MachineZoom_command(controller, 3)
        time.sleep(0.1)

# ----------------------- [main] 主要執行序 -----------------------
def main() -> None:
    controller = CommunicationController(DEVICE_IP, DEVICE_PORT)
    try:
        controller.connect()
        print("[連線] 嵌入式第腦")
        
        xbox_controller_loop(controller)
        
    except Exception as e:
        print("[main] 出現錯誤:", e)
    finally:
        controller.disconnect()
        print("連線已關閉")

if __name__ == "__main__":
    main()
