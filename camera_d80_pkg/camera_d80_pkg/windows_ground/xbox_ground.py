#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
檔案   : xbox_ground.py
作者   : FantasyWilly
Email  : bc697522h04@gmail.com

相機型號 : D-80 Pro
檔案大綱 : 
    A. 地面端透過 Xbox 控制器發送控制命令
"""

# Python
import time
import sys
import pygame

# 引用自定義程式
import camera_command as cm
from gcu_controller import GCUController


# ---------- 基本參數 (全域) ----------
DEVICE_IP = "192.168.0.230"     # Server IP
DEVICE_PORT = 9999              # Server Port 

CONTROL_INCREMENT = 5.0         # 雲台角度增量 (預設 5 度)

# ---------- (xbox_controller_loop) 添加 xbox 循環事件 ----------
def xbox_controller_loop(controller: GCUController) -> None:

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
                if event.button == 7:
                    print("按下按鈕 7, 程式將結束")
                    pygame.quit()
                    return

                # 處理按鈕 (button) 事件, 用以控制 [自定義按鈕]
                if joystick.get_button(0):
                    print("A 按鈕按下：向下")
                    cm.down(controller)
                elif joystick.get_button(1):
                    print("B 按鈕按下：拍照")
                    cm.photo(controller)
                elif joystick.get_button(2):
                    print("X 按鈕按下：錄影")
                    cm.video(controller)
                elif joystick.get_button(3):
                    print("Y 按鈕按下：回中")
                    cm.reset(controller)
                elif joystick.get_button(4):
                    print("L 按鈕按下：鎖定")
                    cm.lock(controller)
                elif joystick.get_button(5):
                    print("R 按鈕按下：跟隨")
                    cm.follow(controller)

            # 處理方向鍵 (hat) 事件, 用以控制 [雲台角度]
            elif event.type == pygame.JOYHATMOTION:
                hat = joystick.get_hat(0)
                if hat != (0, 0):
                    pitch = hat[1] * CONTROL_INCREMENT
                    yaw   = hat[0] * CONTROL_INCREMENT
                    print(f"Hat 更新：發送雲台控制指令 -> pitch: {pitch}°, yaw: {yaw}°")
                    cm.control_gimbal(controller, pitch=pitch, yaw=yaw)
            
            # 處理觸發器 (axis) 事件，用以控制 [相機放大縮小]
            elif event.type == pygame.JOYAXISMOTION:
                if event.axis == 5:
                    rt_value = joystick.get_axis(5)
                    if rt_value > 0.5:
                        print("RT 按下：放大")
                        cm.zoom_in(controller)
                    else:
                        print("RT 釋放：停止放大縮小")
                        cm.zoom_stop(controller)
                elif event.axis == 4:
                    lt_value = joystick.get_axis(4)
                    if lt_value > 0.5:
                        print("LT 按下：縮小")
                        cm.zoom_out(controller)
                    else:
                        print("LT 釋放：停止放大縮小")
                        cm.zoom_stop(controller)
        time.sleep(0.1)

# -------------------- [main] 主要執行序 --------------------
def main() -> None:

    # 連線 Server
    controller = GCUController(DEVICE_IP, DEVICE_PORT)

    try:
        controller.connect()
        print("[連線] 嵌入式電腦")
        
        xbox_controller_loop(controller)
        
    except Exception as e:
        print("[main] 出現錯誤:", e)
    finally:
        controller.disconnect()
        print("連線已關閉")

if __name__ == "__main__":
    main()
