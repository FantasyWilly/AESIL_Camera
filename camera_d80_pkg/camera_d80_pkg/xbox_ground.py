#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
檔案   : ground_controller.py
作者   : FantasyWilly
Email  : bc697522h04@gmail.com

相機型號 : D-80 Pro
檔案大綱 : 
    A. 地面端透過 Xbox 控制器發送控制命令
    B. 使用原有封包格式（camera_command.py 與 camera_protocol.py）
    C. 當按下搖桿上的按鈕 11 時結束程式
    ※ 此程式不依賴 ROS，只負責透過 TCP 連線傳送命令與接收回應
"""

# Python
import time
import sys
import pygame

# ROS2
import camera_d80_pkg.camera_command as cm
from camera_d80_pkg.gcu_controller import GCUController


# ---------- 基本參數 (全域) ----------
# 如果你是直連相機，請使用相機 IP 與埠號；如果是透過代理服務，請改為代理服務的 IP 與埠號
DEVICE_IP = "192.168.0.230"     # 或改成代理服務所在的 IP
DEVICE_PORT = 9999              # 或代理服務的埠號

# 每次雲台調整的角度增量（單位：度）
CONTROL_INCREMENT = 5.0

def xbox_controller_loop(controller: GCUController) -> None:
    """
    使用 pygame 監聽 Xbox 控制器事件，
    依據控制器操作呼叫對應的命令函式，
    當按下按鈕 11 時結束程式。
    (這個版本是單線程的，所有事件都在主線程中處理)
    """
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
            # 處理按鈕按下事件
            if event.type == pygame.JOYBUTTONDOWN:
                # 若按下按鈕 11，結束程式
                if event.button == 7:
                    print("按下按鈕 7，程式將結束")
                    pygame.quit()
                    return  # 直接返回結束事件循環

                # 依據其他按鈕發送命令
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

            # 處理方向鍵 (hat) 事件，用以控制雲台角度
            elif event.type == pygame.JOYHATMOTION:
                hat = joystick.get_hat(0)
                if hat != (0, 0):
                    pitch = hat[1] * CONTROL_INCREMENT
                    yaw   = hat[0] * CONTROL_INCREMENT
                    print(f"Hat 更新：發送雲台控制指令 -> pitch: {pitch}°, yaw: {yaw}°")
                    cm.control_gimbal(controller, pitch=pitch, yaw=yaw)
            
            # 處理觸發器 (axis) 事件，用以控制 zoom
            elif event.type == pygame.JOYAXISMOTION:
                # 假設 RT 為軸 5；當值大於 0.5 表示按下，否則釋放
                if event.axis == 5:
                    rt_value = joystick.get_axis(5)
                    if rt_value > 0.5:
                        print("RT 按下：放大")
                        cm.zoom_in(controller)
                    else:
                        print("RT 釋放：停止放大縮小")
                        cm.zoom_stop(controller)
                # 假設 LT 為軸 2；當值大於 0.5 表示按下，否則釋放
                elif event.axis == 4:
                    lt_value = joystick.get_axis(4)
                    if lt_value > 0.5:
                        print("LT 按下：縮小")
                        cm.zoom_out(controller)
                    else:
                        print("LT 釋放：停止放大縮小")
                        cm.zoom_stop(controller)
        time.sleep(0.1)

def main() -> None:
    """
    主要流程：
      1. 建立與相機（或代理服務）的 TCP 連線，建立 GCUController 實例
      2. 呼叫 controller.connect() 連線
      3. 進入 Xbox 控制器事件循環
      4. 當事件循環結束（按下按鈕 11）後，斷開連線
    """
    controller = GCUController(DEVICE_IP, DEVICE_PORT)
    try:
        controller.connect()
        print("[連線] 嵌入式第腦")
        
        # 直接在主線程中進入事件循環
        xbox_controller_loop(controller)
        
    except Exception as e:
        print("[main] 出現錯誤:", e)
    finally:
        controller.disconnect()
        print("連線已關閉")

if __name__ == "__main__":
    main()
