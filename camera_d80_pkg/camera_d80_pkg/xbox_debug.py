#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
File   : camera_command.py
Author : FantasyWilly
Email  : bc697522h04@gmail.com

相機型號 : D-80 Pro
檔案大綱 : 
    A. 根據 [廠家手冊] 編寫 控制命令
"""

import pygame           # **導入 pygame 模組，用來處理控制器事件**
import time             # **導入 time 模組，方便控制循環間隔**

pygame.init()           # **初始化 pygame 模組**
pygame.joystick.init()  # **初始化搖桿模組，使我們能夠讀取控制器訊息**

# **檢查是否有連接控制器，若無則印出提示並結束程式**
if pygame.joystick.get_count() == 0:
    print("未找到控制器")
    exit()

# **取得並初始化第一個連接的控制器**
joystick = pygame.joystick.Joystick(0)
joystick.init()
print("控制器已啟動，請按下任一按鈕、方向鍵或操控搖桿")

# **進入無限循環，持續監聽各種控制器事件**
while True:
    # **從事件佇列中取得所有事件**
    for event in pygame.event.get():
        # **處理按鈕按下事件**
        if event.type == pygame.JOYBUTTONDOWN:
            # **遍歷所有按鈕，檢查哪個按鈕被按下**
            for i in range(joystick.get_numbuttons()):
                if joystick.get_button(i):
                    print(f"按鈕 {i} 被按下")
        # **處理方向鍵 (hat) 事件**
        elif event.type == pygame.JOYHATMOTION:
            # **使用 get_hat(0) 取得方向鍵狀態，返回一個 (x, y) 的元組**
            hat_value = joystick.get_hat(0)
            print(f"Hat 狀態：{hat_value}")
            # **根據 x 值判斷左右方向**
            if hat_value[0] == -1:
                print("左方向")
            elif hat_value[0] == 1:
                print("右方向")
            # **根據 y 值判斷上下方向**
            if hat_value[1] == 1:
                print("上方向")
            elif hat_value[1] == -1:
                print("下方向")

        elif event.type == pygame.JOYAXISMOTION:
            num_axes = joystick.get_numaxes()
            axis_values = [joystick.get_axis(i) for i in range(num_axes)]
            print(f"搖桿軸值：{axis_values}")
    time.sleep(0.1)
