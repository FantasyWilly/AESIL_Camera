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

# Python
import pygame          
import time            

pygame.init()           
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("未找到控制器")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print("控制器已啟動，請按下任一按鈕、方向鍵或操控搖桿")

while True:
    for event in pygame.event.get():

        # 處理按鈕按下事件
        if event.type == pygame.JOYBUTTONDOWN:
            for i in range(joystick.get_numbuttons()):
                if joystick.get_button(i):
                    print(f"按鈕 {i} 被按下")

        # 處理方向鍵 (hat) 事件
        elif event.type == pygame.JOYHATMOTION:
            hat_value = joystick.get_hat(0)
            print(f"Hat 狀態：{hat_value}")
            if hat_value[0] == -1:
                print("左方向")
            elif hat_value[0] == 1:
                print("右方向")
            if hat_value[1] == 1:
                print("上方向")
            elif hat_value[1] == -1:
                print("下方向")

        elif event.type == pygame.JOYAXISMOTION:
            num_axes = joystick.get_numaxes()
            axis_values = [joystick.get_axis(i) for i in range(num_axes)]
            print(f"搖桿軸值：{axis_values}")
    time.sleep(0.1)
