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

# ROS2
from gcu_controller import GCUController

# ---------- (empty) 空命令 ----------
def empty(controller: GCUController) -> None:
    print("發送 [指令] : [empty] - 空指令")
    try:
        controller.send_command(
            command=0x00,
            parameters=b'',
            include_empty_command=False,
            enable_request=True
        )
    except Exception as e:
        print("[empty] 發送回中指令時出現錯誤:", e)

# ---------- (reset) 回中 ----------
def reset(controller: GCUController) -> None:
    print("發送 [指令] : [reset] - 回中")
    try:
        controller.send_command(
            command=0x03,
            parameters=b'',
            include_empty_command=False,
            enable_request=True
        )
    except Exception as e:
        print("[reset] 發送回中指令時出現錯誤:", e)

# ---------- (down) 俯拍 ----------
def down(controller: GCUController) -> None:
    print("發送 [指令] : [down] - 俯拍")
    try:
        controller.send_command(
            command=0x13,
            parameters=b'',
            include_empty_command=False,
            enable_request=True
        )
    except Exception as e:
        print("[reset] 發送回中指令時出現錯誤:", e)

# ---------- (photo) 拍照 ----------
def photo(controller: GCUController) -> None:
    print("發送 [指令] : [photo] - 拍照")
    try:
        controller.send_command(
            command=0x20,
            parameters=b'\x01',
            include_empty_command=False,
            enable_request=True
        )
    except Exception as e:
        print("[photo] 發送拍照指令時出現錯誤:", e)

# ---------- (video) 錄影 ----------
def video(controller: GCUController) -> None:
    print("發送 [指令] : [video] - 錄影")
    try:
        controller.send_command(
            command=0x21,
            parameters=b'\x01',
            include_empty_command=False,
            enable_request=True
        )
    except Exception as e:
        print("[video] 發送錄影指令時出現錯誤:", e)

# ---------- (lock) 鎖定 ----------
def lock(controller: GCUController) -> None:
    print("發送 [指令] : [lock] - 鎖定")
    try:
        controller.send_command(
            command=0x11,
            parameters=b'',
            include_empty_command=False,
            enable_request=True
        )
    except Exception as e:
        print("[video] 發送錄影指令時出現錯誤:", e)

# ---------- (follow) 鎖定 ----------
def follow(controller: GCUController) -> None:
    print("發送 [指令] : [follow] - 跟隨")
    try:
        controller.send_command(
            command=0x12,
            parameters=b'',
            include_empty_command=False,
            enable_request=True
        )
    except Exception as e:
        print("[video] 發送錄影指令時出現錯誤:", e)

# ---------- (zoom_in) 連續放大 ----------
def zoom_in(controller: GCUController) -> None:
    print("發送 [指令] : [zoom_in] - 連續放大")
    try:
        controller.send_command(
            command=0x22,
            parameters=b'\x01',
            include_empty_command=False,
            enable_request=True
        )
    except Exception as e:
        print("[video] 發送錄影指令時出現錯誤:", e)

# ---------- (zoom_out) 連續縮小 ----------
def zoom_out(controller: GCUController) -> None:
    print("發送 [指令] : [zoom_out] - 連續縮小")
    try:
        controller.send_command(
            command=0x23,
            parameters=b'\x01',
            include_empty_command=False,
            enable_request=True
        )
    except Exception as e:
        print("[video] 發送錄影指令時出現錯誤:", e)

# ---------- (zoom_stop) 停止放大縮小 ----------
def zoom_stop(controller: GCUController) -> None:
    print("發送 [指令] : [zoom_stop] - 停止放大縮小")
    try:
        controller.send_command(
            command=0x24,
            parameters=b'\x01',
            include_empty_command=False,
            enable_request=True
        )
    except Exception as e:
        print("[video] 發送錄影指令時出現錯誤:", e)

# ---------- (control_gimbal) 控制雲台角度 ----------
def control_gimbal(controller: GCUController, pitch: float, yaw: float) -> None:
    print(f"發送 [指令] : [control_gimbal] - 控制雲台, pitch: {pitch}°, yaw: {yaw}°")
    try:
        controller.send_command(
            command=0x00,            # **設定指令代碼為 0x10 代表控制雲台角度**
            parameters=b'',          # **若有其他參數需求, 可在此填入, 否則空值**
            include_empty_command=True,
            enable_request=True,
            pitch=pitch,             # **傳入俯仰角參數(單位：度)**
            yaw=yaw                  # **傳入偏航角參數(單位：度)**
        )
    except Exception as e:
        print("[control_gimbal] 發送雲台控制指令時出現錯誤:", e)