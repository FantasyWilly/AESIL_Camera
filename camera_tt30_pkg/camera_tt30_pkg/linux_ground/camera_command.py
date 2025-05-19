#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
File   : camera_command.py
author : LYX(先驅), FantasyWilly
email  : FantasyWilly - bc697522h04@gmail.com

相機型號 : KTG-TT30
檔案大綱 : 控制命令
    1. 固定格式
    2. 命令程式控制
'''

# 引用 Python 檔 (mine)
from camera_communication import CommunicationController

# ------------------------------------- 固定程式 --------------------------------------------- 

# FIXED_BYTES   : KTG-TT30 雲台控制前面固定頭幀
FIXED_BYTES     = (b'\x4B\x4B\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x40\x88')

# 定義光源控制位元：可見光與熱成像
VISIBLE_LIGHT   = (b'\x01')       # 可見光
THERMAL_IMAGING = (b'\x02')       # 熱成像

# 補齊字元  
class Pad:
    def pad_to_8_bytes(bytes_check):
        assert isinstance(bytes_check,(bytes,bytearray)) and len (bytes_check) <= 23
        padding_needed = 23 -len(bytes_check)
        padding = bytes_check + b'\x00' * padding_needed

        return padding

# 校驗碼
class CrcTmp:
    def calc(data: bytes) -> int:
        crc_tmp = 0
        for b in data:
            crc_tmp += b
        return crc_tmp

# ------------------------------------- 命令控制 ---------------------------------------------

# 命令控制
class Command:

    # 指點變焦 (移動 & 放大2倍) - CMD(0x01)
    def Indicator_command(controller: CommunicationController, x_offset, y_offset):
        """
            參數:
                x_offset: int, 偏離圖像中心的橫向距離 [-10000, 10000]
                y_offset: int, 圖像中心的縱向距離偏離 [-10000, 10000]
        """

        send_bytes = bytearray(FIXED_BYTES)
        send_bytes = send_bytes + VISIBLE_LIGHT
        
        send_bytes = send_bytes + (b'\x01')

        x_offset_bytes = x_offset.to_bytes(2, byteorder='little', signed=True)
        y_offset_bytes = y_offset.to_bytes(2, byteorder='little', signed=True)

        send_bytes += x_offset_bytes
        send_bytes += y_offset_bytes

        send_bytes = Pad.pad_to_8_bytes(send_bytes)
        
        crc = CrcTmp.calc(send_bytes)
        send_bytes += crc.to_bytes(2, 'little')

        controller.send_command(send_bytes)
        
        return send_bytes

    # 跟隨機頭 - CMD(0x02)
    def FollowHeader_command(controller: CommunicationController):
        send_bytes = bytearray(FIXED_BYTES)
        send_bytes = send_bytes + VISIBLE_LIGHT
        
        send_bytes = send_bytes + (b'\x02')

        send_bytes = Pad.pad_to_8_bytes(send_bytes)

        crc = CrcTmp.calc(send_bytes)
        send_bytes += crc.to_bytes(2, 'little')

        controller.send_command(send_bytes)    

        return send_bytes

    # 回中 - CMD(0x03)
    def Netural_command(controller: CommunicationController):
        send_bytes = bytearray(FIXED_BYTES)
        send_bytes = send_bytes + VISIBLE_LIGHT
        
        send_bytes = send_bytes + (b'\x03')

        send_bytes = Pad.pad_to_8_bytes(send_bytes)

        crc = CrcTmp.calc(send_bytes)
        send_bytes += crc.to_bytes(2, 'little')

        controller.send_command(send_bytes)       

        return send_bytes
    
    # 雲台控制 - CMD(0x04)
    def GimbalControl_command(controller: CommunicationController, yaw_speed, pitch_speed):
        """
            參數:
                yaw_speed:   int, deg/s * 100 (範圍 -10000 ~ +10000)
                pitch_speed: int, deg/s * 100 (範圍 -10000 ~ +10000)
        """
        
        send_bytes = bytearray(FIXED_BYTES)
        send_bytes = send_bytes + VISIBLE_LIGHT

        send_bytes = send_bytes + b'\x04'

        yaw_speed_bytes = int(yaw_speed * 100).to_bytes(2, byteorder='little', signed=True)
        pitch_speed_bytes = int(pitch_speed * 100).to_bytes(2, byteorder='little', signed=True)

        send_bytes += yaw_speed_bytes
        send_bytes += pitch_speed_bytes

        send_bytes = Pad.pad_to_8_bytes(send_bytes)
        
        crc = CrcTmp.calc(send_bytes)
        send_bytes += crc.to_bytes(2, 'little')

        controller.send_command(send_bytes)
        
        return send_bytes
    
    # 開始跟蹤 - CMD(0x05)
    def StartTracking_command(controller: CommunicationController, center_x, center_y, length_x, width_y):
        """
            參數:
                center_x: int, 框中心點橫座標 [0 ~ 8191]
                center_y: int, 框中心點縱座標 [0 ~ 8191]
                length_x: int, 框在 x 方向的長度 (畫面中可見的寬度)
                width_y:  int, 框在 y 方向的高度
        """

        send_bytes = bytearray(FIXED_BYTES)
        send_bytes += VISIBLE_LIGHT
        
        send_bytes += b'\x05'

        center_x_bytes = center_x.to_bytes(2, byteorder='little', signed=False)
        center_y_bytes = center_y.to_bytes(2, byteorder='little', signed=False)
        
        length_x_16 = length_x // 16
        width_y_16  = width_y // 16
        
        length_x_bytes = length_x_16.to_bytes(2, byteorder='little', signed=False)
        width_y_bytes  = width_y_16.to_bytes(2, byteorder='little', signed=False)
        
        send_bytes += center_x_bytes
        send_bytes += center_y_bytes
        send_bytes += length_x_bytes
        send_bytes += width_y_bytes

        send_bytes = Pad.pad_to_8_bytes(send_bytes)
        
        crc = CrcTmp.calc(send_bytes)
        send_bytes += crc.to_bytes(2, 'little')

        controller.send_command(send_bytes)
        
        return send_bytes
    
    # 停止跟蹤 - CMD(0x06)
    def StopTracking_command(controller: CommunicationController):
        send_bytes = bytearray(FIXED_BYTES)
        send_bytes = send_bytes + VISIBLE_LIGHT
        
        send_bytes = send_bytes + (b'\x06')

        send_bytes = Pad.pad_to_8_bytes(send_bytes)

        crc = CrcTmp.calc(send_bytes)
        send_bytes += crc.to_bytes(2, 'little')

        controller.send_command(send_bytes)       

        return send_bytes
    
    # 向下 - CMD(0x07)
    def Down_command(controller: CommunicationController):
        send_bytes = bytearray(FIXED_BYTES)
        send_bytes = send_bytes + VISIBLE_LIGHT
        
        send_bytes = send_bytes + (b'\x07')

        send_bytes = Pad.pad_to_8_bytes(send_bytes)

        crc = CrcTmp.calc(send_bytes)
        send_bytes += crc.to_bytes(2, 'little')

        controller.send_command(send_bytes)       

        return send_bytes
    
    # 拍照 - CMD(0x010)
    def Photo_command(controller: CommunicationController, photo_mode, parameters):
        """
            photo_mode, parameters (int), 代表以下意義:
                (1) => 拍一張照 + (0)
                (2) => 相機連拍 + (parameters)連拍次數
                (3) => 延時拍照 + (parameters)延遲時間    
                (4) => 定時拍照 + (parameters)定時時間
                (5) => 停止拍照 + (0)
        """

        send_bytes = bytearray(FIXED_BYTES)
        send_bytes = send_bytes + VISIBLE_LIGHT
        
        send_bytes = send_bytes + (b'\x10')

        send_bytes += photo_mode.to_bytes(2, 'little')
        send_bytes += parameters.to_bytes(2, 'little')

        send_bytes = Pad.pad_to_8_bytes(send_bytes)

        crc = CrcTmp.calc(send_bytes)
        send_bytes += crc.to_bytes(2, 'little')

        controller.send_command(send_bytes)       

        return send_bytes
    
    # 錄影 - CMD(0x011)
    def Video_command(controller: CommunicationController, start_or_stop):
        """
            start_or_stop (int) 代表以下意義:
                (1) => 開始錄影
                (2) => 關閉錄影
        """

        send_bytes = bytearray(FIXED_BYTES)
        send_bytes = send_bytes + VISIBLE_LIGHT
        
        send_bytes = send_bytes + (b'\x11')

        send_bytes += start_or_stop.to_bytes(2, 'little')

        send_bytes = Pad.pad_to_8_bytes(send_bytes)

        crc = CrcTmp.calc(send_bytes)
        send_bytes += crc.to_bytes(2, 'little')

        controller.send_command(send_bytes)       

        return send_bytes
    
    # 機芯變焦 - CMD(0x12)
    def MachineZoom_command(controller: CommunicationController, zoom_code):
        """
            zoom_code (int) 代表以下意義:
                (1) => 持續放大
                (2) => 持續縮小
                (3) => 停止變焦
                (4) => 縮放=1 (恢復 1 倍)
                (5) => 放大 2 倍
                (6) => 縮小 2 倍
        """

        send_bytes = bytearray(FIXED_BYTES)
        send_bytes += VISIBLE_LIGHT
        
        send_bytes += (b'\x12')
        
        zoom_code_byte = zoom_code.to_bytes(1, byteorder='little', signed=False)
        send_bytes += zoom_code_byte
        
        send_bytes = Pad.pad_to_8_bytes(send_bytes)
        
        crc = CrcTmp.calc(send_bytes)
        send_bytes += crc.to_bytes(2, 'little')

        controller.send_command(send_bytes)
        
        return send_bytes
    
    # 機芯聚焦 - CMD(0x13)
    def MachineFocucomand(controller: CommunicationController, focus_code):
        """
            focus_code (int), 代表以下意義:
                (1) => 聚焦增加
                (2) => 聚焦減少
                (3) => 聚焦停止
                (4) => 自動聚焦
        """

        send_bytes = bytearray(FIXED_BYTES)
        send_bytes += VISIBLE_LIGHT
        
        send_bytes += (b'\x13')
        
        focus_code_byte = focus_code.to_bytes(1, byteorder='little', signed=False)
        send_bytes += focus_code_byte
        
        send_bytes = Pad.pad_to_8_bytes(send_bytes)
        
        crc = CrcTmp.calc(send_bytes)
        send_bytes += crc.to_bytes(2, 'little')

        controller.send_command(send_bytes)
        
        return send_bytes
    
    # 指點聚焦 - CMD(0x14)
    def PointFocucomand(controller: CommunicationController, center_x, center_y):
        """
            參數:
                center_x: int, 框中心點橫座標 [0 ~ 8191]
                center_y: int, 框中心點縱座標 [0 ~ 8191]
        """

        send_bytes = bytearray(FIXED_BYTES)
        send_bytes += VISIBLE_LIGHT
        
        send_bytes += (b'\x14')
        
        center_x_bytes = center_x.to_bytes(2, byteorder='little', signed=False)
        center_y_bytes = center_y.to_bytes(2, byteorder='little', signed=False)
        
        fixed_byte_5 = (25).to_bytes(1, byteorder='little')  # => b'\x19'
        fixed_byte_6 = (25).to_bytes(1, byteorder='little')  # => b'\x19'
        
        send_bytes += center_x_bytes
        send_bytes += center_y_bytes
        send_bytes += fixed_byte_5
        send_bytes += fixed_byte_6
        
        send_bytes = Pad.pad_to_8_bytes(send_bytes)
        
        crc = CrcTmp.calc(send_bytes)
        send_bytes += crc.to_bytes(2, 'little')

        controller.send_command(send_bytes)
        
        return send_bytes

    # 雷射測距開關 - CMD(0x21)
    def Laser_command(controller: CommunicationController, open_or_close):
        """
            open_or_close (int), 代表以下意義:
                (1) => 開起雷射
                (0) => 關閉雷射
        """

        send_bytes = bytearray(FIXED_BYTES)
        send_bytes = send_bytes + VISIBLE_LIGHT
        
        send_bytes = send_bytes + (b'\x21')

        send_bytes += open_or_close.to_bytes(2, 'little')

        send_bytes = Pad.pad_to_8_bytes(send_bytes)

        crc = CrcTmp.calc(send_bytes)
        send_bytes += crc.to_bytes(2, 'little')  

        controller.send_command(send_bytes)     

        return send_bytes
    