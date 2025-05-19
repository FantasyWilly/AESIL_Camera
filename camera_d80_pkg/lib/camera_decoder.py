#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
File   : camera_decoder.py
Author : FantasyWilly
Email  : bc697522h04@gmail.com

相機型號 : D-80 Pro
檔案大綱 : 
    A. 接收 資料
    B. 解碼 資料
"""

# ---------- (decode_gcu_response) TCP連接 ----------
def decode_gcu_response(response: bytes) -> dict:
    """    
    - 說明 (decode_gcu_response)
        1. 接收 回傳 16 進制數值
        2. 解碼 16 進制數值

    Args:
        ip (str)                  : 目標主機 IP
        port (int)                : 目標主機 Port
        timeout (float [可動參數]) : Socket 超時時間 (預設 5 秒)

    Returns:
        data: 解析後的資訊 (roll, yaw, pitch)
    """

    data = {}

    # (1) 先檢查至少要有 72 bytes 或更多 (視協議長度)
    if len(response) < 72:
        data['error'] = '封包長度不足，無法解析'
        return data

    # (2) 檢查協議頭 (假設是 0x8A 0x5E 或 0xA8 0x5E)
    header1, header2 = response[0], response[1]
    if not (header1 == 0x8A and header2 == 0x5E):
        data['error'] = f'協議頭錯誤: {header1:02X}{header2:02X}'
        return data

    # (3) 嘗試讀取 roll/pitch/yaw (Byte18~19, 20~21, 22~23)
    import struct
    
    # 先切出 raw bytes
    roll_bytes  = response[18:20]   # 2 bytes
    pitch_bytes = response[20:22]   # 2 bytes
    yaw_bytes   = response[22:24]   # 2 bytes

    # 有號 16-bit (S16): struct.unpack("<h") => big-endian 16位元有號
    roll_raw  = struct.unpack("<h", roll_bytes)[0]
    pitch_raw = struct.unpack("<h", pitch_bytes)[0]
    # 無號 16-bit (U16): struct.unpack("<H") => big-endian 16位元無號
    yaw_raw   = struct.unpack("<H", yaw_bytes)[0]

    # 分辨率 0.01
    roll_deg  = roll_raw  * 0.01
    pitch_deg = pitch_raw * 0.01
    yaw_deg   = yaw_raw   * 0.01

    data['roll']  = roll_deg
    data['pitch'] = pitch_deg
    data['yaw']   = yaw_deg

    # (4) 回傳
    return data
