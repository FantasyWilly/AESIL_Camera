#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
File   : camera_protocol.py
Author : FantasyWilly
Email  : bc697522h04@gmail.com

相機型號 : D-80 Pro
檔案大綱 : 
    A. CRC 計算
    B. 根據 [廠家手冊] 編寫 完整封包資訊
    C. 發送空命令 (回傳當前雲台資訊)
    !! 先發送控制命令 -> 空命令 !! (根據廠家手冊)
"""

# Python
import socket
import struct

# ---------- (calculate_crc) 計算 CRC 校驗碼 ----------
def calculate_crc(data: bytes) -> int:
    crc = 0
    crc_table = [
        0x0000, 0x1021, 0x2042, 0x3063,
        0x4084, 0x50A5, 0x60C6, 0x70E7,
        0x8108, 0x9129, 0xA14A, 0xB16B,
        0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    ]

    # 高4位和低4位分別進行 CRC 計算
    for byte in data:
        crc = ((crc << 4) ^ crc_table[(crc >> 12) ^ (byte >> 4)]) & 0xFFFF
        crc = ((crc << 4) ^ crc_table[(crc >> 12) ^ (byte & 0x0F)]) & 0xFFFF
    return crc

# ---------- (calculate_crc) 發送完整指令封包 架構 ----------
def build_packet(
    command: int, 
    parameters: bytes = None,
    include_empty_command: bool = False, 
    enable_request: bool = False,
    pitch: float = None,
    yaw: float = None
) -> bytes:
    
    """
    - 說明 (build_packet)
        1. 構建要傳送的協議封包

    Args:
        command (int)                : 指令代碼 (0x01, 0x20, ...)
        parameters (bytes)           : 指令的參數 (可空)
        include_empty_command (bool) : 是否在封包中添加空命令 (0x00)
        enable_request (bool)        : 是否啟用修改封包第 30 位元為 0x01

    Returns:
        bytes: 組好的完整封包 (包含 2 byte CRC)
    """

    # 協議頭
    header = b'\xA8\xE5'

    # 包長度占位符（後續填充實際值）
    length_bytes = b'\x00\x00'
    
    # 協議版本
    version = b'\x02'

    # 主幀和副幀（固定 32 bytes）
    main_frame = bytearray(b'\x00' * 32)
    sub_frame = bytearray(b'\x00' * 32)

    if command == 0x00 and (pitch is not None or yaw is not None):
        pitch_value = int(pitch * 100)
        yaw_value = int(yaw * 100)

        main_frame[2:4] = struct.pack('<h', pitch_value)
        main_frame[4:6] = struct.pack('<h', yaw_value)
        main_frame[6] = 0x04

    # 控制指令參數
    if parameters is None:
        parameters = b''

    # 空命令
    empty_command = b'\x00' if include_empty_command else b''

    # 先組建不含長度與 CRC 的初步載荷
    payload_without_length = (
        header +
        length_bytes +
        version +
        main_frame +
        sub_frame +
        command.to_bytes(1, 'little') +
        parameters +
        empty_command
    )

    # ----------------------------------------------------------------

    # 計算總長度（含 CRC 2 byte）
    total_length = len(payload_without_length) + 2
    length_bytes = total_length.to_bytes(2, 'little')

    # 重新組裝載荷（包含真實長度資訊）
    payload = bytearray(
        header +
        length_bytes +
        version +
        main_frame +
        sub_frame +
        command.to_bytes(1, 'little') +
        parameters +
        empty_command
    )

    # 如果 enable_request 為 True，則修改第 30 位元（index 從 0 開始）
    if enable_request:
        payload[30] = 0x01

    # 計算 CRC
    crc_value = calculate_crc(payload)
    crc_bytes = crc_value.to_bytes(2, 'big')

    return bytes(payload) + crc_bytes

# 傳送空命令  
def send_empty_command(sock: socket.socket) -> None:
    empty_packet = build_packet(command=0x00, parameters=b'', include_empty_command=True)
    print("發送 [數據包] :", empty_packet.hex().upper())
    sock.sendall(empty_packet)
