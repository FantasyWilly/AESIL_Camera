#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
File   : gcu_controller.py
Author : FantasyWilly
Email  : bc697522h04@gmail.com

相機型號 : D-80 Pro
檔案大綱 : 
    A. 管理TCP連線
    B. 發送控制命令
    C. 連續發送空命令 & 解碼回傳資訊
"""

# Python
import socket
import threading
import time

# ROS2
from camera_protocol import build_packet, send_empty_command
from camera_decoder import decode_gcu_response

# -------------------- [GCUController] 用於連接、發送指令和接收響應 --------------------
class GCUController:
    def __init__(self, ip: str, port: int, timeout: float = 5.0):
        """
        - 說明 [GCUController]
            1. 接收 IP, Port 參數
            2. 管理 TCP 連線
            3. 發送 控制命令

        Args:
            ip (str)                  : 目標主機 IP
            port (int)                : 目標主機 Port
            timeout (float, [可動參數]) : Socket 超時時間 (預設 5 秒)
        """

        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(timeout)

        # 確保一次只能有一個執行緒在呼叫 send_command()
        self.lock = threading.Lock()

    # ---------- (connect) 開啟 TCP連接 ----------
    def connect(self) -> None:
        self.sock.connect((self.ip, self.port))
        print(f"已連接到 GCU: {self.ip}:{self.port}")

    # ---------- (disconnect) 關閉 TCP連接 ----------
    def disconnect(self) -> None:
        self.sock.close()
        print("連接已關閉")

        # ---------- (send_command) 發送 控制命令 ----------
    def send_command(
        self, 
        command: int, 
        parameters: bytes = b'', 
        include_empty_command: bool = False, 
        enable_request: bool = False,
        pitch: float = None,
        yaw: float = None
    ) -> bytes:

        # (1) 構建數據包並發送
        packet = build_packet(command, parameters, include_empty_command, enable_request, pitch=pitch, yaw=yaw)
        print("發送 [數據包] :", packet.hex().upper())
        self.sock.sendall(packet)

        # (2) 接收本次指令的回覆
        response = self.sock.recv(256)
        print("接收 [返回數據] :", response.hex().upper())

        # (3) 解碼本次指令回覆
        parsed = decode_gcu_response(response)
        if 'error' in parsed:
            print("解碼失敗:", parsed['error'])
        else:
            print(f"接收 [解碼] : roll={parsed['roll']:.2f}, pitch={parsed['pitch']:.2f}, yaw={parsed['yaw']:.2f}")

        time.sleep(0.02)

        # (4) 空命令收尾
        if command != 0x00 or command == 0x00 and (pitch is not None or yaw is not None):
            print("發送 [指令] : [empty] - 空命令")
            send_empty_command(self.sock)

        return response
    
    # ---------- (loop_send_command) 發送空命令 ----------
    def loop_send_command(
        self, 
        command: int, 
        parameters: bytes = b'', 
        include_empty_command: bool = False, 
        enable_request: bool = False
    ) -> bytes:
        
        # (1) 構建數據包並發送
        packet = build_packet(command, parameters, include_empty_command, enable_request)
        self.sock.sendall(packet)

        # (2) 接收本次指令的回覆
        response = self.sock.recv(256)
        #print("接收 [返回數據] :", response.hex().upper())

        # (3) 解碼本次指令回覆
        parsed = decode_gcu_response(response)
        if 'error' in parsed:
            print("解碼失敗:", parsed['error'])
        else:
            roll = parsed['roll']
            pitch = parsed['pitch']
            yaw = parsed['yaw']
            print(f"接收 [解碼] : roll={parsed['roll']:.2f}, pitch={parsed['pitch']:.2f}, yaw={parsed['yaw']:.2f}")

        return response    