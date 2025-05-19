#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
File   : camera_communication.py
author : FantasyWilly
email  : bc697522h04@gmail.com

檔案大綱 : 
    A. 建立 / 關閉 TCP 連線
    B. 接收 回傳資訊
"""

# Python
import socket
import threading

# -------------------- [CommunicationController] 用於連接、發送指令和接收響應 --------------------
class CommunicationController:
    def __init__(self, ip: str, port: int, timeout: float = 5.0):
        """
        - 說明 [CommunicationController]
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
        print(f"正在連接 GCU: {self.ip}:{self.port}")

    # ---------- (disconnect) 關閉 TCP連接 ----------
    def disconnect(self) -> None:
        self.sock.close()
        print("連接已關閉")

    # ---------- (send_command) 發送 控制命令 ----------
    def send_command(self, cmd_bytes):
        try:
            self.sock.sendall(cmd_bytes)
            print("[發送]: 成功")
            print("-----------------------------")
        except socket.error as e:
            print("[發送]: !!失敗!!")
            print("[錯誤訊息]:", e)
            print("-----------------------------")

    # ---------- (send_command) 發送 回傳命令 ----------
    def loop_send_command(self, loop_cmd_bytes):
        try:
            self.sock.sendall(loop_cmd_bytes)

        except socket.error as e:
            print("[發送]: !!失敗!!")
            print("[錯誤訊息]:", e)
            print("-----------------------------")

    # ---------- (recv_packets) 接收 回傳資訊 ----------
    @staticmethod
    def recv_packets(sock, packet_size=32, header=b'\x4B\x4B'):
        """
        - 說明 *(recv_packets)*
            從 socket 中接收資料並提取完整封包

        Args:
            sock:           已連線的 socket 物件
            packet_size:    每個封包的位元組數
            header:         封包起始的標識位元
        """
        
        # 建立資料緩衝區
        buffer = bytearray()  
        while True:
            data = sock.recv(256)
            if not data:
                break
            buffer.extend(data)

            # 當緩衝區內資料足夠一個封包時，開始尋找封包頭
            while len(buffer) >= packet_size:
                start_idx = buffer.find(header)
                if start_idx == -1:
                    buffer = bytearray()
                    break
                
                # 刪除封包頭前的雜訊資料
                if start_idx > 0:
                    del buffer[:start_idx]  
                
                # 資料不足完整封包則等待更多資料
                if len(buffer) < packet_size:
                    break

                # 提取出一個完整封包
                packet = buffer[:packet_size]
                del buffer[:packet_size]
                yield packet         
