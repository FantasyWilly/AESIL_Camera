#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
檔案   : xbox_air.py
作者   : FantasyWilly
Email  : bc697522h04@gmail.com

相機型號 : D-80 Pro
檔案大綱 : 
    A. 在 Orin Nano 上啟動一個 TCP 代理服務, 監聽地面端連線
    B. 每次接收到命令時, 直接利用 controller 持久連線 轉發命令給相機
"""

# Python
import threading
import socketserver
import time

# ROS2
import rclpy
from camera_d80_pkg.gcu_ros2_publisher import GCUPublisher
from camera_d80_pkg.guc_ros2_controller import GCUController
import camera_d80_pkg.gcu_loop as gcu_loop

# ---------- 基本參數(全域參數) ----------
CAMERA_IP = "192.168.168.121"       # 相機 IP
CAMERA_PORT = 2332                  # 相機埠號

PROXY_LISTEN_IP = "192.168.0.230"   # 代理服務監聽的 IP
PROXY_LISTEN_PORT = 9999            # 代理服務監聽的埠號

ros2_publisher = None
controller = None

# ---------- (forward_command_to_camera) 發送命令至相機 ----------
def forward_command_to_camera(command_data: bytes) -> bytes:
    """
    - 說明 (forward_command_to_camera)
        利用 controller 持久連線轉發命令
        使用 controller.lock 保護，避免多執行緒同時存取
    """
    global controller
    try:
        with controller.lock:

            # 發送命令
            controller.sock.sendall(command_data)
            # print(f"[代理] 已發送命令：{command_data.hex().upper()}")

            # 接收命令
            response = controller.sock.recv(1024)
            # print(f"[代理] 從相機接收到回應：{response.hex().upper()}")

            return response
    except Exception as e:
        print(f"[代理] forward_command_to_camera 發生錯誤：{e}")
        return b''

# -------------------- [ProxyHandler] 接收地面端命令 --------------------
class ProxyHandler(socketserver.BaseRequestHandler):
    """
    - 說明 [ProxyHandler]
        1. 持續接收來自地面端的命令
        2. 對每次接收的命令, 透過 controller 的連線轉發給相機, 並將相機回應傳回給地面端
    """
    def handle(self):
        print(f"[代理] 接收到來自 {self.client_address} 的連線")
        while True:
            try:
                data = self.request.recv(1024)
                if not data:
                    print(f"[代理] 客戶端 {self.client_address} 已關閉連線")
                    break
                # print(f"[代理] 從地面端接收到命令：{data.hex().upper()}")
                response = forward_command_to_camera(data)
                if response:
                    self.request.sendall(response)
                    # print(f"[代理] 已將回應發送給地面端：{response.hex().upper()}")
                else:
                    print("[代理] 未獲得有效回應")
            except Exception as e:
                print(f"[代理] 處理連線錯誤：{e}")
                break

# -------------------- [ThreadedTCPServer] TCP代理 基礎設定 --------------------
class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    """
    - 說明 [ThreadedTCPServer]
        1. 繼承自 socketserver.ThreadingMixIn 與 socketserver.TCPServer
        2. socketserver.TCPServer : 是 Python 內建的 TCP 伺服器類別, 提供基本的 TCP 連線功能
        3. socketserver.ThreadingMixIn : 伺服器每收到一個連線請求就會啟動一個新的執行緒來處理, 可以同時處理多個客戶端連線
        4. allow_reuse_address = True : 這個設定允許伺服器在關閉後可以立即重新綁定同一個 IP 與埠
    """
    allow_reuse_address = True

# -------------------- (start_proxy_server) 啟動 TCP 代理 --------------------
def start_proxy_server(host: str, port: int) -> ThreadedTCPServer:
    """啟動 TCP 代理服務器"""
    server = ThreadedTCPServer((host, port), ProxyHandler)
    server_thread = threading.Thread(target=server.serve_forever)
    server_thread.daemon = True
    server_thread.start()
    print(f"[代理] 代理服務器已啟動，監聽 {host}:{port}")
    return server

# -------------------- [main] 主要執行序 --------------------
def main():

    global controller

    # 初始化 ROS2 節點與發布者
    rclpy.init()
    ros2_publisher = GCUPublisher()
    ros2_thread = threading.Thread(target=rclpy.spin, args=(ros2_publisher,), daemon=True)
    ros2_thread.start()

    print("[等待]: ROS2 節點打開...")
    print("-----------------------")
    time.sleep(1)

    # 建立與相機的 TCP 連線
    controller = GCUController(CAMERA_IP, CAMERA_PORT, ros2_publisher=ros2_publisher)
    controller.connect()

    # 啟動背景線程, 不斷發送空命令
    stop_event = threading.Event()
    loop_thread = threading.Thread(
        target=gcu_loop.loop_in_background,
        args=(controller, stop_event),
        daemon=True
    )
    loop_thread.start()
    print("[LOOP]: 開始不斷發送空命令...")
    print("---------------------------")
    time.sleep(1)

    # 啟動代理服務器
    proxy_server = start_proxy_server(PROXY_LISTEN_IP, PROXY_LISTEN_PORT)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("關閉代理服務器與退出")
        proxy_server.shutdown()
        proxy_server.server_close()
        stop_event.set()
        rclpy.shutdown()

if __name__ == "__main__":
    main()