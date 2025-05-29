#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import threading
import subprocess
import shlex
import time
from queue import Queue, Empty

import rclpy
from rclpy.node import Node


class RTSPStreamerNode(Node):
    def __init__(self):
        """
        Args:
            camera_source (str): 原始影像來源 (RTSP).
            rtsp_server_url (str): 推流目標 RTSP 伺服器位址.
            fps (int): 輸出的影格率 (推流端).
        """

        self.camera_source = "rtsp://user:user@192.168.144.108:554/cam/realmonitor?channel=1&subtype=0"
        self.rtsp_server_url = "rtsp://192.168.0.230:8554/live/stream"
        self.fps = 25

        super().__init__('rtsp_streamer_node')
        self.get_logger().info("初始化 RTSPStreamerNode...")

        # 1) 建立 OpenCV VideoCapture
        self.cap = cv2.VideoCapture(self.camera_source)
        if not self.cap.isOpened():
            self.get_logger().error(f"無法連線到來源: {self.camera_source}")

        # 2) 建立佇列，用來儲存最新影格 (只留最新，避免延遲)
        self.q = Queue(maxsize=1)

        # 3) 啟動背景執行緒：不斷從 cap 讀影格，放進 Queue
        self.stop_thread = False
        self.capture_thread = threading.Thread(
            target=self._reader, 
            name="CaptureThread", 
            daemon=True
        )
        self.capture_thread.start()

        # 4) 讀取畫面大小 (若失敗，使用 640x480)
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if self.width == 0 or self.height == 0:
            self.get_logger().warn("讀取影像大小失敗，使用預設 640x480")
            self.width, self.height = 640, 480
        self.get_logger().info(f"來源影像大小: {self.width}x{self.height}")

        # 5) 建立 FFmpeg 推流 Process
        self.ffmpeg_process = self._setup_ffmpeg_process(self.rtsp_server_url, self.fps)

        # 6) 設定 ROS2 Timer (每 1/fps 秒 callback 一次)
        timer_period = 1.0 / float(self.fps)
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("RTSPStreamerNode 初始化完成。")

    def _reader(self):
        """背景執行緒：不斷從 self.cap 讀影格，放進 self.q (只留最新)。"""
        while not self.stop_thread:
            ret, frame = self.cap.read()
            if not ret:
                # 無法讀到影格時，小睡一下，避免卡 CPU
                time.sleep(0.01)
                continue

            # 如果 queue 有舊影格，丟棄
            if not self.q.empty():
                try:
                    self.q.get_nowait()
                except Empty:
                    pass
            
            self.q.put(frame)

    def timer_callback(self):
        """
        ROS2 Timer callback: 以 fps 頻率被呼叫。
        取出最新影格，透過 FFmpeg stdin 寫出 (raw bgr24)，實現推流。
        """
        if self.q.empty():
            # 暫時沒有新的影格
            return

        frame = self.q.get()
        # 直接把 raw bgr24 bytes 寫給 ffmpeg stdin
        try:
            self.ffmpeg_process.stdin.write(frame.tobytes())
        except BrokenPipeError:
            # 代表 ffmpeg process 已經斷線
            self.get_logger().error("FFmpeg pipe 已中斷，無法寫入影格。")

    def _setup_ffmpeg_process(self, rtsp_server_url, fps):
        """建立一個 FFmpeg process 把原始 bgr24 影像推到指定 RTSP 伺服器。"""
        ffmpeg_command = (
            "ffmpeg -y "
            f"-f rawvideo -pixel_format bgr24 -video_size {self.width}x{self.height} -framerate {fps} -i - "
            "-c:v libx264 -preset ultrafast -tune zerolatency "
            "-pix_fmt yuv420p "
            "-g 60 -keyint_min 60 "
            "-b:v 1M "
            "-bufsize 1M "
            "-max_delay 0 "
            "-an "
            f"-f rtsp {rtsp_server_url}"
        )

        # self.get_logger().info(f"啟動 FFmpeg 推流命令:\n{ffmpeg_command}")

        return subprocess.Popen(
            shlex.split(ffmpeg_command),
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

    def destroy_node(self):
        """
        關閉所有資源：
        1) 停止 capture_thread
        2) 關閉攝影機
        3) 關閉 FFmpeg process
        然後呼叫父類的 destroy_node
        """
        self.get_logger().info("正在關閉 RTSPStreamerNode...")

        # 1) 停止 capture thread
        self.stop_thread = True
        self.capture_thread.join(timeout=1.0)

        # 2) 關閉攝影機
        if self.cap.isOpened():
            self.cap.release()

        # 3) 關閉 FFmpeg
        if self.ffmpeg_process and self.ffmpeg_process.stdin:
            try:
                self.ffmpeg_process.stdin.close()
                self.ffmpeg_process.wait(timeout=1.0)
            except Exception as e:
                self.get_logger().error(f"關閉 FFmpeg process 時發生錯誤: {e}")

        # 最後呼叫父類 Node 的銷毀
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    # 建立節點
    node = RTSPStreamerNode()

    print("[main] Relay 已啟動, 按下 Ctrl+C 結束...")

    try:
        rclpy.spin(node)  # 進入 ROS2 spin，直到 Ctrl+C 或其他事件
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

        # 等待所有子執行緒結束
        for t in threading.enumerate():
            if t is not threading.current_thread():
                t.join(timeout=1.0)

if __name__ == "__main__":
    main()
