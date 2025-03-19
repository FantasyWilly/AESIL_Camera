#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import threading
import subprocess
import shlex
import time
from queue import Queue, Empty

class RTSPRelay:
    def __init__(self):
        """
        Args:
            camera_source (str): 原始影像來源 (RTSP)。
            rtsp_server_url (str): 推流目標 RTSP 伺服器位址。
            fps (int): 推流的影格率。
        """
        self.camera_source = "rtsp://user:user@192.168.144.108:554/cam/realmonitor?channel=1&subtype=0"
        self.rtsp_server_url = "rtsp://192.168.0.230:8554/live/stream"
        self.fps = 25

        print(f"[RTSPRelay] 初始化, source={self.camera_source}, dest={self.rtsp_server_url}, fps={self.fps}")

        # 建立 OpenCV VideoCapture
        self.cap = cv2.VideoCapture(self.camera_source)
        if not self.cap.isOpened():
            print(f"[RTSPRelay] 無法連線到來源: {self.camera_source}")

        # 讀取畫面大小 (若失敗，使用 640x480)
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if self.width == 0 or self.height == 0:
            print("[RTSPRelay] 讀取影像大小失敗，使用預設 640x480")
            self.width, self.height = 640, 480

        print(f"[RTSPRelay] 來源影像大小: {self.width}x{self.height}")

        # 建立佇列，用來存放最新影格 (只保留 1 張，避免延遲)
        self.frame_queue = Queue(maxsize=1)

        # FFmpeg process
        self.ffmpeg_process = None

        # 控制執行緒的停止
        self.stop_flag = False

        # 執行緒物件
        self.capture_thread = None
        self.push_thread = None

    def start(self):
        """啟動兩條執行緒: 讀取影像 & 推流"""
        print("[RTSPRelay] 準備啟動 relay ...")

        # 建立 ffmpeg 子行程
        self.ffmpeg_process = self._setup_ffmpeg_process()

        # 啟動「讀取影像」執行緒
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()

        # 啟動「推流」執行緒
        self.push_thread = threading.Thread(target=self._push_loop, daemon=True)
        self.push_thread.start()

    def stop(self):
        """停止所有執行緒、關閉資源"""
        print("[RTSPRelay] 停止 relay ...")
        self.stop_flag = True

        # 等待執行緒結束
        if self.capture_thread:
            self.capture_thread.join(timeout=2.0)
        if self.push_thread:
            self.push_thread.join(timeout=2.0)

        # 釋放攝影機
        if self.cap and self.cap.isOpened():
            self.cap.release()

        # 關閉 FFmpeg
        if self.ffmpeg_process and self.ffmpeg_process.stdin:
            try:
                self.ffmpeg_process.stdin.close()
                self.ffmpeg_process.wait(timeout=2.0)
            except Exception as e:
                print(f"[RTSPRelay] 關閉 FFmpeg process 時發生錯誤: {e}")

        print("[RTSPRelay] 成功停止所有動作.")

    def _capture_loop(self):
        """讀取執行緒：不斷從 cap 讀影格，放進 frame_queue."""
        while not self.stop_flag:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.01)
                continue

            # 如果 queue 裡已有舊影格 => 丟棄它
            if not self.frame_queue.empty():
                try:
                    self.frame_queue.get_nowait()
                except Empty:
                    pass

            self.frame_queue.put(frame)

    def _push_loop(self):
        """推流執行緒：以 fps 頻率，從 queue 取出影格，寫入 ffmpeg_process.stdin."""
        interval = 1.0 / self.fps
        while not self.stop_flag:
            start_time = time.time()

            # 從 queue 取出一張影像
            if self.frame_queue.empty():
                # 如果暫時沒有影格 => 稍微睡一下
                time.sleep(0.005)
            else:
                frame = self.frame_queue.get()

                # 寫到 ffmpeg stdin (raw bgr24)
                try:
                    self.ffmpeg_process.stdin.write(frame.tobytes())
                except BrokenPipeError:
                    print("[RTSPRelay] FFmpeg pipe 已中斷，無法寫入影格。")
                    # 可能可以考慮重新啟動 ffmpeg，但這裡就直接終止
                    self.stop_flag = True
                    break

            # 控制推送頻率 (fps)
            elapsed = time.time() - start_time
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _setup_ffmpeg_process(self):
        """建立一個 FFmpeg process 把原始 bgr24 影像推到指定 RTSP 伺服器。"""
        ffmpeg_command = (
            "ffmpeg -y "
            f"-f rawvideo -pixel_format bgr24 -video_size {self.width}x{self.height} -framerate {self.fps} -i - "
            "-c:v libx264 -preset ultrafast -tune zerolatency "
            "-pix_fmt yuv420p "
            "-g 60 -keyint_min 60 "
            "-b:v 1M "
            "-bufsize 1M "
            "-max_delay 0 "
            "-an "
            f"-f rtsp {self.rtsp_server_url}"
        )

        print("[RTSPRelay] 啟動 FFmpeg 推流命令:")

        return subprocess.Popen(
            shlex.split(ffmpeg_command),
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

def main():

    # 建立 RTSPRelay 物件
    relay = RTSPRelay()
    # 啟動
    relay.start()

    print("[main] Relay 已啟動, 按下 Ctrl+C 結束...")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        relay.stop()
        print("[main] 程式結束")

if __name__ == "__main__":
    main()
