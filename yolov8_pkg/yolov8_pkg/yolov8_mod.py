#!/usr/bin/env python3

# Python 套件
import cv2
import threading
import subprocess
import shlex
import time
from queue import Queue, Empty

# YOLO
from ultralytics import YOLO

# ROS2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from yolo_msg_pkg.msg import BoundingBox, CenterBox

class VideoCapture:
    def __init__(self, camera_source):
        # **初始化 OpenCV 的 VideoCapture 物件，用來讀取指定的攝影機或串流來源**
        self.cap = cv2.VideoCapture(camera_source)
        
        # **建立一個大小為 1 的佇列 (Queue)，只保留最新的一幀 (frame)**
        self.q = Queue(maxsize=1)
        
        # **用於控制讀取執行緒的中止**
        self.stop_thread = False
        
        # **建立並啟動攝影機讀取的子執行緒**
        self.thread = threading.Thread(target=self._reader, name="VideoCaptureThread")
        self.thread.daemon = True      # 將執行緒設為 "Daemon" - 主程式結束時自動終止
        self.thread.start()
    
    # 取得相機畫面的寬高
    def get_frame_size(self):
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return width, height

    # 持續從攝影機讀取影像並存入佇列中
    def _reader(self):
        while not self.stop_thread:
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            # **如果佇列裡已有舊影像 就丟棄 保留最新的一幀 - Queue(maxsize=1)**
            if not self.q.empty():
                try:
                    self.q.get_nowait()
                except Empty:
                    pass
            
            self.q.put(frame)

    # 從佇列中取得最新的一幀影像
    def read(self):
        return self.q.get()

    # 中止執行緒並釋放攝影機資源
    def release(self):
        self.stop_thread = True
        self.thread.join()
        self.cap.release()

class YoloRtspRosNode(Node):
    def __init__(self):
        # **初始化 ROS2 的 Node，節點名稱為 'yolo_rtsp_ros_node'**
        super().__init__('yolov8_mod_node')

        # --------------------------------------------------------------------------
        # **宣告可修改參數 (使用者可透過 YAML/launch 檔改動這些值)**
        # --------------------------------------------------------------------------
        self.declare_parameter('model_path', '/home/fantasywilly/weight/Car_Model.pt')
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('conf_thresh', 0.5)
        self.declare_parameter('camera_source', 'rtsp://user:user@192.168.144.108:554/cam/realmonitor?channel=1&subtype=0')
        self.declare_parameter('rtsp_server_url', 'rtsp://192.168.0.230:8554/live/stream')
        self.declare_parameter('frame_rate', 30)

        # --------------------------------------------------------------------------
        # **從參數伺服器讀取值 (若 YAML/launch 有給值，會覆蓋此處的預設)**
        # --------------------------------------------------------------------------
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.imgsz = self.get_parameter('imgsz').get_parameter_value().integer_value
        self.conf_thresh = self.get_parameter('conf_thresh').get_parameter_value().double_value
        self.camera_source = self.get_parameter('camera_source').get_parameter_value().string_value
        self.rtsp_server_url = self.get_parameter('rtsp_server_url').get_parameter_value().string_value
        self.frame_rate = self.get_parameter('frame_rate').get_parameter_value().integer_value
        # --------------------------------------------------------------------------

        # **初始化用來計算 FPS 的變數**
        self.frame_count = 0
        self.start_time = time.time()
        self.fps = 0.0

        # **初始化 CvBridge，用於在 ROS 與 OpenCV 影像之間做轉換**
        self.bridge = CvBridge()

        # **載入 YOLOv8 模型**
        self.get_logger().info("正在加載 YOLOv8 模型...")
        self.model = YOLO(self.model_path)
        self.get_logger().info("YOLOv8 模型加載完成")

        # **初始化 ROS2 的 Publisher (發布者)，用於發布 BoundingBox、CenterBox 以及影像**
        self.box_pub = self.create_publisher(BoundingBox, "/box", 10)
        self.box_center_pub = self.create_publisher(CenterBox, "/box_center", 10)

        # **初始化攝影機讀取物件，並取得畫面尺寸**
        self.cap = VideoCapture(self.camera_source)
        width, height = self.cap.get_frame_size()
        self.get_logger().info(f"影像尺寸: {width}x{height}")

        # **建立處理影格與結果的佇列**
        self.frame_queue = Queue(maxsize=1)
        self.result_queue = Queue(maxsize=1)

        # **開兩個子執行緒，分別執行 yolo_predict() 與 publish_results() 方法**
        threading.Thread(target=self.yolo_predict, name="YoloPredictThread").start()
        threading.Thread(target=self.publish_results, name="PublishResultsThread").start()

        # **啟動 FFmpeg 進程，用於將影像推送到 RTSP 伺服器**
        self.ffmpeg_process = self.setup_ffmpeg_process(width, height)

    def setup_ffmpeg_process(self, width, height):
        # **此處組裝 FFmpeg 的命令列，用來推流到指定的 rtsp_server_url**
        
        ffmpeg_command = (
            'ffmpeg -y '
            '-f rawvideo -pixel_format bgr24 '
            f'-video_size {width}x{height} '
            f'-framerate {self.frame_rate} '
            '-i - '
            '-c:v libx264 '
            '-preset ultrafast '
            '-tune zerolatency '
            '-pix_fmt yuv420p '
            '-x264-params "bframes=0" '
            '-g 25 -keyint_min 25 '
            '-b:v 4M '
            '-bufsize 4M '
            '-max_delay 0 '
            '-an '
            f'-f rtsp {self.rtsp_server_url}'
        )


        self.get_logger().info(f"FFmpeg command: {ffmpeg_command}")  # **顯示 FFmpeg 完整指令，可供除錯**
        
        # **使用 subprocess.Popen 執行 FFmpeg，stdin 用於接收影像數據**
        return subprocess.Popen(
            shlex.split(ffmpeg_command),
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

    def yolo_predict(self):
        # **此函式在子執行緒中不斷地進行以下流程：**
        # 1. 讀取攝影機最新的影格
        # 2. 用 YOLO 模型進行推論
        # 3. 放進 result_queue 供後續處理 (publish_results) 使用

        while rclpy.ok():
            # **讀取最新影格**
            frame = self.cap.read()
            if frame is None:
                continue

            # **維持佇列中只保留最新影格**
            if not self.frame_queue.empty():
                try:
                    self.frame_queue.get_nowait()
                except Empty:
                    pass
            self.frame_queue.put(frame)

            # **進行 YOLO 推論 (predict)**
            results = self.model.predict(
                source=frame, 
                device=self.device, 
                imgsz=self.imgsz, 
                conf=self.conf_thresh
            )

            # **計算並印出 FPS**
            self.frame_count += 1
            current_time = time.time()
            elapsed_time = current_time - self.start_time
            if elapsed_time >= 1.0:
                self.fps = self.frame_count / elapsed_time
                self.get_logger().info(f"當前 FPS: {self.fps:.2f}")
                self.frame_count = 0
                self.start_time = current_time

            # **將結果放入 result_queue，同樣只保留最新一次的結果**
            if not self.result_queue.empty():
                try:
                    self.result_queue.get_nowait()
                except Empty:
                    pass
            self.result_queue.put((frame, results))

    def publish_results(self):
        # **此函式在另一個子執行緒中負責：**
        # 1. 從 result_queue 取出最新的 (frame, results)
        # 2. 發布檢測結果至 ROS2 (BoundingBox、CenterBox)
        # 3. 在畫面上繪製偵測框、FPS，推送到 FFmpeg，再發布到 ROS2 影像主題

        while rclpy.ok():
            try:
                frame, results = self.result_queue.get(timeout=1)
            except Empty:
                continue

            # **發布檢測結果 (BoundingBox、CenterBox)**
            self.publish_detections(results)

            # **如果 YOLO 有結果，用結果裡的 plot() 方法得到加上偵測框的影像；若無，直接使用原始影像**
            if results and len(results) > 0:
                annotated_frame = results[0].plot()
            else:
                annotated_frame = frame

            # **在影像左上角顯示 FPS**
            cv2.putText(
                annotated_frame, 
                f"FPS: {self.fps:.2f}", 
                (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 
                1, 
                (0, 255, 0), 
                2
            )

            # **把影像資料寫入 FFmpeg stdin，送往 RTSP 伺服器**
            try:
                self.ffmpeg_process.stdin.write(annotated_frame.tobytes())
            except (BrokenPipeError, IOError) as e:
                self.get_logger().error(f"FFmpeg 寫入錯誤: {e}, 正在重啟 FFmpeg 進程...")
                self.ffmpeg_process.stdin.close()
                self.ffmpeg_process.wait()
                width, height = annotated_frame.shape[1], annotated_frame.shape[0]
                self.ffmpeg_process = self.setup_ffmpeg_process(width, height)
        # **等待所有子執行緒結束**
    def publish_detections(self, results):
        # **解析 YOLO 偵測結果，發布 BoundingBox 與 CenterBox**
        for r in results:
            boxes = r.boxes.xyxy.cpu().numpy()  # **取得 [x_min, y_min, x_max, y_max] 格式的框座標**
            classes = r.boxes.cls.cpu().numpy() # **取得分類類別索引**
            confidences = r.boxes.conf.cpu().numpy() # **取得置信度**

            for box, cls, conf in zip(boxes, classes, confidences):
                # **BoundingBox 訊息**
                bbox_msg = BoundingBox()
                bbox_msg.xmin = int(box[0])
                bbox_msg.ymin = int(box[1])
                bbox_msg.xmax = int(box[2])
                bbox_msg.ymax = int(box[3])
                bbox_msg.label = self.model.names[int(cls)]
                bbox_msg.confidence = float(conf)
                self.box_pub.publish(bbox_msg)

                # **CenterBox 訊息，計算框的中心點**
                x_center = int((box[0] + box[2]) / 2)
                y_center = int((box[1] + box[3]) / 2)
                center_msg = CenterBox()
                center_msg.x_center = x_center
                center_msg.y_center = y_center
                self.box_center_pub.publish(center_msg)

    def destroy_node(self):
        # **節點銷毀時，關閉攝影機及 FFmpeg**
        self.cap.release()
        self.ffmpeg_process.stdin.close()
        self.ffmpeg_process.wait()
        super().destroy_node()

def main(args=None):
    # **初始化 ROS2**
    rclpy.init(args=args)
    node = YoloRtspRosNode()

    try:
        # **讓程式進入 Spin 狀態，持續運作並監聽事件**
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # **結束前銷毀節點並關閉 ROS**
        node.destroy_node()
        rclpy.shutdown()

        # **等待子執行緒結束後再真正退出**
        for thread in threading.enumerate():
            if thread is not threading.current_thread():
                thread.join()

if __name__ == "__main__":
    main()
