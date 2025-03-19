#!/usr/bin/env python3

'''
File   : yolov8_sv_gps_save.py
author : FantasyWilly
email  : bc697522h04@gmail.com

相機型號 : KTG-TT30
檔案大綱 :
    1. 接收 - 經緯度資訊
    2. 使用 - SuperVision
    3. 發布 - YOLO偵測結果
'''

# Python3
import cv2
import threading
import subprocess
import shlex
import time
import os
from queue import Queue, Empty

# YOLO
from ultralytics import YOLO
import supervision as sv

# ROS2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import NavSatFix
from yolo_msg_pkg.msg import BoundingBox, CenterBox

# ----------------------- [VideoCapture] 取得影像 & 抓取影像 -----------------------
class VideoCapture:

    # 定義基本參數(相機來源, Queue暫存器, Thread多線程)
    def __init__(self, camera_source):
        self.cap = cv2.VideoCapture(camera_source)                                      # 初始化攝影機連線
        self.q = Queue(maxsize=1)                                                       # 建立一個 Queue 用來暫存讀取到的影像 大小限制為 1（只存最新一筆）
        self.stop_thread = False
        self.thread = threading.Thread(target=self._reader, name="VideoCaptureThread")  # 建立並啟動一個背景執行緒，持續讀取影像
        self.thread.daemon = True
        self.thread.start()

    # 取得影像的寬度與高度
    def get_frame_size(self):
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return width, height

    # 持續讀取 鏡頭 RTSP 影像直到要求停止
    def _reader(self):
        
        while not self.stop_thread:
            ret, frame = self.cap.read()
            if not ret:
                continue
            if not self.q.empty():
                try:
                    self.q.get_nowait()
                except Empty:
                    pass
            self.q.put(frame)

    # 從 Queue 中取出最新影像
    def read(self):
        return self.q.get()

    # 停止讀取影像並釋放攝影機資源
    def release(self):
        self.stop_thread = True
        self.thread.join()
        self.cap.release()

# ----------------------- [YoloRtspRosNode] 結合 YOLO 推論, 追蹤, 影像標註 及 GPS 資訊訂閱 -----------------------
class YoloRtspRosNode(Node):
    # Node: yolov8_sv_gps_save_node
    # A.[宣告] 可動參數 
    # B.[發布] 偵測結果話題 
    # C.[推流] ffmpeg - RTSP 
    # D.[儲存] ffmpeg - MP4
    def __init__(self):
        super().__init__('yolov8_sv_gps_save_node')
        qos = QoSProfile(
            reliability=ReliabilityPolicy(0),
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_ALL,
        )

        self.declare_parameter('model_path', '/home/fantasywilly/weight/Car_Model.pt')
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('conf_thresh', 0.5)
        self.declare_parameter('camera_source', 'rtsp://user:user@192.168.144.108:554/cam/realmonitor?channel=1&subtype=0')
        self.declare_parameter('rtsp_server_url', 'rtsp://192.168.0.230:8554/live/stream')
        self.declare_parameter('frame_rate', 30)
        self.declare_parameter('gps_topic', '/mavros/global_position/global')
        self.declare_parameter('save_directory', '/home/fantasywilly/Yolov8-Video')
        self.declare_parameter('base_filename', 'Yolov8-Recording.mp4')
        
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.imgsz = self.get_parameter('imgsz').get_parameter_value().integer_value
        self.conf_thresh = self.get_parameter('conf_thresh').get_parameter_value().double_value
        self.camera_source = self.get_parameter('camera_source').get_parameter_value().string_value
        self.rtsp_server_url = self.get_parameter('rtsp_server_url').get_parameter_value().string_value
        self.frame_rate = self.get_parameter('frame_rate').get_parameter_value().integer_value
        self.gps_topic = self.get_parameter('gps_topic').get_parameter_value().string_value
        self.save_directory = self.get_parameter('save_directory').get_parameter_value().string_value
        self.base_filename = self.get_parameter('base_filename').get_parameter_value().string_value

        # FPS 計算相關變數初始化
        self.frame_count = 0
        self.start_time = time.time()
        self.fps = 0.0

        """
        可選[ROS_Bridge]: 建立 CvBridge 物件，用以轉換 OpenCV 與 ROS 影像格式 
        """
        # self.bridge = CvBridge()

        # 載入 YOLOv8 模型
        self.get_logger().info("正在加載 YOLOv8 模型...")
        self.model = YOLO(self.model_path)
        self.get_logger().info("YOLOv8 模型加載完成")

        # 初始化 Supervision 追蹤器與標註器
        self.tracker = sv.ByteTrack()
        self.box_annotator = sv.BoxAnnotator()
        self.label_annotator = sv.LabelAnnotator()

        # 建立 ROS2 Publisher，用於發布 BoundingBox 與 CenterBox 訊息
        self.box_pub = self.create_publisher(BoundingBox, "/box", 10)
        self.box_center_pub = self.create_publisher(CenterBox, "/box_center", 10)

        # 建立 ROS2 Subscriber，訂閱 MAVROS 發佈的 GPS (NavSatFix) 資訊
        self.gps_sub = self.create_subscription(
            NavSatFix,
            self.gps_topic,
            self.gps_callback,
            qos_profile=qos
        )

        # 建立一個暫存器存放最新的飛機經緯度資料，並使用鎖來確保線程安全
        self.gps_lock = threading.Lock()
        self.current_gps = None  # 初始為 None，待收到第一筆資料後更新

        # 初始化攝影機
        self.cap = VideoCapture(self.camera_source)
        width, height = self.cap.get_frame_size()
        self.get_logger().info(f"影像尺寸: {width}x{height}")

        # 建立 Queue 以傳遞影像與偵測結果
        self.frame_queue = Queue(maxsize=1)
        self.result_queue = Queue(maxsize=1)

        # 開啟子執行緒，分別執行 YOLO 推論與結果發布
        threading.Thread(target=self.yolo_predict, name="YoloPredictThread").start()
        threading.Thread(target=self.publish_results, name="PublishResultsThread").start()

        # 啟動 FFmpeg 進程，負責串流標註後的影像
        self.ffmpeg_rtsp_process = self.setup_rtsp_ffmpeg_process(width, height)
        self.ffmpeg_file_process = self.setup_file_ffmpeg_process(width, height)

    # ---------- (generate_unique_filename) 儲存 [MP4] 路徑 ----------
    def generate_unique_filename(self):

        # 組合完整路徑
        base_path = os.path.join(self.save_directory, self.base_filename)
        directory, filename = os.path.split(base_path)
        name, ext = os.path.splitext(filename)

        # 若儲存目錄不存在，則建立目錄
        if not os.path.exists(directory):
            os.makedirs(directory)
        counter = 1
        candidate = base_path

        # 檢查檔案是否存在，若存在則加上後綴
        while os.path.exists(candidate):
            candidate = os.path.join(directory, f"{name}_{counter:02d}{ext}")
            counter += 1
        return candidate

    # ---------- (gps_callback) 訂閱ROS2 GPS的相關訊息 ----------
    def gps_callback(self, msg):
        with self.gps_lock:
            self.current_gps = (msg.latitude, msg.longitude)
        self.get_logger().debug(f"收到 GPS 資料: 經度 {msg.latitude}, 緯度 {msg.longitude}")

    # ---------- (setup_rtsp_ffmpeg_process) 啟動 [RTSP] FFmpeg 進程 ----------
    def setup_rtsp_ffmpeg_process(self, width, height):
        rtsp_command = (
            'ffmpeg -y '                            # 若檔案已存在則自動覆蓋（推流情況通常不會寫入檔案，但此處保險起見）
            '-f rawvideo -pixel_format bgr24 '      # 指定輸入影像格式與色彩空間
            f'-video_size {width}x{height} '        # 指定影像尺寸
            f'-framerate {self.frame_rate} '        # 指定影像幀率
            '-i - '                                 # 從標準輸入讀取影像資料
            '-c:v libx264 '                         # 使用 x264 編碼器
            '-preset ultrafast '                    # 編碼預設值：極速
            '-tune zerolatency '                    # 降低延遲
            '-pix_fmt yuv420p '                     # 指定像素格式
            '-x264-params "bframes=0" '             # 關閉 B-frames
            '-g 60 -keyint_min 60 '                 # 設定 GOP 大小與最小間隔
            '-b:v 8M '                              # 設定視訊碼率
            '-bufsize 8M '                          # 設定緩衝區大小
            '-max_delay 0 '                         # 設定最大延遲為 0 (ms)
            '-an '                                  # 不處理音訊
            f'-f rtsp {self.rtsp_server_url}'       # 指定 RTSP 推流目的地
        )
        self.get_logger().info(f"RTSP FFmpeg command: {rtsp_command}")
        return subprocess.Popen(
            shlex.split(rtsp_command),
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

    # ----------------------- (setup_file_ffmpeg_process) 啟動 [MP4] FFmpeg 進程-----------------------
    def setup_file_ffmpeg_process(self, width, height):
        output_file = self.generate_unique_filename()
        file_command = (
            'ffmpeg -y '                            # -y 表示自動覆蓋，但這裡檔名唯一通常不會衝突
            '-f rawvideo -pixel_format bgr24 '      # 設定輸入原始影像格式
            f'-video_size {width}x{height} '        # 指定影像尺寸
            f'-framerate {self.frame_rate} '        # 指定影像幀率
            '-i - '                                 # 從標準輸入讀取影像資料
            '-c:v libx264 '                         # 使用 x264 編碼器
            '-preset ultrafast '                    # 編碼預設值：極速
            '-tune zerolatency '                    # 降低延遲
            '-pix_fmt yuv420p '                     # 指定像素格式
            '-x264-params "bframes=0" '             # 關閉 B-frames
            '-g 60 -keyint_min 60 '                 # 設定 GOP 大小與最小間隔
            '-b:v 4M '                              # 設定視訊碼率
            '-bufsize 4M '                          # 設定緩衝區大小
            '-max_delay 0 '                         # 設定最大延遲為 0
            '-an '                                  # 不處理音訊
            f'{output_file}'                        # 指定輸出檔案路徑
        )
        self.get_logger().info(f"File FFmpeg command: {file_command}")
        return subprocess.Popen(
            shlex.split(file_command),
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

    # ----------------------- (yolo_predict) 執行 [YOLO] 影像辨識-----------------------
    def yolo_predict(self):
        while rclpy.ok():
            try:
                frame = self.cap.read()
                if frame is None:
                    self.get_logger().info("Got empty frame, skipping...")
                    continue

                # 進行 YOLO 推論
                results = self.model.predict(
                    source=frame,
                    device=self.device,
                    imgsz=self.imgsz,
                    conf=self.conf_thresh
                )

                # 將 YOLO 結果轉換成 Detections 物件
                detections = sv.Detections.from_ultralytics(results[0])
                
                # 使用 ByteTrack 更新追蹤器，取得帶有 tracker_id 的偵測結果
                tracked_detections = self.tracker.update_with_detections(detections=detections)

                # 將 (frame, tracked_detections) 放入 Queue，僅保留最新一筆
                if not self.result_queue.empty():
                    self.result_queue.get_nowait()
                self.result_queue.put((frame, tracked_detections))

                # 計算並更新 FPS
                self.frame_count += 1
                current_time = time.time()
                elapsed_time = current_time - self.start_time
                if elapsed_time >= 1.0:
                    self.fps = self.frame_count / elapsed_time
                    self.get_logger().info(f"當前 FPS: {self.fps:.2f}")
                    self.frame_count = 0
                    self.start_time = current_time

            except Exception as e:
                self.get_logger().error(f"yolo_predict 發生錯誤: {e}")
                continue

    # ----------------------- (publish_results) 發布 [YOLO] 影像辨識結果-----------------------
    def publish_results(self):
        while rclpy.ok():
            try:
                frame, detections = self.result_queue.get(timeout=1)
            except Empty:
                self.get_logger().debug("publish_results: 1秒內未取得資料 continue...")
                continue

            try:
                # 先發布偵測結果到 ROS2 topic
                self.publish_detections(detections)

                # 從暫存器中讀取最新的 GPS 資料
                with self.gps_lock:
                    gps = self.current_gps if self.current_gps is not None else (0.0, 0.0)

                # 根據每個追蹤結果生成標籤字串 - 飛機的經緯度資訊
                labels = [
                    f"#{tracker_id} {self.model.names[int(cls)]} - [{gps[0]:.7f}, {gps[1]:.7f}]"
                    for cls, tracker_id in zip(detections.class_id, detections.tracker_id)
                ]

                # 利用 supervision 標註器在影像上繪製方框
                annotated_frame = self.box_annotator.annotate(
                    scene=frame.copy(),
                    detections=detections
                )
                
                # 在影像上標註剛生成的文字標籤
                annotated_frame = self.label_annotator.annotate(
                    scene=annotated_frame,
                    detections=detections,
                    labels=labels
                )

                # 在影像上顯示 FPS
                cv2.putText(
                    annotated_frame,
                    f"FPS: {self.fps:.2f}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 255, 0),
                    2
                )

                # 將標註後的影像推送到 [RTSP] 推流的 FFmpeg 進程
                try:
                    self.ffmpeg_rtsp_process.stdin.write(annotated_frame.tobytes())
                except (BrokenPipeError, IOError) as e:
                    self.get_logger().error(f"RTSP FFmpeg 寫入錯誤: {e}, 正在重啟 RTSP FFmpeg 進程...")
                    self.ffmpeg_rtsp_process.stdin.close()
                    self.ffmpeg_rtsp_process.wait()
                    self.ffmpeg_rtsp_process = self.setup_rtsp_ffmpeg_process(annotated_frame.shape[1], annotated_frame.shape[0])

                # 將標註後的影像推送到 [MP4] 存檔的 FFmpeg 進程
                try:
                    self.ffmpeg_file_process.stdin.write(annotated_frame.tobytes())
                except (BrokenPipeError, IOError) as e:
                    self.get_logger().error(f"File FFmpeg 寫入錯誤: {e}, 正在重啟 File FFmpeg 進程...")
                    self.ffmpeg_file_process.stdin.close()
                    self.ffmpeg_file_process.wait()
                    self.ffmpeg_file_process = self.setup_file_ffmpeg_process(annotated_frame.shape[1], annotated_frame.shape[0])
                
                """
                可選[ROS_Bridge]: 發布影像到其他 ROS topic
                """
                # image_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
                # self.image_pub.publish(image_msg)

            except Exception as e:
                self.get_logger().error(f"publish_results 發生錯誤: {e}")
                continue
    
    # ----------------------- (publish_detections) 發布 [YOLO] 影像辨識結果 至 ROS2-----------------------
    def publish_detections(self, detections):
        for box, cls, conf in zip(detections.xyxy, detections.class_id, detections.confidence):
            bbox_msg = BoundingBox()
            bbox_msg.xmin = int(box[0])
            bbox_msg.ymin = int(box[1])
            bbox_msg.xmax = int(box[2])
            bbox_msg.ymax = int(box[3])
            bbox_msg.label = self.model.names[int(cls)]
            bbox_msg.confidence = float(conf)
            self.box_pub.publish(bbox_msg)

            # 計算方框中心點
            x_center = int((box[0] + box[2]) / 2)
            y_center = int((box[1] + box[3]) / 2)
            center_msg = CenterBox()
            center_msg.x_center = x_center
            center_msg.y_center = y_center
            self.box_center_pub.publish(center_msg)

    # ----------------------- (destroy_node) 清理 [Node] 節點 -----------------------
    def destroy_node(self):
        self.cap.release()
        self.ffmpeg_process.stdin.close()
        self.ffmpeg_process.wait()
        super().destroy_node()

# ----------------------- [main] 主要執行序 -----------------------
def main(args=None):
    rclpy.init(args=args)
    node = YoloRtspRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        for thread in threading.enumerate():
            if thread is not threading.current_thread():
                thread.join()

if __name__ == "__main__":
    main()
