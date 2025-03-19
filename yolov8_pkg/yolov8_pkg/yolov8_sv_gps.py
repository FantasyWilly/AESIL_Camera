#!/usr/bin/env python3

# Python3 套件
import cv2
import threading
import subprocess
import shlex
import time
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

# -----------------------------------------
# 定義 VideoCapture 類別，負責從指定來源取得影像
# -----------------------------------------
class VideoCapture:
    def __init__(self, camera_source):
        # **初始化攝影機連線**
        self.cap = cv2.VideoCapture(camera_source)
        # **建立一個 Queue 用來暫存讀取到的影像，大小限制為 1（只存最新一筆）**
        self.q = Queue(maxsize=1)
        self.stop_thread = False
        # **建立並啟動一個背景執行緒，持續讀取影像**
        self.thread = threading.Thread(target=self._reader, name="VideoCaptureThread")
        self.thread.daemon = True
        self.thread.start()

    def get_frame_size(self):
        # **取得影像的寬度與高度**
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return width, height

    def _reader(self):
        # **持續讀取影像直到要求停止**
        while not self.stop_thread:
            ret, frame = self.cap.read()
            if not ret:
                continue
            # **若 Queue 中已有資料，則移除，保證 Queue 中只保留最新一筆影像**
            if not self.q.empty():
                try:
                    self.q.get_nowait()
                except Empty:
                    pass
            self.q.put(frame)

    def read(self):
        # **從 Queue 中取出最新影像**
        return self.q.get()

    def release(self):
        # **停止讀取影像並釋放攝影機資源**
        self.stop_thread = True
        self.thread.join()
        self.cap.release()

# -----------------------------------------
# 定義 YoloRtspRosNode 類別，結合 YOLO 推論、追蹤、影像標註及 GPS 資訊訂閱
# -----------------------------------------
class YoloRtspRosNode(Node):
    def __init__(self):
        super().__init__('yolov8_sv_gps_node')
        # **QoS**
        qos=QoSProfile(
			reliability=ReliabilityPolicy(0),
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_ALL,
        )

        # **宣告並讀取參數**
        self.declare_parameter('model_path', '/home/fantasywilly/weight/Car_Model.pt')
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('conf_thresh', 0.5)
        self.declare_parameter('camera_source', 'rtsp://user:user@192.168.144.108:554/cam/realmonitor?channel=1&subtype=0')
        self.declare_parameter('rtsp_server_url', 'rtsp://192.168.0.230:8554/live/stream')
        self.declare_parameter('frame_rate', 30)

        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.imgsz = self.get_parameter('imgsz').get_parameter_value().integer_value
        self.conf_thresh = self.get_parameter('conf_thresh').get_parameter_value().double_value
        self.camera_source = self.get_parameter('camera_source').get_parameter_value().string_value
        self.rtsp_server_url = self.get_parameter('rtsp_server_url').get_parameter_value().string_value
        self.frame_rate = self.get_parameter('frame_rate').get_parameter_value().integer_value

        # **FPS 計算相關變數初始化**
        self.frame_count = 0
        self.start_time = time.time()
        self.fps = 0.0

        # **建立 CvBridge 物件，用以轉換 OpenCV 與 ROS 影像格式**
        self.bridge = CvBridge()

        # **載入 YOLOv8 模型**
        self.get_logger().info("正在加載 YOLOv8 模型...")
        self.model = YOLO(self.model_path)
        self.get_logger().info("YOLOv8 模型加載完成")

        # **初始化 supervision 追蹤器與標註器**
        self.tracker = sv.ByteTrack()
        self.box_annotator = sv.BoxAnnotator()
        self.label_annotator = sv.LabelAnnotator()

        # **建立 ROS2 Publisher，用於發布 BoundingBox 與 CenterBox 訊息**
        self.box_pub = self.create_publisher(BoundingBox, "/box", 10)
        self.box_center_pub = self.create_publisher(CenterBox, "/box_center", 10)

        # **建立 ROS2 Subscriber，訂閱 MAVROS 發佈的 GPS (NavSatFix) 資訊**
        # **一般而言，飛機經緯度資訊會從 "/mavros/global_position/global" 主題發佈**
        self.gps_sub = self.create_subscription(
            NavSatFix,
            "/target_position",
            self.gps_callback,
            qos_profile=qos
        )
        # **建立一個暫存器存放最新的飛機經緯度資料，並使用鎖來確保線程安全**
        self.gps_lock = threading.Lock()
        self.current_gps = None  # **初始為 None，待收到第一筆資料後更新**

        # **初始化攝影機**
        self.cap = VideoCapture(self.camera_source)
        width, height = self.cap.get_frame_size()
        self.get_logger().info(f"影像尺寸: {width}x{height}")

        # **建立 Queue 以傳遞影像與偵測結果**
        self.frame_queue = Queue(maxsize=1)
        self.result_queue = Queue(maxsize=1)

        # **開啟子執行緒，分別執行 YOLO 推論與結果發布**
        threading.Thread(target=self.yolo_predict, name="YoloPredictThread").start()
        threading.Thread(target=self.publish_results, name="PublishResultsThread").start()

        # **啟動 FFmpeg 進程，負責串流標註後的影像**
        self.ffmpeg_process = self.setup_ffmpeg_process(width, height)

    # -----------------------------------------
    # GPS 資訊訂閱回調函數
    # -----------------------------------------
    def gps_callback(self, msg):
        """**接收 GPS 訊息並更新暫存器中的飛機經緯度資料**"""
        with self.gps_lock:
            # **若有新資料，則直接覆蓋原先的暫存器內容**
            self.current_gps = (msg.latitude, msg.longitude)
        self.get_logger().debug(f"收到 GPS 資料: 經度 {msg.latitude}, 緯度 {msg.longitude}")

    # -----------------------------------------
    # 設定並啟動 FFmpeg 進程
    # -----------------------------------------
    def setup_ffmpeg_process(self, width, height):
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
            '-g 60 -keyint_min 60 '
            '-b:v 4M '
            '-bufsize 4M '
            '-max_delay 0 '
            '-an '
            f'-f rtsp {self.rtsp_server_url}'
        )
        self.get_logger().info(f"FFmpeg command: {ffmpeg_command}")

        return subprocess.Popen(
            shlex.split(ffmpeg_command),
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

    # -----------------------------------------
    # YOLO 推論與追蹤處理
    # -----------------------------------------
    def yolo_predict(self):
        """**不斷從攝影機讀取最新影像，執行 YOLO + ByteTrack 推論，並將結果放入 Queue**"""
        while rclpy.ok():
            try:
                frame = self.cap.read()
                if frame is None:
                    self.get_logger().info("Got empty frame, skipping...")
                    continue

                # **進行 YOLO 推論**
                results = self.model.predict(
                    source=frame,
                    device=self.device,
                    imgsz=self.imgsz,
                    conf=self.conf_thresh
                )

                # **將 YOLO 結果轉換成 Detections 物件**
                detections = sv.Detections.from_ultralytics(results[0])
                
                # **使用 ByteTrack 更新追蹤器，取得帶有 tracker_id 的偵測結果**
                tracked_detections = self.tracker.update_with_detections(detections=detections)

                # **將 (frame, tracked_detections) 放入 Queue，僅保留最新一筆**
                if not self.result_queue.empty():
                    self.result_queue.get_nowait()
                self.result_queue.put((frame, tracked_detections))

                # **計算並更新 FPS**
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

    # -----------------------------------------
    # 發布推論結果與標註影像
    # -----------------------------------------
    def publish_results(self):
        """**從 result_queue 取出影像與偵測結果，標註並推送到 FFmpeg / ROS topic**"""
        while rclpy.ok():
            try:
                frame, detections = self.result_queue.get(timeout=1)
            except Empty:
                self.get_logger().debug("publish_results: 1秒內未取得資料，continue...")
                continue

            try:
                # **先發布偵測結果到 ROS topic**
                self.publish_detections(detections)

                # **從暫存器中讀取最新的 GPS 資料**
                with self.gps_lock:
                    gps = self.current_gps if self.current_gps is not None else (0.0, 0.0)

                # **根據每個追蹤結果生成標籤字串 - 飛機的經緯度資訊**
                labels = [
                    f"#{tracker_id} {self.model.names[int(cls)]} - [{gps[0]:.7f}, {gps[1]:.7f}]"
                    for cls, tracker_id in zip(detections.class_id, detections.tracker_id)
                ]

                # **利用 supervision 標註器在影像上繪製方框**
                annotated_frame = self.box_annotator.annotate(
                    scene=frame.copy(),
                    detections=detections
                )
                
                # **在影像上標註剛生成的文字標籤**
                annotated_frame = self.label_annotator.annotate(
                    scene=annotated_frame,
                    detections=detections,
                    labels=labels
                )

                # **在影像上顯示 FPS**
                cv2.putText(
                    annotated_frame,
                    f"FPS: {self.fps:.2f}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 255, 0),
                    2
                )

                # **推送標註後的影像到 FFmpeg 進程**
                try:
                    self.ffmpeg_process.stdin.write(annotated_frame.tobytes())
                except (BrokenPipeError, IOError) as e:
                    self.get_logger().error(f"FFmpeg 寫入錯誤: {e}, 正在重啟 FFmpeg 進程...")
                    self.ffmpeg_process.stdin.close()
                    self.ffmpeg_process.wait()
                    width, height = annotated_frame.shape[1], annotated_frame.shape[0]
                    self.ffmpeg_process = self.setup_ffmpeg_process(width, height)

                # **可選：發布影像到其他 ROS topic**
                # image_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
                # self.image_pub.publish(image_msg)

            except Exception as e:
                self.get_logger().error(f"publish_results 發生錯誤: {e}")
                continue

    # -----------------------------------------
    # 發布偵測框與中心點訊息
    # -----------------------------------------
    def publish_detections(self, detections):
        """**解析帶 tracker_id 的 Detections，發布 BoundingBox 與 CenterBox 訊息**"""
        for box, cls, conf in zip(detections.xyxy, detections.class_id, detections.confidence):
            bbox_msg = BoundingBox()
            bbox_msg.xmin = int(box[0])
            bbox_msg.ymin = int(box[1])
            bbox_msg.xmax = int(box[2])
            bbox_msg.ymax = int(box[3])
            bbox_msg.label = self.model.names[int(cls)]
            bbox_msg.confidence = float(conf)
            self.box_pub.publish(bbox_msg)

            # **計算方框中心點**
            x_center = int((box[0] + box[2]) / 2)
            y_center = int((box[1] + box[3]) / 2)
            center_msg = CenterBox()
            center_msg.x_center = x_center
            center_msg.y_center = y_center
            self.box_center_pub.publish(center_msg)

    # -----------------------------------------
    # 清理與關閉資源
    # -----------------------------------------
    def destroy_node(self):
        self.cap.release()
        self.ffmpeg_process.stdin.close()
        self.ffmpeg_process.wait()
        super().destroy_node()

# -----------------------------------------
# 主函數：初始化 ROS 節點並啟動執行
# -----------------------------------------
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
        # **等待所有子執行緒結束**
        for thread in threading.enumerate():
            if thread is not threading.current_thread():
                thread.join()

if __name__ == "__main__":
    main()
