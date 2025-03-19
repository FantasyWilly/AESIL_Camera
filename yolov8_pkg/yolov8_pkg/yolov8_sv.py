#!/usr/bin/env python3

import cv2
import threading
import subprocess
import shlex
import time
from queue import Queue, Empty

# Yolo
from ultralytics import YOLO
import supervision as sv

# ROS2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from yolo_msg_pkg.msg import BoundingBox, CenterBox

class VideoCapture:
    def __init__(self, camera_source):
        self.cap = cv2.VideoCapture(camera_source)
        self.q = Queue(maxsize=1)
        self.stop_thread = False
        self.thread = threading.Thread(target=self._reader, name="VideoCaptureThread")
        self.thread.daemon = True
        self.thread.start()

    def get_frame_size(self):
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return width, height

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

    def read(self):
        return self.q.get()

    def release(self):
        self.stop_thread = True
        self.thread.join()
        self.cap.release()

class YoloRtspRosNode(Node):
    def __init__(self):
        super().__init__('yolov8_sv_node')

        # 宣告並讀取參數
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

        # FPS
        self.frame_count = 0
        self.start_time = time.time()
        self.fps = 0.0

        # CvBridge
        self.bridge = CvBridge()

        # 載入 YOLOv8 模型
        self.get_logger().info("正在加載 YOLOv8 模型...")
        self.model = YOLO(self.model_path)
        self.get_logger().info("YOLOv8 模型加載完成")

        # 初始化 supervision 追蹤器
        self.tracker = sv.ByteTrack()
        # 初始化標註器
        self.box_annotator = sv.BoxAnnotator()
        self.label_annotator = sv.LabelAnnotator()

        # ROS2 Publisher
        self.box_pub = self.create_publisher(BoundingBox, "/box", 10)
        self.box_center_pub = self.create_publisher(CenterBox, "/box_center", 10)

        # 初始化攝影機
        self.cap = VideoCapture(self.camera_source)
        width, height = self.cap.get_frame_size()
        self.get_logger().info(f"影像尺寸: {width}x{height}")

        # Queue
        self.frame_queue = Queue(maxsize=1)
        self.result_queue = Queue(maxsize=1)

        # 開子執行緒
        threading.Thread(target=self.yolo_predict, name="YoloPredictThread").start()
        threading.Thread(target=self.publish_results, name="PublishResultsThread").start()

        # 啟動 FFmpeg
        self.ffmpeg_process = self.setup_ffmpeg_process(width, height)

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

    def yolo_predict(self):
        """不斷從攝影機讀取最新影像，執行 YOLO + ByteTrack 將帶有 tracker_id 的結果放入 result_queue。"""
        while rclpy.ok():
            try:
                frame = self.cap.read()
                if frame is None:
                    self.get_logger().info("Got empty frame, skipping...")
                    continue

                # 進行推論
                results = self.model.predict(
                    source=frame,
                    device=self.device,
                    imgsz=self.imgsz,
                    conf=self.conf_thresh
                )

                # 將 YOLO 結果轉成 Detections
                detections = sv.Detections.from_ultralytics(results[0])
                
                # 更新追蹤器，**回傳**帶有 tracker_id 的新的 Detections
                tracked_detections = self.tracker.update_with_detections(detections=detections)

                # 把(影像、帶 tracker_id 的偵測結果)放到 Queue
                if not self.result_queue.empty():
                    self.result_queue.get_nowait()
                self.result_queue.put((frame, tracked_detections))

                # 計算並印出 FPS
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

    def publish_results(self):
        """從 result_queue 取出 (frame, tracked_detections)，標註並推送到 FFmpeg / ROS topic。"""
        while rclpy.ok():
            try:
                frame, detections = self.result_queue.get(timeout=1)
            except Empty:
                self.get_logger().debug("publish_results: queue 1秒內沒拿到資料 continue...")
                continue

            try:
                # 先發布檢測結果
                self.publish_detections(detections)

                # 繪製標註
                labels = [
                    f"#{tracker_id} {self.model.names[int(cls)]} - Hello "
                    for cls, tracker_id
                    in zip(detections.class_id, detections.tracker_id)
                ]

                # 繪製方框
                annotated_frame = self.box_annotator.annotate(
                    scene=frame.copy(),
                    detections=detections
                )
                
                # 繪製 label
                annotated_frame = self.label_annotator.annotate(
                    scene=annotated_frame,
                    detections=detections,
                    labels=labels
                )

                # 顯示 FPS
                cv2.putText(
                    annotated_frame,
                    f"FPS: {self.fps:.2f}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 255, 0),
                    2
                )

                # 推送到 FFmpeg
                try:
                    self.ffmpeg_process.stdin.write(annotated_frame.tobytes())
                except (BrokenPipeError, IOError) as e:
                    self.get_logger().error(f"FFmpeg 寫入錯誤: {e}, 正在重啟 FFmpeg 進程...")
                    self.ffmpeg_process.stdin.close()
                    self.ffmpeg_process.wait()
                    width, height = annotated_frame.shape[1], annotated_frame.shape[0]
                    self.ffmpeg_process = self.setup_ffmpeg_process(width, height)

                # 發布到 /camera/image
                #image_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
                #self.image_pub.publish(image_msg)

            except Exception as e:
                self.get_logger().error(f"publish_results 發生錯誤: {e}")
                continue

    def publish_detections(self, detections):
        """解析帶 tracker_id 的 Detections 發布 BoundingBox 與 CenterBox。"""
        for box, cls, conf in zip(detections.xyxy, detections.class_id, detections.confidence):
            bbox_msg = BoundingBox()
            bbox_msg.xmin = int(box[0])
            bbox_msg.ymin = int(box[1])
            bbox_msg.xmax = int(box[2])
            bbox_msg.ymax = int(box[3])
            bbox_msg.label = self.model.names[int(cls)]
            bbox_msg.confidence = float(conf)
            self.box_pub.publish(bbox_msg)

            # 計算中心點
            x_center = int((box[0] + box[2]) / 2)
            y_center = int((box[1] + box[3]) / 2)
            center_msg = CenterBox()
            center_msg.x_center = x_center
            center_msg.y_center = y_center
            self.box_center_pub.publish(center_msg)

    def destroy_node(self):
        self.cap.release()
        self.ffmpeg_process.stdin.close()
        self.ffmpeg_process.wait()
        super().destroy_node()


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
