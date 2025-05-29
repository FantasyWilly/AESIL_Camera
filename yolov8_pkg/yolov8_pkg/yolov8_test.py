#!/usr/bin/env python3

import cv2
import threading
import time
from queue import Queue, Empty
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from yolo_msg_pkg.msg import BoundingBox, CenterBox

class VideoCapture:
    def __init__(self, camera_source):
        # **初始化 OpenCV 的 VideoCapture 物件，用來讀取指定的攝影機或串流來源**
        self.cap = cv2.VideoCapture(camera_source)

        # **建立一個大小為 1 的佇列 (Queue)，只保留最新的一幀 (frame)**
        self.q = Queue(maxsize=1)
        
        # **用於控制讀取執行緒是否停止**
        self.stop_thread = False
        
        # **建立並啟動攝影機讀取的子執行緒**
        self.thread = threading.Thread(target=self._reader, name="VideoCaptureThread")
        self.thread.daemon = True  # **守護執行緒，主程式結束時會自動中止**
        self.thread.start()

    def get_frame_size(self):
        # **取得當前攝影機畫面的寬度與高度，並回傳 (width, height)**
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return width, height

    def _reader(self):
        # **此方法在子執行緒中不斷地抓取最新影格並放進 Queue 裡**
        while not self.stop_thread:
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            # **如果佇列裡已有舊影像，就把它丟棄，保留最新影格**
            if not self.q.empty():
                try:
                    self.q.get_nowait()
                except Empty:
                    pass
            
            self.q.put(frame)

    def read(self):
        # **從佇列裡拿取最新的一幀影像**
        return self.q.get()

    def release(self):
        # **中止執行緒並釋放攝影機資源**
        self.stop_thread = True
        self.thread.join()
        self.cap.release()


class YoloRtspRosNode(Node):
    def __init__(self):
        # **初始化 ROS2 Node，節點名稱為 'yolo_rtsp_ros_node'**
        super().__init__('yolo_rtsp_ros_node')

        # ================================================================
        # **可修改參數區：請根據你的需求，在這裡集中管理各種易調整的參數**
        self.model_path = "/home/fantasywilly/weight/Car_Model.pt"  # **YOLOv8 權重檔路徑**
        self.device = "cuda:0"                                      # **推論所使用的裝置，如 'cuda:0' 或 'cpu'**
        self.imgsz = 640                                            # **推論時的輸入影像大小 (imgsz x imgsz)**
        self.conf_thresh = 0.5                                      # **推論時的置信度閾值 (confidence threshold)**
        
        self.camera_source = "rtsp://user:user@192.168.144.108:554/cam/realmonitor?channel=1&subtype=0"           # **攝影機或串流來源**
        
        # ================================================================

        # **初始化 FPS 相關變數 (用於計算即時推論畫面率)**
        self.frame_count = 0
        self.start_time = time.time()
        self.fps = 0.0

        # **初始化 CvBridge，用於 ROS 與 OpenCV 影像的互轉**
        self.bridge = CvBridge()

        # **開始載入 YOLOv8 模型**
        self.get_logger().info("正在加載 YOLOv8 模型...")
        self.model = YOLO(self.model_path)
        self.get_logger().info("YOLOv8 模型加載完成")

        # **初始化 ROS2 的 Publisher (發布者)**
        self.box_pub = self.create_publisher(BoundingBox, "/box", 10)
        self.box_center_pub = self.create_publisher(CenterBox, "/box_center", 10)
        self.image_pub = self.create_publisher(Image, "/camera/image", 10)

        # **初始化攝影機讀取物件，並獲取影像尺寸**
        self.cap = VideoCapture(self.camera_source)
        width, height = self.cap.get_frame_size()
        self.get_logger().info(f"影像尺寸: {width}x{height}")

        # **用來暫存影格、結果的佇列 (Queue)**
        self.frame_queue = Queue(maxsize=1)
        self.result_queue = Queue(maxsize=1)

        # **開啟兩個子執行緒：yolo_predict() 與 publish_results()**
        threading.Thread(target=self.yolo_predict, name="YoloPredictThread").start()
        threading.Thread(target=self.publish_results, name="PublishResultsThread").start()

    def yolo_predict(self):
        # **持續從攝影機獲取影格，進行 YOLO 推論並更新結果到 result_queue**
        while rclpy.ok():
            frame = self.cap.read()
            if frame is None:
                continue

            # **確保 frame_queue 只保留最新影格**
            if not self.frame_queue.empty():
                try:
                    self.frame_queue.get_nowait()
                except Empty:
                    pass
            self.frame_queue.put(frame)

            # **執行 YOLO 推論**
            results = self.model.predict(
                source=frame,
                device=self.device,
                imgsz=self.imgsz,
                conf=self.conf_thresh
            )

            # **更新影格計數並計算 FPS**
            self.frame_count += 1
            current_time = time.time()
            elapsed_time = current_time - self.start_time
            if elapsed_time >= 1.0:
                self.fps = self.frame_count / elapsed_time
                self.get_logger().info(f"當前 FPS: {self.fps:.2f}")
                self.frame_count = 0
                self.start_time = current_time

            # **將最新結果放進 result_queue，也只保留最新一次結果**
            if not self.result_queue.empty():
                try:
                    self.result_queue.get_nowait()
                except Empty:
                    pass
            self.result_queue.put((frame, results))

    def publish_results(self):
        # **負責從 result_queue 中取出 (frame, results)，發布偵測框、影像等資訊到 ROS2**
        while rclpy.ok():
            try:
                frame, results = self.result_queue.get(timeout=1)
            except Empty:
                continue

            # **先發布偵測結果 (BoundingBox、CenterBox)**
            self.publish_detections(results)

            # **如果有偵測到結果，就用 YOLOv8 的 plot() 進行繪製；否則直接用原圖**
            if results and len(results) > 0:
                annotated_frame = results[0].plot()
            else:
                annotated_frame = frame

            # **在影像上顯示當前 FPS**
            cv2.putText(
                annotated_frame,
                f"FPS: {self.fps:.2f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2
            )

            # **轉換影像為 ROS 影像訊息，發布到 /camera/image**
            image_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            self.image_pub.publish(image_msg)

    def publish_detections(self, results):
        # **將 YOLO 偵測結果解析並發布到 /box, /box_center**
        for r in results:
            boxes = r.boxes.xyxy.cpu().numpy()   # **取得 [x_min, y_min, x_max, y_max] 座標**
            classes = r.boxes.cls.cpu().numpy()  # **取得對應的類別索引**
            confidences = r.boxes.conf.cpu().numpy()  # **取得置信度**

            for box, cls, conf in zip(boxes, classes, confidences):
                # **填入 BoundingBox 訊息**
                bbox_msg = BoundingBox()
                bbox_msg.xmin = int(box[0])
                bbox_msg.ymin = int(box[1])
                bbox_msg.xmax = int(box[2])
                bbox_msg.ymax = int(box[3])
                bbox_msg.label = self.model.names[int(cls)]
                bbox_msg.confidence = float(conf)
                self.box_pub.publish(bbox_msg)

                # **計算中心點並發布 CenterBox 訊息**
                x_center = int((box[0] + box[2]) / 2)
                y_center = int((box[1] + box[3]) / 2)
                center_msg = CenterBox()
                center_msg.x_center = x_center
                center_msg.y_center = y_center
                self.box_center_pub.publish(center_msg)

    def destroy_node(self):
        # **節點銷毀時，關閉攝影機等資源**
        self.cap.release()
        super().destroy_node()


def main(args=None):
    # **初始化 ROS2**
    rclpy.init(args=args)
    node = YoloRtspRosNode()

    try:
        # **讓程式保持運行，等待並處理 ROS 事件**
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # **銷毀節點並關閉 ROS2**
        node.destroy_node()
        rclpy.shutdown()

        # **等待所有子執行緒結束後，程式才完全退出**
        for thread in threading.enumerate():
            if thread is not threading.current_thread():
                thread.join()


if __name__ == "__main__":
    main()
