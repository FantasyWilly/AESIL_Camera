#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
File   : camera_feedback_ros2.py
author : LYX(先驅), FantasyWilly
email  : FantasyWilly - bc697522h04@gmail.com

檔案大綱 : 
    A. 接收 - 相機＆雲台回傳
    B. 打開 - 雷射測距
    C. 持續傳送回傳指令以取得數據
    D. 發布資訊至 ROS2
'''

# Python
import time
import threading

# ROS2
import rclpy
from rclpy.node import Node

# ROS2 自定義消息包
from camera_msg_pkg.msg import Camera, CameraData
from camera_msg_pkg.msg import Laser, LaserData

# ROS2 引用 Python 檔 (mine)
import camera_tt30_pkg.camera_command as cm
import camera_tt30_pkg.camera_loop_command  as loop_cm
from camera_tt30_pkg.camera_communication import CommunicationController
from camera_tt30_pkg.camera_decoder import ReceiveMsg

# ----------------------- [CameraFeedbackPublisher] ROS2 - [Node] & [TOPIC] -----------------------
class CameraFeedbackPublisher(Node):
    # Node       : camera_feedback_publisher_node
    # Topic(PUB) : /camera_data_pub
    # Topic(PUB) : /laser_data_pub
    def __init__(self):
        super().__init__('camera_feedback_publisher_node')
        
        self.publisher_camera = self.create_publisher(Camera, '/camera_data_pub', 10)
        self.publisher_laser  = self.create_publisher(Laser, '/laser_data_pub', 10)

# ----------------------- [main] 主要執行序 -----------------------
def main():

    # 創建 CommunicationController[連線] & ReceiveMsg[解析]
    controller = CommunicationController("192.168.144.200", 2000)
    gimbal_msg = ReceiveMsg()

    # 初始化 Node 節點
    rclpy.init()
    node = CameraFeedbackPublisher()

    try:
        
        # Step1 - 開始連線
        controller.connect()

        # Step2 - 打開雷射測距
        node.get_logger().info("[開啟]: 雷射測距 (等待 1 秒)")
        cm.Command.Laser_command(controller, 1)
        time.sleep(2)
        
        # Step3 - 建立一個 stop_event 讓背景線程知道什麼時候要結束
        stop_event = threading.Event()

        # Step4 - 開啟 Loop 線程
        loop_thread = threading.Thread(
            target=loop_cm.loop_in_background,
            args=(controller, stop_event),
            daemon=True
        )
        loop_thread.start()
        print("[開啟]: LOOP-CMD")

        # Step5 - 接收相機數據 & 呼叫解碼器
        for packet in CommunicationController.recv_packets(controller.sock, packet_size=32, header=b'\x4B\x4B'):
            if stop_event.is_set():
                break
            if gimbal_msg.parse(packet, len(packet), gimbal_msg):
                camera_data = CameraData()
                camera_data.rollangle = float(gimbal_msg.rollAngle)
                camera_data.yawangle = float(gimbal_msg.yawAngle)
                camera_data.pitchangle = float(gimbal_msg.pitchAngle)

                camera_msg = Camera()
                camera_msg.data = [camera_data]
                node.publisher_camera.publish(camera_msg)
                # node.get_logger().info(
                #     f"[發布] Camera 資料: [ROLL]={camera_data.rollangle}, [YAW]={camera_data.yawangle}, [PITCH]={camera_data.pitchangle}"
                # )

                # -----------------------------------------------------------------------------------------

                target_distance = float(gimbal_msg.targetDist)
                if target_distance > 1500:
                    # print("超出範圍 (超過 1500) 的資料，忽略:", target_distance)
                    continue
                laser_data = LaserData()
                laser_data.targetdist = target_distance

                laser_msg = Laser()
                laser_msg.data = [laser_data]
                node.publisher_laser.publish(laser_msg)
                # node.get_logger().info(
                #     f"[發布] Laser 資料: [TARGET-DIST]={laser_data.targetdist}"
                # )

            else:
                print("無法解析資料")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n!!使用者中斷程式執行!!")
    except Exception as e:
        print("發生例外:", e)
    finally:
        stop_event.set()
        loop_thread.join()
        controller.disconnect()

if __name__ == "__main__":
    main()
