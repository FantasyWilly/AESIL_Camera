#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
File   : camera_feedback_ros2.py
author : LYX(先驅), FantasyWilly
email  : FantasyWilly - bc697522h04@gmail.com

說明  : 接收雲台資訊並發布至 ROS2
         透過 communication 模組處理相機資料解析與封包接收
'''

import time
import threading

import rclpy
from rclpy.node import Node
from camera_msg_pkg.msg import Camera, CameraData

import camera_pkg.camera_command as cm
import camera_pkg.camera_loop_command  as loop_cm
from camera_pkg.camera_communication import CommunicationController
from camera_pkg.camera_decoder import ReceiveMsg

class CameraFeedbackPublisher(Node):
    def __init__(self):
        super().__init__('camera_feedback_publisher_node')
        self.publisher_ = self.create_publisher(Camera, '/camera_data_pub', 10)

def main():

    # 創建 CommunicationController[連線] & ReceiveMsg[解析]
    controller = CommunicationController("192.168.144.200", 2000)
    gimbal_msg = ReceiveMsg()

    # 初始化 Node 節點
    rclpy.init()
    node = CameraFeedbackPublisher()

    try:
        
        # (1) 開始連線
        controller.connect()

        # (2) 打開雷射測距
        node.get_logger().info("[開啟]: 雷射測距 (等待 1 秒)")
        cm.Command.Laser_command(controller, 1)
        time.sleep(2)
        
        # (3) 建立一個 stop_event 讓背景線程知道什麼時候要結束
        stop_event = threading.Event()

        # (4) 開啟 Loop 線程
        loop_thread = threading.Thread(
            target=loop_cm.loop_in_background,
            args=(controller, stop_event),
            daemon=True
        )
        loop_thread.start()
        print("[開啟]: LOOP-CMD")

        # (5) 接收相機數據 & 呼叫解碼器
        for packet in CommunicationController.recv_packets(controller.sock, packet_size=32, header=b'\x4B\x4B'):
            if gimbal_msg.parse(packet, len(packet), gimbal_msg):
                data_msg = CameraData()
                data_msg.rollangle = float(gimbal_msg.rollAngle)
                data_msg.yawangle = float(gimbal_msg.yawAngle)
                data_msg.pitchangle = float(gimbal_msg.pitchAngle)
                data_msg.targetdist = float(gimbal_msg.targetDist)

                camera_msg = Camera()
                camera_msg.data = [data_msg]

                node.publisher_.publish(camera_msg)
                node.get_logger().info(
                    f"[發布] Camera 資料: [ROLL]={data_msg.rollangle}, [YAW]={data_msg.yawangle}, [PITCH]={data_msg.pitchangle}"
                    f"[TARGET-DIST]={data_msg.targetdist}"
                )
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
