#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
File   : camera_feedback.py
author : LYX(先驅), FantasyWilly
email  : FantasyWilly - bc697522h04@gmail.com

相機型號 : KTG-TT30
檔案大綱 :
    A. 接收 - 相機＆雲台回傳
    B. 打開 - 雷射測距
    C. 持續傳送回傳指令以取得數據
'''

# Python
import time
import threading

# ROS2 引用 Python 檔 (mine)
import camera_tt30_pkg.camera_command as cm
import camera_tt30_pkg.camera_loop_command  as loop_cm
from camera_tt30_pkg.camera_communication import CommunicationController
from camera_tt30_pkg.camera_decoder import ReceiveMsg

# ----------------------- [main] 主要執行序 -----------------------
def main():

    # 創建 CommunicationController[連線] & ReceiveMsg[解析]
    controller = CommunicationController("192.168.144.200", 2000)
    gimbal_msg = ReceiveMsg()

    try:

        # Step1 - 開始連線
        controller.connect()

        # Step2 - 打開雷射測距
        cm.Command.Laser_command(controller, 1)
        print("[開啟]: 雷射測距(Wait 1 sec)")
        print("--------------------------")
        time.sleep(1)

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
        print("--------------------------")

        # Step5 - 接收相機數據 & 呼叫解碼器
        for packet in CommunicationController.recv_packets(controller.sock, packet_size=32, header=b'\x4B\x4B'):
            if gimbal_msg.parse(packet, len(packet), gimbal_msg):
                print(f"[ROLL]={gimbal_msg.rollAngle} 度, [YAW]={gimbal_msg.yawAngle} 度, [PITCH]={gimbal_msg.pitchAngle} 度")
                print(f"[TARGET-DIST]={gimbal_msg.targetDist} m")
            else:
                print("無法解析資料")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("使用者中斷程式執行")
    except Exception as e:
        print("發生例外:", e)
    finally:
        stop_event.set()
        loop_thread.join()
        controller.disconnect()

if __name__ == "__main__":
    main()
