#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
File   : LRFX00M1LSQ_laser
Author : FantasyWilly
Email  : bc697522h04@gmail.com

雷射測距型號 : LRFX00M1LSQ
官方網站 : https://www.iadiy.com/Compact-Long-Distance-Laser-RangeFinder-Module?srsltid=AfmBOor8ERMPpWPX-U7E2Ig0S8S-6BoiHfJP5-MVnabJV1fj3RpsWT0D
檔案大綱 :
    A. 接收解碼數據 & 發布至ROS2
"""

# Python
import sys
import threading
import serial.tools.list_ports

# ROS2
import rclpy
from rclpy.node import Node

# ROS2 自定義消息包
from camera_msg_pkg.msg import LaserData, Laser

# ==========================
# 初始化 ROS2 節點
# ==========================
class LaserPublisher(Node):
    def __init__(self):
        super().__init__('laser_node')  # ROS2 節點名稱
        self.publisher_laser = self.create_publisher(Laser, 'laser_data_pub', 10)  # Topic 名稱

    def publish_laser_data(self, target_distance):
        """ 發布測距數據到 ROS2 topic """
        laser_data = LaserData()
        laser_data.targetdist = target_distance  # 設置距離數據

        laser_msg = Laser()
        laser_msg.data = [laser_data]  # 將測距數據存入陣列
        self.publisher_laser.publish(laser_msg)

        self.get_logger().info(f'Published Distance: {target_distance} m')


# ==========================
# 連接並選擇串口
# ==========================
ports = serial.tools.list_ports.comports()
portlist = []
index = 0
for port, desc, hwid in ports:
    portlist.append(port)
    print(str(index) + ") " + desc + "|" + port)
    index += 1
if portlist == []:
    print("No serial ports detected")
    sys.exit()

portname = input("Please input the number of the desired port: \n")
try:
    ser = serial.Serial(
        port=portlist[int(portname)],
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=0.5)
except Exception as e:
    print(str(e))
    sys.exit()

# ==========================
# 停止測量 (避免設備一直測量)
# ==========================
cmd = bytearray(b'\x55\xAA\x8E\xFF\xFF\xFF\xFF\x8A')
ser.write(cmd)

# ==========================
# 連續測量函式
# ==========================
globalFlag = False
threadFlag = True

def continuousMeasurement(node):
    while threadFlag:
        while globalFlag:
            ser.reset_input_buffer()
            data = ser.read(8)
            
            # 若讀取數據長度小於 7，則表示讀取失敗
            if len(data) < 7:
                print("Continuous Measurement Quit")
            
            # 若雷射測距回傳錯誤值
            elif (hex(data[5]) == '0xff') | (hex(data[4]) == '0x00'):
                print("Measurement Failed")
            
            else:
                # 解析距離數據
                target_distance = (int.from_bytes(data[5:7], "big")) / 10.0
                print(f"Distance is: {target_distance} m")
                
                # **將距離數據發送到 ROS2**
                node.publish_laser_data(target_distance)


# ==========================
# 啟動 ROS2 節點與執行緒
# ==========================
def main():
    rclpy.init()
    laser_publisher = LaserPublisher()

    # 啟動測量執行緒
    t = threading.Thread(target=continuousMeasurement, args=(laser_publisher,))
    t.start()

    # ==========================
    # 選擇功能
    # ==========================
    while rclpy.ok():
        key = input("\nPlease enter the operation command:\n -s single measurement\n -c continuous measurement\n -q Exit\n")

        # **單次測量**
        if key == 's':
            ser.reset_input_buffer()
            cmd = bytearray(b'\x55\xAA\x88\xFF\xFF\xFF\xFF\x84')
            ser.write(cmd)
            data = ser.read(8)
            
            if len(data) < 7:
                print("Measurement Failed")
            elif (hex(data[5]) == '0xff') | (hex(data[4]) == '0x00'):
                print("Measurement Failed")
            else:
                target_distance = (int.from_bytes(data[5:7], "big")) / 10.0
                print(f"Distance is: {target_distance} m")

                # **發送至 ROS2**
                laser_publisher.publish_laser_data(target_distance)

        # **連續測量**
        elif key == 'c':
            cmd = bytearray(b'\x55\xAA\x89\xFF\xFF\xFF\xFF\x85')
            ser.write(cmd)
            global globalFlag
            globalFlag = True
            input("\nPlease press any key to stop the measurement\n")
            cmd = bytearray(b'\x55\xAA\x8E\xFF\xFF\xFF\xFF\x8A')
            ser.write(cmd)
            globalFlag = False

        # **退出程式**
        elif key == 'q':
            global threadFlag
            threadFlag = False
            ser.close()
            rclpy.shutdown()
            sys.exit()

        # **無效輸入**
        else:
            print("Invalid option. Please enter one of the above options.")

if __name__ == '__main__':
    main()
