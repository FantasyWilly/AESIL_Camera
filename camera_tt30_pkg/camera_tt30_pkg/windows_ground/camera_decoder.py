#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
File   : camera_decoder.py
author : FantasyWilly
email  : bc697522h04@gmail.com

檔案大綱 : 
    A. 解析回傳資料
"""

# Python
import numpy as np

class ReceiveMsg:
    """
    處理相機回傳資料的解析
    """
    def __init__(self):
        # 初始化所有解析後的數值變數
        self.zAngle = 0.0         # Z 軸轉動角
        self.pitchAngle = 0.0     # 俯仰角
        self.rollAngle = 0.0      # 滾動角
        self.yawAngle = 0.0       # 航向角
        self.targetDist = 0.0     # 目標距離（公尺）
        self.targetAtt = 0.0      # 目標相對高度
        self.targetLng = 0.0      # 目標經度
        self.targetLat = 0.0      # 目標緯度
        self.eoZoom = 0.0         # 可見光放大倍數

    def parse(self, buffer, length, out):
        """
        解析相機回傳的原始資料:
            :param [buffer]: 原始位元組資料
            :param [length]: 資料長度
            :param [out]   : 儲存解析結果的 ReceiveMsg 物件
        """
        if length < 30 or not buffer:
            return False

        try:
            # 解析雲台角度數據
            out.zAngle = np.frombuffer(buffer[0:2], dtype=np.float16)[0]
            out.pitchAngle = np.frombuffer(buffer[5:7], dtype=np.dtype('<i2'))[0] / 100
            out.rollAngle = np.frombuffer(buffer[7:9], dtype=np.dtype('<i2'))[0] / 100
            out.yawAngle = np.frombuffer(buffer[9:11], dtype=np.dtype('<i2'))[0] / 100

            # 解析目標測距與經緯度數據
            out.targetDist = np.frombuffer(buffer[12:14], dtype=np.dtype('<u2'))[0] / 10
            out.targetLat = np.frombuffer(buffer[16:20], dtype=np.dtype('<i4'))[0] / 10000000
            out.targetLng = np.frombuffer(buffer[20:24], dtype=np.dtype('<i4'))[0] / 10000000

            return True
        except Exception as e:
            print("解析資料失敗:", e)
            return False

