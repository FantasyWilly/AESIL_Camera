# camera_loop_command.py
import time
import threading

from camera_pkg.camera_communication import CommunicationController
from camera_pkg.camera_decoder import ReceiveMsg

gimbal_msg=ReceiveMsg()

def loop_in_background(controller: CommunicationController, stop_event: threading.Event):
    while not stop_event.is_set():
        try:

            controller.loop_send_command(b'\x4B\x4B\x01\x97')

        except Exception as e:
            print("無法送出資料", e)

        time.sleep(0.05)
