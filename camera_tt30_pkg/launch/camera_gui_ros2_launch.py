# camera_gui_ros2_launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 取得 camera_tt30_pkg 底下的 config/ 參數檔路徑
    config_file_path = os.path.join(
        get_package_share_directory('camera_tt30_pkg'),                     # 請改成你的 package 名稱
        'config',
        'camera_gui_ros2.yaml'                                         # 與上面建立的 YAML 檔名一致
    )

    return LaunchDescription([
        Node(
            package='camera_tt30_pkg',                                      # package 名稱
            executable='camera_feedback_publisher_gui_node',           # 在 setup.py 裡註冊的執行檔或 entry point 名稱
            name='camera_feedback_publisher_gui_node',                 # 對節點重新命名
            output='screen',
            parameters=[config_file_path],                             # 指定要讀取的參數檔
        )
    ])
