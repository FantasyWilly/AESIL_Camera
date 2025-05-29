# <div align="center">AESIL - Camera</div>

## <div align="center">Outline</div>

- [程式下載 (Downloads)](#downloads)
- [相機控制 (Camera Controll)](#camera)
- [影像串流 (Live Stream)](#live-stream)
- [影像辨識 (AI Detectition)](#yolo)
- [目標位置 (Target Position)](#target-position)

## <div align="center">Downloads</div>

***Step1* - 下載至 ROS2 的工作空間**
```bash
cd ~/<your_workspace>/src
git clone https://github.com/FantasyWilly/AESIL_Camera.git
```
---

***Step2* - 編譯工作空間**
```bash
colcon build
source ~/.bashrc
```
---

## <div align="center">Camera</div>

  ### [ D-80 系列 ] - (Pro)

  - **ROS2 Running**

    ```bash
    ros2 run camera_d80_pkg gcu_ro2_main_node
    ```

    `Node:` gcu_ros2_main_node
  
    `Topic:` camera_data_pub
    
    ---

  ### [ KTG 系列 ] - (TT30)

  > **★ 三種方式開啟程式 ( Optional )**

  - **ROS2 Running - (Optional)**

    ```bash
    ros2 run camera_tt30_pkg camera_gui_ros2_node
    ```

    `Node:` gcu_ros2_main_node

    `Topic:` /camera_data_pub, /laser_data_pub

    ---

  - **ROS2 Launch - (Optional)**

    ```bash
    ros2 launch camera_tt30_pkg xbox_air_launch.py
    ```

    - **Config**

      ```yaml
      gimbal_step:             # 雲台移動度數 (gimbal_step/10)
      zoom_duration:           # 持續放大縮小時間 (s)
      photo_continous_count:   # 連續拍照次數
      ```
    
    ---

  - **Linux - [ 天空端 ] ROS2 Xbox - (Optional)**

    
    ```bash
    ros2 run camera_tt30_pkg xbox_air_node
    ```

  - **Linux - [ 地面端 ] Xbox Control**

    ⚠️ 記得修改檔案的 `Server` IP, Port 且 在同網域底下

    ```bash
    python3 camera_tt30_pkg/camera_tt30_pkg/linux_ground/linux_xbox_ground.py
    ```

  - **Windows - [ 地面端 ] Xbox Control**

    ⚠️ 記得修改檔案的 `Server` IP, Port 且 在同網域底下

    ```bash
    python3 camera_tt30_pkg/camera_tt30_pkg/windows_ground/windows_xbox_ground.py
    ```

    ---

## <div align="center">Live-Stream</div>

  ### [ 影像串流 ] - Mediamtx

  `官網連結:` **[ Mediamtx 官方 Github  ](https://github.com/bluenviron/mediamtx)**  
  `下載連結:` **[ Mediamtx 官方 Release ](https://github.com/bluenviron/mediamtx/releases)**

  - **Linux**

    ```bash
    cd <your_mediamtx_dir>
    ./mediamtx
    ```

  - **Windows**

    ```bash
    直接點 .exe 執行檔即可
    ```

    ---

## <div align="center">YOLO</div>

  ### [ 影像辨識 ] - YOLOv8

  > **★ 二種方式開啟程式 ( Optional )**

  - **ROS2 YOLO**

    ```bash
    ros2 launch yolov8_pkg yolov8_sv_gps_launch.py
    ```

  - **Config**

    ```yaml
    model_path:         # 權重檔路徑 
    device:             # 使用 CPU, GPU(CUDA) 推理    (預設為 cuda:0)
    imgsz:              # 推理 imgsz 大小             (預設為 640)
    conf_thresh:        # 信心度閥值
    camera_source:      # rtsp 原影像地址
    rtsp_server_url:    # 伺服器傳輸 rtsp 處理後影像地址
    frame_rate:         # 伺服器傳輸 fps 幀數          (預設為 25)
    ```

    `Node:`  yolov8_sv_gps_node
  
    `Topic:` /box, /box_center 

    ---

  - **ROS2 YOLO + 儲存影片**

    ```bash
    ros2 launch yolov8_pkg yolov8_sv_gps_save_launch.py
    ```

  - **Config**

    ```yaml
    model_path:         # 權重檔路徑 
    device:             # 使用 CPU, GPU(CUDA) 推理    (預設為 cuda:0)
    imgsz:              # 推理 imgsz 大小             (預設為 640)
    conf_thresh:        # 信心度閥值
    camera_source:      # rtsp 原影像地址
    rtsp_server_url:    # 伺服器傳輸 rtsp 處理後影像地址
    frame_rate:         # 伺服器傳輸 fps 幀數          (預設為 25)
    gps_topic:          # 接收 目標物經緯度 的 GPS 話題
    save_directory:     # 儲存 影片路徑
    base_filename:      # 影片 命名規則
    ```

    `Node:`  yolov8_sv_gps_node
  
    `Topic:` /box, /box_center 

    ---

## <div align="center">TARGET-POSITION</div>

  ### [ 目標位置 ]

  > **★ 兩種模式 ( Optional )**

  - **GPS - 模式**

    ```bash
    ros2 run target_position_pkg laser_target_gps_node
    ```

    `Node:`  laser_target_gps_node
  
    `Topic:` /target_position

    ---

  - **VIO - 模式**

    ```bash
    ros2 launch target_position_pkg vio_laser_target_gps_launch.py
    ```

  - **Config**

    ```yaml
    initial_lat:      # 初始緯度
    initial_lon:      # 初始經度
    vio_pose_topic:   # VIO 四元數姿態消息
    ```

    `Node:`  vio_laser_target_gps_node
  
    `Topic:` /target_position

    ---

  > **★ 補充 ( Optional )**

  - **外掛式雷射測距模組 - (LRFX00M1LSQ)**

    ```bash
    ros2 run laser_pkg extra_laser_node
    ```

    ---
  