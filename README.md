# <div align="center">AESIL - Camera</div>

## <div align="center">Outline</div>

- [程式下載](#downloads)
- [相機控制](#camera)
- [影像串流](#live-stream)
- [影像辨識](#YOLO)

## <div align="center">Downloads</div>

***Step1* - 下載至 ROS2 的工作空間**
```bash
cd ~/<your_workspace>/src
git clone https://github.com/FantasyWilly/AESIL_Camera.git
```
---

***Step2* - 編譯工作空間**
```bash
colon build
source ~/.bashrc
```
---

## <div align="center">Camera</div>

  ### [ D-80 系列 ] - (Pro)

  - **ROS2 Running**

    ```bash
    ros2 run camera_d80_pkg guc_ro2_main_node
    ```

    `Node:` gcu_ros2_main_node
  
    `Topic:` camera_data_pub
    
    ---

  ### [ KTG 系列 ] - (TT30)

  > **★ 三種方式開啟程式 ( Optional )**

  - **ROS2 Running - (Optional)**

    ```bash
    ros2 run camera_tt30_pkg camera_feedback_publisher_gui_node
    ```

    `Node:` gcu_ros2_main_node

    `Topic:` /camera_data_pub, /laser_data_pub

    ---

  - **ROS2 Launch - (Optional)**

    ```bash
    ros2 launch camera_tt30_pkg camera_gui_ros2_launch.py
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
    python3 camera_tt30_pkg/camera_tt30_pkg/xbox_ground.py
    ```

  - **Windows - [ 地面端 ] Xbox Control**

    ⚠️ 記得修改檔案的 `Server` IP, Port 且 在同網域底下

    ```bash
    python3 camera_tt30_pkg/camera_tt30_pkg/windows_ground/xbox_ground.py
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

  - **ROS2 YOLO + 儲存影片**

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
      gps_topic:          # 接收 目標物經緯度 的 GPS 話題
      save_directory:     # 儲存 影片路徑
      base_filename:      # 影片 命名規則
      ```

  ---