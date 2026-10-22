# 机器狗双侧相机评估平台

## 启动

```bash
cd /home/zy/E_dog_sim/JiFang_world/03_robot_cameras
bash run.sh
```

默认打开中文控制窗口并发布 ROS 2。单相机平台位于 `../02_single_camera/`。

- 左侧控件调整整机 X/Y 平移、整机朝向及伸缩杆高度。
- 顶部、左侧、右侧相机分别有独立 yaw、pitch 和水平视场角。右上三个页签查看对应的全分辨率相机画面。
- 底部场景显示机柜、站姿机器狗、三台相机及视场锥，支持拖动旋转和滚轮缩放。
- “左侧靠柜／右侧靠柜”切换整机朝向；“转向此码”只调整所选相机，“平移对齐此码并转向”会移动整机，顶部相机还会在实际伸缩杆范围内调整高度。
- 表格单独显示三路的当前解码数量，以及最底部 1/2/3 号二维码是否读出。
- “保存三路图像/配置”保存三张原始 PNG、逐相机识别结果、整机配置及总览；默认目录为 本目录的 `experiments/`。

只打开界面、不接 ROS：

```bash
bash run.sh --no-ros
```

## 尺寸来源与模型范围

使用**宇树 A2-W** 尺寸。厂商页面核对日期：2026-10-21。

| 项目 | 数值 | 来源 |
| --- | --- | --- |
| 站立长×宽×高 | 900×440×625 mm | 宇树官方参数 |
| 轮胎直径×宽度 | 190×51 mm | 宇树官方参数 |
| 伸缩杆总高 | 600–1200 mm | 项目输入 |
| 顶部云台增加高度 | 100 mm | 项目输入；暂按光心竖向偏移处理 |
| 双侧相机光心高度 | 700 mm | 本次需求 |
| 双侧光心横向位置 | 机身中心线左右各 320 mm | 设计假设；每侧超出原机身宽度包络 100 mm |
| 相机旋转扫掠半径 | 50 mm | 设计假设，包住简化相机外壳 |

官方来源：<https://www.unitree.com/A2-W/>，对应“Standing dimensions”“Tire diameter”“Tire width”字段。机器可读记录见 `robot_sources.json`。

外形为固定的机身、四条简化腿和轮胎，只用于空间占位、视角与遮挡研究。**没有关节、步态、轮驱动、姿态控制或运动动力学；整机通过 mocap 位姿直接平移/旋转，始终站立。** 机身、腿部、支架的细分尺寸不是厂家 CAD，精确硬件装配须用实物或正式 CAD 复核。

底盘外形尺寸按官方 625 mm 站高建模，所以顶部相机光心为：

```text
0.625 + (0.600–1.200) + 0.100 = 1.325–1.925 m
```

该值与之前项目约数 1.30–1.90 m 的差异来自站高采用官方 0.625 m，而不是约 0.600 m。两台侧相机在伸缩杆变化时仍保持 0.700 m。

## 坐标、间隙及遮挡

- `world`：Z 向上；机柜正面外框位于 `y=-0.500 m`，二维码表面约 `y=-0.488 m`。
- `robot_base`：原点为整机中心的地面投影，X 向机器狗前方、Y 向左、Z 向上；它不是厂家 SDK 中已标定的基座坐标。
- 默认整机中心 `(-0.170, -1.420, 0)`，朝向世界 +X，与机柜并行。
- 默认左侧相机为靠柜侧，光心 `(-0.170, -1.100, 0.700)`，距前框 0.600 m；顶部和对侧相机距前框分别为 0.920 m、1.240 m。三台相机的距离不被强行设为相同。
- 相机 yaw 相对机身定义：0°看向机身左侧（+Y），±180°看向右侧（−Y），正值向机身后方转；pitch 正值抬头、负值低头。每台支持 yaw ±180°、pitch ±80°；此处为评估用范围，尚未对应具体云台限位。
- 用 50 mm 半径球保守包住每台相机的全部旋转外形。双侧球与 900×440×625 mm 整机包络最短净距约 **75 mm**，竖向最低点比机身顶面高 25 mm，避免旋转外形侵入机身尺寸包络。该检查不包含未建模的线缆。
- 加上两侧旋转空间后，安装方案总宽包络约 **740 mm**；通道可用宽度需要按此重新核对。
- 支架在机身处连接属于设计连接点，不视为干涉。界面会拒绝机身/相机包络与机柜明显相交的位姿；这是静态几何检查，不是碰撞动力学。

**机身、腿、杆和支架会真实遮挡相机图像，且参与可见性射线检查。** 两侧相机通常分别服务于机房两侧机柜，不能假定两台都能穿过机身观察同一侧柜底。将对侧相机转向同侧柜底，可以直接看到遮挡；切换整机朝向后，另一台侧相机成为靠柜侧。

相机小外壳、尺寸框和视场锥只在总览显示，避免传感器从自身外壳内部成像；这里没有对机器人主体做透明处理。

## 成像与性能

默认三路均为 1920×1080、3 Hz，独立水平视场角默认 60°。支持 4K 与其他分辨率预设；识别始终使用原始分辨率，不以缩小预览替代扫码。

沿用后台渲染线程、单任务限速及静态缓存，避免三路渲染阻塞滑块。相机/机器人参数变化时重新采图，静止时复用确定性的理想画面。未来接入真实机器人运动或其他动态物体时，应关闭“静态画面复用”。

当前为理想针孔成像，没有真实镜头畸变、曝光、模糊或机器人振动；渲染中读出二维码不能直接等同于实机可靠性。

## ROS 2

节点：`jifang_robot_cameras`。

对 `top`、`left`、`right` 各提供：

```text
/inspection_platform/<camera>/image_raw       sensor_msgs/msg/Image, rgb8
/inspection_platform/<camera>/camera_info     sensor_msgs/msg/CameraInfo
/inspection_platform/<camera>/pose            geometry_msgs/msg/PoseStamped, world 中的相机 link
/inspection_platform/<camera>/qr_detections   std_msgs/msg/String, JSON
```

额外接口：

```text
/inspection_platform/robot_pose          geometry_msgs/msg/PoseStamped
/inspection_platform/robot_target_pose   geometry_msgs/msg/PoseStamped
/inspection_platform/reset_coverage      std_srvs/srv/Trigger
/tf, /tf_static
```

图像与内参使用 sensor data QoS（Best Effort）。每次三路采样结果共用发布时间戳；这是静态位姿的批次一致性，不模拟真实多相机硬件同步。发布的内参/位姿来自该批图像实际采集设置，不混用更新后的 UI 参数。

TF 链：

```text
world → robot_base → top_camera_link   → top_camera_optical_frame
                   → left_camera_link  → left_camera_optical_frame
                   → right_camera_link → right_camera_optical_frame
```

参数控制示例：

```bash
source /opt/ros/humble/setup.bash
ros2 param set /jifang_robot_cameras robot_x_m 0.2
ros2 param set /jifang_robot_cameras robot_y_m -1.6
ros2 param set /jifang_robot_cameras pole_length_m 1.2
ros2 param set /jifang_robot_cameras left_pitch_deg -40.0
ros2 param set /jifang_robot_cameras right_yaw_deg 0.0
```

`robot_target_pose` 必须在 `world` 中，z=0，只允许平面平移和 yaw。参数全表见 `config/robot_platform.json`；侧相机高度固定 0.70 m，不会因升降参数变化。

## 无窗口、配置及独立场景

```bash
# 使用 ROS 发布三路图像，保存一批后退出
bash run.sh --headless --frames 1

# 指定平台配置
bash run.sh --config /完整路径/settings.json

# 无 ROS，保存三路截图
bash run.sh --headless --frames 1 --no-ros

# 仅查看静态站姿模型
simulate robot_scene.xml
```

`robot_scene.xml` 提供静态独立入口；它不包含中文调参窗口或 ROS 节点。可通过以下方式重新导出当前配置：

```bash
bash run.sh --config /完整路径/settings.json \
  --export-model /完整路径/robot_scene.xml
```

## 已完成的验证

记录见 `verification/`，预览见 `previews/`。

- 无活动关节；整机平移/转向后三台相机位姿一致跟随。
- 伸缩杆两端对应顶部 1.325/1.925 m，侧相机恒为 0.700 m。
- 靠柜左相机在默认 1080p、60°条件下读出底部 1/2/3 号二维码。
- 左侧相机在 4K、115°、pitch −6.3°条件下也读出底部 1/2/3 号二维码。
- 对侧相机越过机身看向最底码时受到真实遮挡；右侧靠柜后右相机可读出底部三个码。
- 中文界面平移、伸缩、侧面切换、独立转向及三路保存通过检查。
- 三路 ROS 图像、内参、位姿、TF、参数控制及目标平移通过收发检查。

这些结果只适用于当前尺寸代理和理想成像条件，用于方案比较，尚非真实安装和识别可靠性的最终验收。

## 整理后的文件归属

- `run.sh`、`config/robot_platform.json`：本方案入口和配置。
- `robot_scene.xml`：机器狗与三相机的静态场景，使用相对路径引用方案一的机柜资产。
- `recorded_experiments/`：整理前保存的历史三路图像、配置与识别结果。
- `previews/`、`verification/`、`robot_sources.json`：预览、已有验证记录及尺寸来源。
- `experiments/`、`runtime/`：新采集结果和运行日志，默认忽略提交。

Python/ROS 代码共用 `../shared/camera_lab/`，机柜模型复用 `../01_single_cabinet/`。历史报告中的原始绝对路径保留，代表当时的采集环境。
