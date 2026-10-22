# 机房巡检相机实验平台

本平台复用 `/home/zy/E_dog_sim/JiFang_world/01_single_cabinet/scene.xml` 中的单机柜和 35 个正面二维码，提供可移动相机、中文控制窗口、实际二维码解码、ROS 2 接口和参数对比实验。

已在本机 Ubuntu 22.04 / x86_64 / ROS 2 Humble / MuJoCo 3.16.0 验证。程序使用当前可用的 OpenGL 设备；宿主机实测使用 NVIDIA RTX 4060。受限测试环境会回退为 Mesa llvmpipe 软件渲染（CPU），状态栏会显示实际渲染设备。相机初始参数为 1920×1080、水平视场角 60°，目标帧率 5 Hz；实际速度取决于分辨率与识别耗时，CPU 忙时会降低更新频率。

## 1. 一键打开中文窗口

```bash
cd /home/zy/E_dog_sim/JiFang_world/02_single_camera
bash run.sh
```

默认同时启动 ROS 2 节点 `jifang_inspection_camera`。不需要先打开单机柜窗口；该程序自己加载 MuJoCo 场景。

窗口左侧可以拖动或输入：水平位置、高度、yaw、pitch、视场角、分辨率、帧率。也能切换到“传感器＋焦距”模式。

- 右上是相机实际图像；绿框是解码器识别成功的二维码，标签显示机箱编号和高度。
- “静态画面复用”：默认启用。机柜场景和相机参数均不变时，复用相同的理想图像和解码结果，ROS 仍按目标频率提供图像；相机参数变化和自动巡检会重新采图。以后接入会运动的机器人或动态物体时关闭此选项。
- 右下是整体场景；绿线是轨道，黄色物体是相机，黄线是视场锥。场景图可用鼠标拖动旋转、滚轮缩放；右上方下拉框可切换斜视、俯视、正视。
- “机箱编号 → 移至并对准”：将相机移动到该二维码前方，调整高度；超出升降范围的低位/高位二维码通过 pitch 对准。
- “自动巡检”：依次采样 35 个机箱对应视角，记录逐帧识别结果和累计覆盖率。扫描过程中暂时锁定手动参数，避免混入其他控制命令；可随时停止并保存部分报告。
- “保存图像与配置”：保存原始 RGB 图、相机配置、内参、可见性指标和识别结果。
- “载入配置”：可以载入保存的 `settings.json` 或 `result.json`。
- 关闭窗口会停止图像发布并退出节点。

如只想使用相机窗口，不接 ROS：

```bash
bash run.sh --no-ros
```

ROS 2 日志默认写入 本目录的 `runtime/ros_logs`，实验结果写入 本目录的 `experiments/`。

## 2. 轨道、坐标和角度

长度单位 m，角度参数单位度。世界坐标 Z 向上。

- 机柜外框正面为 `y=-0.500 m`。
- 相机光心固定在 `y=-1.100 m`，与前框平面的垂直距离为 0.600 m。
- 二维码表面稍微内凹，与相机的法向距离约 0.612 m。
- 水平轨道 `x ∈ [-1.0, +1.0] m`，相机高度 `z ∈ [0.60, 1.80] m`。
- yaw=0 看向 +Y（机柜）；yaw 正值向相机左侧转，范围 ±90°。
- pitch 正值抬头，负值低头，范围 ±80°。
- 相机位置/姿态通过 MuJoCo mocap body 更新，属于即时位姿采样；第一版没有电机动力学、移动速度或真实运动模糊。
- 相机外壳、轨道、视场锥只显示在场景总览中，不会出现在传感器图像里，也不参与碰撞。

TF 链：

```text
world → camera_mount → camera_link → camera_optical_frame
```

`camera_mount` 的位置是轨道位置，轴与 world 对齐；`camera_link` 包含相机姿态，局部 X 前、Y 左、Z 上；`camera_optical_frame` 则为 X 右、Y 下、Z 前。ROS 的 pose/target_pose 都表示 **camera_link 在 world 中的位姿**，不是 optical frame 的位姿。

## 3. 镜头配置

### 视场角模式

设置分辨率和水平视场角，使用正方形像素，自动推导垂直视场角与 K：

```text
fx = fy = width / (2 tan(HFOV/2))
cx = width/2, cy = height/2
VFOV = 2 atan(height/(2 fy))
```

### 传感器＋焦距模式

输入传感器**有效成像宽、高（mm）**和焦距（mm）：

```text
fx = width × focal_mm / sensor_width_mm
fy = height × focal_mm / sensor_height_mm
```

传感器使用有效成像区域尺寸，不直接填写“1/2.8 英寸”之类的光学规格名称。若有效区域宽高比与输出宽高比不同，程序按相应 fx、fy 模拟缩放；实际相机是否裁切、缩放或使用非正方形像素，应以厂商参数或实测标定为准。

竖向分辨率预设会交换有效传感器宽高，或在视场角模式中交换水平/垂直视场角，模拟竖装后保持正立的输出画面。当前不开放任意 roll。

发布的 `CameraInfo.K/P` 与 MuJoCo 渲染使用相同内参，已通过非等 fx/fy 的投影检查。

## 4. ROS 2 接口

| 接口 | 类型 | 说明 |
| --- | --- | --- |
| `/inspection_camera/image_raw` | `sensor_msgs/msg/Image` | 原始 `rgb8`，实际传感器分辨率 |
| `/inspection_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 与图像相同时间戳、相同 optical frame |
| `/inspection_camera/pose` | `geometry_msgs/msg/PoseStamped` | world 中的 camera_link 位姿 |
| `/inspection_camera/target_pose` | `geometry_msgs/msg/PoseStamped` | 接收 world 中的 camera_link 目标位姿 |
| `/inspection_camera/qr_detections` | `std_msgs/msg/String` | JSON：解码内容、机箱编号、像素角点、覆盖率、投影指标、当前参数 |
| `/inspection_camera/reset_coverage` | `std_srvs/srv/Trigger` | 清空累计识别 |
| `/inspection_camera/start_scan` | `std_srvs/srv/Trigger` | 图形界面模式下启动自动巡检 |
| `/tf`, `/tf_static` | TF2 | 相机位姿和光学坐标变换 |

图像和 CameraInfo 使用 sensor data QoS（best effort）。RViz2 添加 Image 显示时选择 `/inspection_camera/image_raw`，Reliability 设为 Best Effort；也可以在 RViz2 的 Camera 显示中结合 TF 和 CameraInfo 检查画面。

查看接口和调整相机：

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 param list /jifang_inspection_camera
ros2 param set /jifang_inspection_camera height_m 0.8
ros2 param set /jifang_inspection_camera pitch_deg -30.0
ros2 param set /jifang_inspection_camera hfov_deg 90.0
ros2 service call /inspection_camera/reset_coverage std_srvs/srv/Trigger '{}'
ros2 service call /inspection_camera/start_scan std_srvs/srv/Trigger '{}'
```

相机启动时也支持标准 ROS 参数覆盖：

```bash
bash run.sh --ros-args -p height_m:=0.8 -p pitch_deg:=-30.0
```

所有参数名见 `config/default.json`。width/height 是整数，其他数值参数是浮点数。参数变更先整体验证，通过后才应用；越界参数会被拒绝，界面随 ROS 参数同步更新。

默认姿态下相机朝 +Y，对应 world 中绕 Z 旋转 +90°。一个有效的目标位姿示例：

```bash
ros2 topic pub --once /inspection_camera/target_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: -0.17, y: -1.1, z: 1.2}, orientation: {x: 0.0, y: 0.0, z: 0.70710678, w: 0.70710678}}}'
```

目标必须在固定轨道上、没有 roll，四元数必须归一化。此版本只接受 world frame；后续接机器人时可在控制器中先做 TF 变换，再发布。自动扫描期间，外部位姿和参数控制会被拒绝，避免干扰正在记录的实验。

时间戳使用 ROS 节点时钟；静态世界只更新相机位姿，不发布 `/clock`。记录 bag 示例：

```bash
ros2 bag record -o camera_inspection_bag \
  /inspection_camera/image_raw /inspection_camera/camera_info \
  /inspection_camera/pose /inspection_camera/qr_detections /tf /tf_static
```

二维码消息第一版使用带结构的 JSON，方便无需额外编译就能接入。未来如果需要固定类型或跨语言严格接口，可以增加专用消息包。

## 5. 无窗口采集、巡检、参数比较

只启动 ROS 发布，不打开控制窗口：

```bash
bash run.sh --headless
```

发布并保存 3 帧中的最后一帧，然后退出：

```bash
bash run.sh --headless --frames 3
```

自动巡检：

```bash
bash run.sh --scan --no-ros
```

批量比较 1080p/4K 与 60°/90°/120°：

```bash
bash run.sh --sweep --no-ros
```

自定义比较矩阵：

```bash
bash run.sh --sweep --no-ros \
  --sweep-fovs 70,80,90 \
  --sweep-resolutions 1920x1080,1080x1920
```

传入配置或输出目录：

```bash
bash run.sh --config /完整路径/settings.json --output /完整路径/实验目录
```

无窗口普通模式持续运行时支持 ROS 参数和 target_pose；`start_scan` 服务只在图形界面模式可用，无窗口扫描使用 `--scan`。

扫描策略是对 35 个机箱逐个居中取样，并将相机高度限制在 0.60–1.80 m。它是一个可复现的覆盖基线，尚未优化最少停靠次数，也不意味着需要真的停 35 次。报告保存每帧识别编号，可据此进一步选择更少拍摄位置。

输出：

- `summary.json`：累计识别、漏扫编号、单帧最大识别数。
- `frames.json`：每帧参数、内参、可见性、二维码像素尺寸、观察角度和实际解码。
- `frames.csv`：便于绘图或比较的逐帧摘要。
- `comparison.json/csv`：各组镜头参数的汇总。

## 6. 第一轮本机基线结果

以下是 CPU 软件渲染、理想针孔、静止位姿、当前机柜模型、ZXing-C++ 解码器下的结果。每组采样 35 个位姿：

| 分辨率 | 水平视场角 | 累计识别 | 单帧最多识别 |
| --- | --- | --- | --- |
| 1920×1080 | 60° | 35/35 | 8 |
| 1920×1080 | 90° | 35/35 | 13 |
| 1920×1080 | 120° | 5/35 | 3 |
| 3840×2160 | 60° | 35/35 | 8 |
| 3840×2160 | 90° | 35/35 | 13 |
| 3840×2160 | 120° | 35/35 | 22 |

这些是本轮采样、像素对齐、渲染及解码算法下的结果，不能当成相机硬件的保证指标。广角低分辨率二维码像素不足时解码显著下降；本基线尚未加入真实镜头畸变、失焦、曝光、噪声、运动模糊或灯光反射变化。下一步可在此基础上增加退化模型、不同横向位置/角度策略，然后用候选真实相机复核。

原始结果位于 `baseline_experiments/`；验证记录位于 `verification/`。报告中的耗时只统计渲染与识别，非机器人实际巡检时间。

画幅完整性用二维码四个外边角投影判断；`sampled_unoccluded` 用中心与四个内部角点做射线采样，只是遮挡近似。最终“可读”以实际图像解码为准，不以几何可见或像素阈值代替。

## 7. 作为 ROS 2 包构建（可选）

一键脚本已可直接使用，无须先 colcon build。若要纳入其他 ROS 工作区：

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/jifang_ros_ws/src
ln -s /home/zy/E_dog_sim/JiFang_world/shared/camera_lab ~/jifang_ros_ws/src/jifang_camera_lab
cd ~/jifang_ros_ws
colcon build --symlink-install --packages-select jifang_camera_lab
source install/setup.bash
ros2 launch jifang_camera_lab inspection_camera.launch.py
```

无窗口 launch：

```bash
ros2 launch jifang_camera_lab inspection_camera.launch.py headless:=true
```

运行入口统一使用 venv，优先采用 `EDOG_VENV` 或已激活环境，否则向上查找 `.venv`。在项目根目录执行 `python3 -m venv .venv`，然后执行 `.venv/bin/python -m pip install -r requirements.txt`，即可安装完整 Python 依赖；本目录的 requirements 也转引根清单。ROS 需使用与 venv 匹配的系统 Python 版本，脚本支持 Ubuntu 24.04/Jazzy 和 22.04/Humble；`--no-ros` 跳过 ROS 加载。NumPy 约束为 `<2`。ZXing-C++ 优先使用 pip 安装的当前 Python 版本，`../shared/camera_lab/vendor/` 中的历史二进制仅作兼容保留。


## 8. 卡顿修复与性能诊断

第一版在 Qt 主线程中依次进行渲染、二维码解码、场景预览和 ROS 发布，因此 4K 解码时会阻塞鼠标与滑块。当前版本已经改为：

1. 后台 QThread 独立拥有 MuJoCo 模型、OpenGL 上下文和解码器；主线程只处理控件和最终画面显示。
2. 最多保留一个渲染任务，持续拖动时直接取最新参数，避免积压操作。扫描中止后的旧结果不会混入新扫描。
3. 静态画面复用，避免重复渲染/识别；正式自动扫描仍逐帧计算。
4. 只有界面显示用的预览会缩小，实际扫码、截图和 ROS 图像始终保留设定的 1080p/4K 分辨率。
5. 场景总览使用独立 600×300 缓冲区，视角和相机不变时不重绘。总览关闭阴影和反射，相机实际图像的渲染质量保持原设置。
6. 没有 ROS 图像订阅者时跳过大图像消息的构造和发送；订阅建立后自动恢复。图像、内参、位姿与 TF 始终使用同一帧实际采集参数。

本机 NVIDIA RTX 4060、4K、5 Hz、无 ROS 图像订阅、静态场景下的约 6 秒对照测量：

| 指标 | 第一版 | 优化版 |
| --- | --- | --- |
| CPU 负载（单个逻辑核=100%） | 61.23% | 3.36% |
| 界面 10 ms 定时器事件间隔的 P95 | 108.10 ms | 10.20 ms |
| 静止期间重复渲染/解码次数 | 28 | 0 |

该表量化的是静止时的资源占用及界面响应，不是宣称相机能渲染 100 FPS。连续调整相机位置时，4K、5 Hz 的实测界面事件间隔 P95 为 10.40 ms，CPU 负载约 48.18%（同样以单核=100%计）。动态画面/自动扫描仍需真实计算；本机单次 4K 解码约 65 ms，硬件渲染约 12 ms。可以在 CPU 机器上保留较低目标帧率，或在调视角时关闭实时识别，确定位置后再开启。

性能与功能检查记录在 `verification/performance/`。开启缓存后状态栏显示实际 GPU/CPU 渲染器，统计区显示“静态画面复用”或新图像的后台耗时。如显示 llvmpipe，说明正在使用 CPU 软件渲染，应先确认图形设备/驱动和运行权限，而不是仅提高 fps 参数。

## 整理后的文件归属

- `run.sh`、`config/default.json`：本方案入口和配置。
- `baseline_experiments/`：已有分辨率、视场角与覆盖率基线结果。
- `实验结果/`：安装高度与视角的论证及截图。
- `verification/`、`gui_preview.png`：已有验证记录和界面预览。
- `experiments/`、`runtime/`：新采集结果和运行日志，默认忽略提交。

机柜模型复用 `../01_single_cabinet/`，Python/ROS 代码共用 `../shared/camera_lab/`。历史报告中的原始绝对路径保留，代表当时的采集环境。
