# 机器狗机房仿真 ROS 2 Jazzy 接口

节点名称：`/jifang_robot_environment`。话题前缀：`/robot_environment`。`<camera>` 替换为 `top`、`left`、`right`。使用 ROS 标准消息，无需自定义消息或 colcon 构建。相机控制直接使用角度和高度数值，本仿真不发布 `/tf` 或 `/tf_static`。以下“输入”表示外部程序发布、仿真订阅，“输出”表示仿真发布。

## 启动与查看

在项目根目录执行，无需 sudo：

```bash
source /opt/ros/jazzy/setup.bash
bash run.sh
```

默认启用 ROS，保留完整 GUI 控制台。界面操作与 ROS 操作共享状态；外部相机/位姿/检测命令会同步到控件，控件反馈不会被当作新命令反复覆盖机器人速度。

```bash
# 不打开窗口，持续运行同一套 ROS 接口
bash run.sh --headless

# 只使用本地 GUI，不加载 ROS
bash run.sh --no-ros

# 修改启动参数
bash run.sh --headless \
  --ros-args -p image_rate_hz:=1.0 -p velocity_timeout_s:=0.5
```

其他终端也需要 `source /opt/ros/jazzy/setup.bash`，并使用相同 `ROS_DOMAIN_ID` 和兼容的 DDS 配置：

```bash
ros2 node info /jifang_robot_environment
ros2 topic list -t
ros2 topic echo /robot_environment/state --once
ros2 topic echo /robot_environment/command_status
```

`--frames N` 用于有限次无窗口循环，默认 0 表示持续运行；它不是图像帧数。Ctrl+C 退出。运行日志放在 `runtime/ros_logs/`。

## 坐标系、单位及控制范围

- `world`：Z 朝上，X 从走廊入口指向末端门，Y 朝左排机柜。初始机器人地面投影中心 `(0.9, 0, 0)` m，朝向 +X。
- `robot_base`：+X 朝机身前方、+Y 朝左、+Z 朝上。机器人仅平移 X/Y、旋转 yaw，Z 恒为 0。
- `<camera>_camera_optical_frame`：ROS 光学坐标，+X 为画面右方、+Y 朝下、+Z 沿光轴朝前。图像与 CameraInfo 使用这个 frame。
- 消息中的高度/位置单位为 m、机器人速度为 m/s、机器人角速度为 rad/s。相机 yaw、pitch、水平视场角的话题单位均为**度**。相机接口不使用四元数；机器人原有 PoseStamped 接口的四元数字段为 x/y/z/w。
- 机身 yaw 正向为绕 +Z 逆时针（左转）。界面中的相机 yaw 则以机身左侧为 0°，±180°朝右，正 pitch 抬头。

左右安装光心固定为 `(0, ±0.210, 0.700)` m；距简化机身侧面 50 mm。顶部光心为 `(0, 0, 0.725 + pole_length_m)`，杆长 0.600–1.200 m，光心高度 1.325–1.925 m。相机支持 yaw ±180°、pitch ±80°，不支持独立 roll 或任意移动侧安装点。

这是固定站姿的几何仿真：速度通过积分改变 mocap 位姿，没有步态或轮驱动力学，机器狗可穿过柜体和门。位置限制为 world X∈[−2,9] m、Y∈[−3,3] m；抵达边界时停止运动。当前不发布 `/clock`，`use_sim_time` 必须为 false。

## 机器狗接口

| 方向 | 话题 | 消息类型 | 内容 |
| --- | --- | --- | --- |
| 输入 | `/robot_environment/robot/cmd_vel` | `geometry_msgs/msg/Twist` | 机身坐标速度：linear.x 前后，linear.y 左右，angular.z 左右旋转 |
| 输入 | `/robot_environment/robot/set_pose` | `geometry_msgs/msg/PoseStamped` | world 中的绝对平面位姿，frame_id 必须为 world、z=0、四元数仅包含 yaw |
| 输出 | `/robot_environment/robot/pose` | `geometry_msgs/msg/PoseStamped` | world 中当前机器人位姿 |
| 输出 | `/robot_environment/robot/velocity` | `geometry_msgs/msg/TwistStamped` | robot_base 中当前积分使用的速度；超时或停止后归零 |

速度上限：linear.x、linear.y **各**为 ±1 m/s，angular.z 为 ±2 rad/s；linear.z、angular.x、angular.y 必须为 0。超范围、NaN、Inf 均拒绝，不截断输入。旋转和平移可同时执行，平移方向随 yaw 连续变化。

默认 0.5 s 没有收到有效速度命令就停止，建议 10 Hz 连续发布。无效命令不会刷新超时。GUI 手动修改机器人位姿、复位或成功接收 `robot/set_pose` 会清零速度；修改相机或检测开关不会清零速度。后续新的有效 cmd_vel 可以恢复移动。

```bash
# 前进 0.2 m/s，同时左转 0.3 rad/s（持续发布，Ctrl+C 后超时停止）
ros2 topic pub -r 10 /robot_environment/robot/cmd_vel \
  geometry_msgs/msg/Twist \
  '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}'

# 左移 0.1 m/s；右移或后退用对应的负值
ros2 topic pub -r 10 /robot_environment/robot/cmd_vel \
  geometry_msgs/msg/Twist '{linear: {y: 0.1}}'

# 立即停止（若其他发布端仍持续发布非零速度，下一条命令会再次生效）
ros2 topic pub --once /robot_environment/robot/cmd_vel \
  geometry_msgs/msg/Twist '{}'

# 传送到 world (1.5, 0, 0)，朝 +X
ros2 topic pub --once /robot_environment/robot/set_pose \
  geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: 1.5}, orientation: {w: 1.0}}}'
```

## 相机角度与顶部高度接口

每个控制量单独发布数值，不需要位置向量、四元数或 TF。修改一个量保持其他量不变；多个话题按实际接收顺序生效，不组成原子联合命令。反馈输出为当前数值。

| 方向 | 话题 | 消息类型 | 单位与范围 |
| --- | --- | --- | --- |
| 输入 | `/robot_environment/cameras/<camera>/set_yaw` | `std_msgs/msg/Float64` | 水平角，−180–180° |
| 输入 | `/robot_environment/cameras/<camera>/set_pitch` | `std_msgs/msg/Float64` | 俯仰角，−80–80° |
| 输出 | `/robot_environment/cameras/<camera>/yaw` | `std_msgs/msg/Float64` | 当前水平角，° |
| 输出 | `/robot_environment/cameras/<camera>/pitch` | `std_msgs/msg/Float64` | 当前俯仰角，° |
| 输入 | `/robot_environment/cameras/top/set_height` | `std_msgs/msg/Float64` | 顶部相机光心距地高度，1.325–1.925 m |
| 输出 | `/robot_environment/cameras/top/height` | `std_msgs/msg/Float64` | 当前顶部相机光心距地高度，m |

`<camera>` 可为 top、left、right。**仅顶部相机提供高度话题**；侧面两个光心始终为 0.700 m。高度指光心距地面的总高度，不是杆长；内部换算 `pole_length_m = height − 0.725`。GUI 仍用原有伸缩杆控件并同步显示光心高度。注意 `/state.controls.top_height` 是图像像素高度（例如 2160），而本节 `cameras/top/height` 是光心距地高度（m）；`pole_length_m` 才是杆长。

yaw 相对机身定义：0°朝左侧（+Y），±180°朝右侧（−Y），正值向机身后方转；−90°朝前，+90°朝后。pitch 正值抬头，负值低头。默认顶部 yaw=0°、pitch=−35°；左侧 0°/−40°；右侧 180°/−40°。角度和高度必须有限，越界、NaN、Inf 均拒绝并输出 command_status。不存在侧面高度控制、相机 roll 或安装点移动控制。

```bash
# 顶部光心升到1.725 m（对应杆长1.000 m），朝左侧，俯视35°
ros2 topic pub --once /robot_environment/cameras/top/set_height \
  std_msgs/msg/Float64 '{data: 1.725}'
ros2 topic pub --once /robot_environment/cameras/top/set_yaw \
  std_msgs/msg/Float64 '{data: 0.0}'
ros2 topic pub --once /robot_environment/cameras/top/set_pitch \
  std_msgs/msg/Float64 '{data: -35.0}'

# 左侧朝机身前方，俯视20°；右侧单独调俯仰
ros2 topic pub --once /robot_environment/cameras/left/set_yaw \
  std_msgs/msg/Float64 '{data: -90.0}'
ros2 topic pub --once /robot_environment/cameras/left/set_pitch \
  std_msgs/msg/Float64 '{data: -20.0}'
ros2 topic pub --once /robot_environment/cameras/right/set_pitch \
  std_msgs/msg/Float64 '{data: -40.0}'

ros2 topic echo /robot_environment/cameras/top/height --once
ros2 topic echo /robot_environment/cameras/left/yaw --once
```

旧接口已移除：三路 `cameras/<camera>/set_pose`、`pose`、`pose_local`，以及 本仿真节点的 `/tf`、`/tf_static` 发布。不会兼容订阅旧相机话题，请将调用改为上面的数值话题。机器人 `robot/set_pose`、`robot/pose` 继续提供原有接口。

## 成像参数与图像接口

| 方向 | 话题（每台独立） | 消息类型 | 内容 |
| --- | --- | --- | --- |
| 输入 | `/robot_environment/cameras/<camera>/set_hfov` | `std_msgs/msg/Float64` | 水平视场角，10–150° |
| 输出 | `/robot_environment/cameras/<camera>/hfov` | `std_msgs/msg/Float64` | 当前水平视场角，单位 ° |
| 输入 | `/robot_environment/cameras/<camera>/set_resolution` | `std_msgs/msg/UInt32MultiArray` | data=[width,height]，每项为 240–4096 的整数；layout.dim=[]、data_offset=0 |
| 输出 | `/robot_environment/cameras/<camera>/resolution` | `std_msgs/msg/UInt32MultiArray` | data=[width,height] |
| 输出 | `/robot_environment/cameras/<camera>/image_raw` | `sensor_msgs/msg/Image` | 原始 RGB 图像，encoding=rgb8、step=width×3 |
| 输出 | `/robot_environment/cameras/<camera>/camera_info` | `sensor_msgs/msg/CameraInfo` | 与图像对应的尺寸、K/R/P 和零畸变 D |

默认均为 **3840×2160**，顶部水平视场角 **115°**，左右各 **60°**。可以独立设不同分辨率，包括非 16:9 图像；GUI 会显示 ROS 设置的自定义尺寸。

针孔内参：`fx = fy = width / (2*tan(hfov/2))`，`cx=width/2`、`cy=height/2`；R 为单位阵，P=[K|0]，plumb_bob 的五个畸变参数均为 0。垂直视场角按宽高比换算。

```bash
# 顶部改为115°，左侧改为75°
ros2 topic pub --once /robot_environment/cameras/top/set_hfov \
  std_msgs/msg/Float64 '{data: 115.0}'
ros2 topic pub --once /robot_environment/cameras/left/set_hfov \
  std_msgs/msg/Float64 '{data: 75.0}'

# 左相机改1080p，再恢复4K
ros2 topic pub --once /robot_environment/cameras/left/set_resolution \
  std_msgs/msg/UInt32MultiArray '{data: [1920, 1080]}'
ros2 topic pub --once /robot_environment/cameras/left/set_resolution \
  std_msgs/msg/UInt32MultiArray '{data: [3840, 2160]}'

# 查看内参；图像与内参必须使用兼容的 Best Effort QoS
ros2 topic echo /robot_environment/cameras/left/camera_info \
  --once --qos-reliability best_effort
ros2 topic hz /robot_environment/cameras/left/image_raw \
  --qos-reliability best_effort
```

图像、CameraInfo 和二维码结果共享同一批几何快照的采集时间戳。结果中的 controls 和 camera_position_world_m 来自该图像对应快照，不能用收到结果时最新的 `/state` 代替。机器人状态在独立线程更新，不受耗时渲染阻塞；图像可能晚于对应状态到达。

图像只在有订阅时发送；某相机有 image_raw、camera_info 或 qr_detections 订阅时才触发该路采集。开启全局识别时三路均采集/识别。没有订阅也未开启识别时，headless 不做无用渲染。静态场景可复用确定性理想图像，新的发布批次标记新的快照时间；并非硬件曝光或硬件同步。

## 二维码检测接口

| 方向 | 话题 | 消息类型 | 内容 |
| --- | --- | --- | --- |
| 输入 | `/robot_environment/qr_detection/set_enabled` | `std_msgs/msg/Bool` | 全局开关，影响三路相机；默认 false |
| 输出 | `/robot_environment/qr_detection/enabled` | `std_msgs/msg/Bool` | 当前全局开关状态 |
| 输出 | `/robot_environment/cameras/<camera>/qr_detections` | `std_msgs/msg/String` | JSON，包含本路当前批次检测结果和图像元数据 |

```bash
ros2 topic pub --once /robot_environment/qr_detection/set_enabled \
  std_msgs/msg/Bool '{data: true}'
ros2 topic echo /robot_environment/cameras/left/qr_detections --once

# 关闭检测
ros2 topic pub --once /robot_environment/qr_detection/set_enabled \
  std_msgs/msg/Bool '{data: false}'
```

结果 JSON 示例（controls 缩略，实际包含所有机器人/相机设置）：

```json
{
  "stamp": {"sec": 1792731600, "nanosec": 123456789},
  "frame_id": "left_camera_optical_frame",
  "camera": "left",
  "enabled": true,
  "width": 3840,
  "height": 2160,
  "hfov_deg": 60.0,
  "detections": [
    {
      "payload": "L02-1/39-0.12",
      "rack_id": "L02",
      "height_m": 0.12,
      "corners_px": [[120, 220], [200, 220], [200, 300], [120, 300]]
    }
  ],
  "controls": {"robot_x_m": 0.9, "robot_y_m": 0.0},
  "camera_position_world_m": [0.9, 0.21, 0.7],
  "camera_yaw_deg": 0.0,
  "camera_pitch_deg": -40.0,
  "camera_height_m": 0.7
}
```

示例数值仅说明格式。camera_yaw_deg/pitch_deg 相对机身，camera_height_m 为光心距地高度；结果不再包含相机四元数。corners_px 顺序为左上、右上、右下、左下，坐标来自原图像素而非 GUI 缩略图。内容无法在 equipment.json 中匹配时，rack_id、height_m 为 null。关闭识别后，新采集/发布批次输出 enabled=false 和 detections=[]；开启但没读到二维码时是 enabled=true 和 detections=[]。已经在途的旧批次可能仍先到达，应按 stamp/enabled 判断。这里提供识别框和设备已知高度，不是二维码三维定位估计。

## 状态、命令反馈与 QoS

| 话题 | 消息类型 | 语义 |
| --- | --- | --- |
| `/robot_environment/state` | `std_msgs/msg/String` | JSON：controls（全部设置）、detect（开关）、velocity=[vx,vy,wz]、stamp（状态快照时间） |
| `/robot_environment/command_status` | `std_msgs/msg/String` | ROS 输入命令反馈 JSON：topic、accepted、reason、stamp（处理时间） |

accepted=true 表示目标状态已接受，渲染会在下一批更新。accepted=false 表示命令被拒绝，reason 给出原因；不是服务调用或带 request_id 的逐请求 RPC。GUI 操作不产生 command_status，但会反映到 state。多个控制源的命令按实际处理顺序生效，不设独占控制权。

controls 包括 robot_x_m、robot_y_m、robot_yaw_deg、pole_length_m，以及每路 `<camera>_yaw_deg`、`<camera>_pitch_deg`、`<camera>_hfov_deg`、`<camera>_width`、`<camera>_height`。GUI角度显示可能四舍五入，ROS 状态保留内部精度。

| 接口 | Reliability | Durability | Depth / 发布频率 |
| --- | --- | --- | --- |
| 所有命令输入、command_status | Reliable | Volatile | depth 10；命令收到即处理 |
| robot pose/velocity | Reliable | Volatile | depth 10；默认 20 Hz |
| state、enabled、yaw、pitch、top/height、hfov、resolution | Reliable | Transient Local | depth 1；默认 20 Hz，晚订阅可取最近状态 |
| image_raw、camera_info | Best Effort | Volatile | depth 1；目标上限默认 2 Hz |
| qr_detections | Reliable | Volatile | depth 10；跟随图像采集批次 |

4K 三路渲染、解码和 DDS 传输都耗时，2 Hz 是目标上限，实际采样频率可能更低；速度积分/状态由独立 ROS 执行线程维护。处理掉线超时使用 monotonic 墙钟，不依赖画面刷新或 MuJoCo 门动力学步进时间。可降低分辨率或 image_rate_hz 以减少负载。

启动参数（仅启动时读取，不支持运行期间 ros2 param set 修改）：

| 参数 | 类型 | 默认 | 范围/单位 |
| --- | --- | --- | --- |
| velocity_timeout_s | double | 0.5 | 0.05–5 s |
| state_rate_hz | double | 20.0 | 1–100 Hz，机器人/相机数值状态和积分定时器 |
| image_rate_hz | double | 2.0 | 0.1–15 Hz，图像与检测目标批次上限 |

## 已执行的验证

```bash
source /opt/ros/jazzy/setup.bash
.venv/bin/python validate_ros.py
.venv/bin/python validate_ros_gui.py
```

测试默认分别使用隔离的 ROS_DOMAIN_ID=154/155；可以通过环境变量覆盖。前者启动独立无窗口服务进程与测试客户端，逐项覆盖全部 **16 个控制话题和 27 个输出话题**，核对消息类型、实际接收和发布端 QoS，并验证三路 4K 图像字节数、三路角度、顶部高度、内参、旧话题与 TF 的移除、速度超时、非法命令拒绝、真实二维码解码及图像/内参/结果一致性；后者验证 ROS 与离屏 GUI 的双向同步。记录见 `verification/ros_validation.json` 和 `verification/ros_gui_validation.json`；前者包含本轮测试时间、代码提交号及逐话题清单。

消息定义参考 ROS 官方 [Float64](https://github.com/ros2/common_interfaces/blob/jazzy/std_msgs/msg/Float64.msg)、[Twist](https://github.com/ros2/common_interfaces/blob/jazzy/geometry_msgs/msg/Twist.msg) 和 [CameraInfo](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/CameraInfo.msg)。
