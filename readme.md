# 三摄机器狗机房巡检仿真

项目仅保留机器狗与双列 20 机柜的联合仿真（原 04），代码、模型、纹理和 ROS 2 Jazzy 接口均位于当前目录，无需其他实验目录。

## 启动

在项目根目录执行：

```bash
# 中文控制台 + ROS 2 Jazzy
bash run.sh

# 不使用 ROS，只打开控制台
bash run.sh --no-ros

# 无窗口运行 ROS 接口
bash run.sh --headless
```

启动脚本自动选择 `EDOG_VENV`、已激活的虚拟环境或项目 `.venv`，并加载已有的 Jazzy。安装步骤见 [Ubuntu安装与依赖指南.md](Ubuntu安装与依赖指南.md)。

## 控制台

- 左侧控制机器人位姿、三路 yaw/pitch、顶部升降、各路水平视场角和分辨率；门只用一个按钮切换开闭。
- 中间显示场景，可选择跟随机器狗、通道、俯视、总览、门口或自由观察；跟随/自由视角支持拖动旋转和滚轮缩放。
- 右侧竖排显示可切换的相机画面和二维码结果；勾选识别后按原图解码，显示识别框、二维码内容、机柜及高度。
- 三路默认 **3840×2160**，顶部水平视场角 **115°**，左右各 **60°**。分辨率可选 720p、1080p、1440p、4K，ROS 还支持自定义尺寸。
- 机器狗前后、左右移动相对机身方向，平移和旋转步长可调，按住按钮连续操作；也可直接输入世界 X/Y 和朝向。
- “复位机器人与相机”恢复位姿、角度、杆高及成像参数；“保存画面与当前配置”保存到 `captures/`。

## 模型与接口

两列各 10 柜，正面净距 1.20 m；机器狗初始位于 `(0.9, 0, 0)`，朝 +X。左右光心位于机身局部 `(0, ±0.210, 0.700)` m，距简化机身侧面 50 mm；顶部光心高度 1.325–1.925 m，杆长 0.600–1.200 m。

机器狗是固定站姿尺寸代理，通过 mocap 直接改变位姿或积分速度，不包含步态、轮驱动或机器人碰撞动力学。机身、支架、杆保留成像遮挡；门使用双铰链和位置执行器。尺寸来源见 `robot_sources.json`。

Jazzy 命名空间为 `/robot_environment`，有 16 个控制话题和 27 个输出话题。支持速度控制、相机 yaw/pitch、顶部光心高度、视场角与分辨率、图像和内参、二维码开关与结果。速度命令默认 0.5 s 超时停止，GUI 与 ROS 双向同步。相机控制使用数值，不发布 TF。

完整话题、类型、坐标系、QoS、范围和调用示例见 [ROS2接口文档.md](ROS2接口文档.md)。

## 文件结构

```text
.
├── run.sh / viewer.py          # 启动脚本与中文控制台
├── controller.py               # 渲染、缓存与 ROS 状态协调
├── simulation.py / physics.py  # 三相机仿真与门动力学运行
├── robot_model.py              # 机器狗尺寸、坐标计算和几何生成
├── camera_config.py            # 各路默认视场角、分辨率和投影换算
├── ros_interface.py            # Jazzy 接口
├── scene.xml                   # 联合场景入口
├── robot.xml / room.xml        # 机器狗 / 双列机柜与建筑
├── assets.xml / assets/        # 材质和全部 700 张本地纹理
├── layout.json                 # 布局摘要
├── equipment.json / .csv       # 设备与二维码真值
├── floor_plan.svg              # 机房平面图
├── build_scene.py              # 从本地源重建联合入口和机器狗 XML
├── scripts/venv.sh             # 虚拟环境与 ROS 环境选择
├── validate_*.py               # 场景、可搬移性、GUI 与 ROS 验证
├── verification/               # 验证报告
└── previews/                   # 场景与控制台预览
```

可整体复制项目目录；无需历史实验、vendor 或其他仓库路径。`runtime/` 和 `captures/` 为本机运行输出，不纳入提交。`.venv/` 是本机依赖环境，换机器应重新创建。

修改 `robot_model.py` 或相机默认值后重建：

```bash
.venv/bin/python build_scene.py
```

机柜场景以本地 `room.xml` 为准；所有 include、纹理路径均在项目内。外部 Python 程序可通过 `Path('scene.xml').resolve()` 加载模型。

## 测试

```bash
# 场景、三路4K、镜头参数、二维码、门、位姿及重建
MUJOCO_GL=egl .venv/bin/python validate_scene.py
# 复制到临时目录后验证独立加载、重建和离屏采集
.venv/bin/python validate_portability.py
# 本地控制台，离屏运行
.venv/bin/python validate_gui.py

# Jazzy 真实双进程通信与 GUI 联动，使用隔离 ROS 域
source /opt/ros/jazzy/setup.bash
.venv/bin/python validate_ros.py
.venv/bin/python validate_ros_gui.py
```

结果保存在 `verification/`。ROS 验证覆盖全部 16 个控制和 27 个输出话题；GUI 验证包含数值相机控制、任意分辨率同步和速度控制互不覆盖。
