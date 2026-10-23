# Ubuntu 24.04 / ROS 2 Jazzy 运行说明

当前项目使用 Python 3.12、MuJoCo、Qt 和 ROS 2 Jazzy。无需 Gazebo、colcon 或历史实验目录。系统已有 ROS 时，运行不需要 sudo。

## Python 环境

在项目根目录使用系统 Python 创建虚拟环境，确保与 Jazzy 的 Python ABI 一致：

```bash
/usr/bin/python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

根 `requirements.txt` 包含 MuJoCo、NumPy、Pillow、PyQt5、PyOpenGL、ZXing-C++ 和 PyYAML。二维码纹理已包含在 `assets/`，无需生成二维码或安装 qrcode。

如需使用已有虚拟环境：

```bash
EDOG_VENV=/完整路径/venv bash run.sh
```

虚拟环境不要从其他系统直接复制。运行时使用 `/usr/bin/python3` 对应版本创建的环境，不混入其他 Python 版本的 ROS 或 MuJoCo 包。

## 已有 ROS 2 Jazzy

```bash
source /opt/ros/jazzy/setup.bash
.venv/bin/python -c 'import rclpy; print(rclpy.__file__)'
bash run.sh
```

`run.sh` 会自动加载已有的 `/opt/ros/jazzy/setup.bash`；自定义 ROS 安装可事先 source 环境。每个外部 ROS 命令终端都要加载环境：

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
```

本机无 ROS 时仍可使用 `bash run.sh --no-ros`，此模式不导入 rclpy。完整接口及 headless 启动参数见 [ROS2接口文档.md](ROS2接口文档.md)。

## 系统图形依赖

控制台需要可用的桌面 DISPLAY；无窗口运行使用 `bash run.sh --headless`。脚本默认 `MUJOCO_GL=egl`，需要系统提供 EGL/OpenGL 和 Mesa 或显卡驱动。若系统缺少基础库，可由管理员提供以下依赖（与 Python 包区分）：

```text
python3-venv
libegl1 libgl1 libopengl0 libgl1-mesa-dri libglib2.0-0
Qt xcb 所需的 libxcb-xinerama0 libxcb-cursor0 libxkbcommon-x11-0
ROS 2 Jazzy：rclpy、sensor_msgs、geometry_msgs、std_msgs、rcl_interfaces
```

本项目不再发布 TF，无需 tf2_ros。已安装 Jazzy 时无需重复安装系统包。

## 检查

```bash
.venv/bin/python -m pip check
bash run.sh --no-ros --headless --frames 3
.venv/bin/python validate_portability.py
.venv/bin/python validate_gui.py
source /opt/ros/jazzy/setup.bash
.venv/bin/python validate_ros.py
.venv/bin/python validate_ros_gui.py
```

ROS 测试需要本地 DDS 网络通信；普通终端运行即可。无窗口和离屏 GUI 测试不需要 DISPLAY。若只测试单个外部控制程序，使用与仿真相同的 ROS_DOMAIN_ID 和兼容的 DDS 配置。
