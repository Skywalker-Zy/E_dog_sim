# Ubuntu 依赖需求与安装教程

更新日期：2026-10-22。

本文对应本仓库的单机柜场景、单相机实验平台、机器狗三相机平台和双排 20 机柜场景。安装命令按 Bash 编写。只需要查看场景或采集图像时，可以不安装 ROS 2。

## 1. 系统与版本选择

| Ubuntu | 系统 Python | 对应 ROS 2 | 本项目使用说明 |
| --- | --- | --- | --- |
| Ubuntu 22.04 LTS | Python 3.10 | Humble | 与仓库原有 ROS 文档及自带扫码库的 Python 版本一致 |
| Ubuntu 24.04 LTS | Python 3.12 | Jazzy | 使用本教程重新安装扫码库，并在启动前加载 Jazzy 环境 |

ROS 与 Ubuntu 的对应关系来自官方安装文档，见文末资料 [1][2]。本次编写时检测到的工作环境为 Ubuntu 24.04.3、Python 3.12.3，存在 `/opt/ros/jazzy`；这不代表当前机器的全部依赖已验证可用。

**本教程以 Ubuntu 24.04 + Jazzy 为主要路径，同时给出 22.04 + Humble 的命令。** 仓库历史验证记录以 MuJoCo 3.16.0 和 Humble 为主；Jazzy 的完整功能需要按第 7 节重新验证。

检查自己的系统：

```bash
cat /etc/os-release
/usr/bin/python3 --version
uname -m
```

以下二进制下载示例针对 `x86_64`。ARM64 设备需要选择对应架构的 MuJoCo 安装包，并重新安装适合该 Python/架构的扫码库。

硬件建议：桌面环境、8 GB 以上内存；三路相机或 4K 实验建议 16 GB 以上内存和可用的 GPU 图形驱动。这是使用建议，不是经过测试的最低配置。CPU 软件渲染可作为兼容方案，但多机柜和 4K 扫码可能较慢。

## 2. 需要安装哪些软件

| 依赖 | 是否必需 | 作用与安装方式 |
| --- | --- | --- |
| Python 3、venv、pip | Python 功能必需 | 运行代码、隔离第三方包；通过 apt 安装 |
| MuJoCo 3.16.0 | 仿真必需 | 场景加载、物理推进、相机渲染；Python 功能通过 requirements.txt 安装 |
| NumPy | Python 功能必需 | 几何、矩阵和相机内参计算；通过 requirements.txt 安装到 venv |
| PyQt5 | 中文界面必需 | 相机和机房控制窗口；通过 requirements.txt 安装 |
| Pillow | 图像保存、双排场景生成必需 | PNG 保存和纹理生成；通过 requirements.txt 安装 |
| PyOpenGL | 相机渲染相关 | OpenGL 接口和界面渲染器信息；通过 requirements.txt 安装 |
| ZXing-C++ Python 绑定 | 相机程序必需 | 实际二维码解码；pip 包名 `zxing-cpp`，导入名 `zxingcpp` |
| OpenGL、EGL、GLFW 库 | 图形功能必需 | 离屏渲染和查看窗口；通过 apt 安装 |
| PyYAML | ROS Python 接口需要 | 通过根 requirements.txt 安装到 venv |
| 中文字体 | 中文显示建议安装 | 避免按钮、标签出现方框 |
| ROS 2、RViz2 | 可选 | 图像/内参/TF 发布、外部控制和可视化 |
| colcon | ROS 包开发可选 | 构建 `jifang_camera_lab`；直接运行脚本不需要 |
| MuJoCo 独立 `simulate` 程序 | 可选 | 使用 `run.sh` / `open_native.sh` 查看原生场景 |
| Git、curl | 获取项目和安装包时使用 | 已有完整项目副本时不需要重新克隆 |

仓库已附带生成二维码所用的 `qrcode` 库，无须额外安装。当前代码不依赖 OpenCV、PyTorch、CUDA Toolkit、Gazebo、Blender 或机器狗厂商 SDK；使用 NVIDIA 显卡渲染时需要相应图形驱动。

根目录 `requirements.txt` 覆盖所有 Python 功能。`JiFang_world/requirements.txt` 和 `JiFang_world/shared/camera_lab/requirements.txt` 转引根清单；双排机房保留独立清单，方便整目录复制。ROS 2、显卡驱动和系统图形库不通过 pip 安装。

## 3. 安装基础系统依赖

### 3.1 安装 Ubuntu 软件包

```bash
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe
sudo apt update
sudo apt install -y \
  git curl ca-certificates locales \
  python3 python3-venv python3-pip \
  libgl1 libegl1 libglfw3 libgl1-mesa-dri libegl-mesa0 \
  libosmesa6 mesa-utils \
  libxcb-xinerama0 libxkbcommon-x11-0 \
  fonts-noto-cjk
```

检查 UTF-8 环境：

```bash
locale
```

已有 `zh_CN.UTF-8` 或其他 UTF-8 locale 时可保留。最小安装环境如果尚无 UTF-8，可执行：

```bash
sudo locale-gen en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 3.2 检查图形环境

在桌面会话的终端执行：

```bash
glxinfo -B
```

检查 `OpenGL renderer` 字段。显示实际显卡名称通常表示使用了硬件渲染；显示 `llvmpipe` 表示使用 CPU 软件渲染。该命令检查桌面 GLX，最终相机的 EGL 渲染器仍以程序显示的信息为准。

NVIDIA 用户可通过 Ubuntu“软件和更新 → 附加驱动”安装适合当前显卡的推荐驱动，重启后用 `nvidia-smi` 检查。Intel/AMD 一般先使用系统 Mesa 驱动。无需为本项目单独安装 CUDA Toolkit。

通过 SSH 运行时，`glxinfo` 或 Qt 窗口可能因没有桌面显示而失败；可以先使用第 7 节的无窗口采集命令验证离屏渲染。

## 4. 安装 Python 仿真与扫码依赖

### 4.1 设置项目路径

假设项目放在 `~/E_dog_sim`。若放在其他目录，请修改下面第一行。

```bash
export EDOG_PROJECT_ROOT="$HOME/E_dog_sim"
cd "$EDOG_PROJECT_ROOT"
test -f JiFang_world/01_single_cabinet/scene.xml
```

本机项目路径为 `/home/zy/E_dog_sim`。原有 README 中出现的 `/home/zy/E_dog/` 是历史路径，不要照搬。

### 4.2 在项目目录建立 venv 并安装依赖

所有启动脚本统一使用虚拟环境，不再固定调用系统 Python。第一次安装：

```bash
cd "$EDOG_PROJECT_ROOT"
/usr/bin/python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

已有 `.venv` 时，无需重新创建，直接激活并安装清单即可。这些操作不需要 sudo，前提是系统已有可用的 Python、venv 和图形运行库。

清单包含 MuJoCo、NumPy、Pillow、PyQt5、PyOpenGL、ZXing-C++，以及 ROS Python 接口需要的 PyYAML。NumPy 限制为 `<2`，兼顾当前 ROS 二进制包；无需额外用 apt 安装 Qt/Pillow 的 Python 包。

如果 `python3 -m venv` 提示缺少 `ensurepip` / `python3-venv`，需要管理员安装对应的系统包，或使用机器上已有的可用虚拟环境。将它的目录通过 `EDOG_VENV` 指定即可。

### 4.3 启动脚本如何选择 venv

选择顺序为：

1. `EDOG_VENV` 指定的虚拟环境。
2. 当前已激活的 `VIRTUAL_ENV`。
3. 从脚本所在目录向上查找最近的 `.venv`。

找不到可用环境时会提示创建环境和安装依赖，不会静默切回系统 Python。选中的环境必须装齐 requirements；请使用系统 `/usr/bin/python3` 创建需要接入 ROS 的 venv，确保 Python 版本与 ROS 二进制包一致。

在本项目根目录建好 `.venv` 后，可以直接执行：

```bash
cd "$EDOG_PROJECT_ROOT/JiFang_world/two_rows_20_racks"
bash run.sh
```

也可以指定其他位置的环境：

```bash
EDOG_VENV="$HOME/其他目录/.venv" bash run.sh
```

原有 `ROOM_PYTHON` 不再选择解释器；请改用 `EDOG_VENV`。`MUJOCO_PYTHON_SITE` / `JIFANG_MUJOCO_SITE` 仅作为直接使用系统 Python 时的兼容配置，正常的 `.sh` 启动不再需要它们。

如果希望保留一个随时可加载的个人环境文件，可创建：

```bash
mkdir -p "$HOME/.config/e_dog_sim"
cat > "$HOME/.config/e_dog_sim/env.sh" <<'ENV'
export EDOG_PROJECT_ROOT="$HOME/E_dog_sim"
export EDOG_VENV="$EDOG_PROJECT_ROOT/.venv"
source "$EDOG_VENV/bin/activate"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
ENV
source "$HOME/.config/e_dog_sim/env.sh"
```

项目路径不同时请修改 `EDOG_PROJECT_ROOT`。旧环境文件中手动添加其他 venv 的 `PYTHONPATH` / site-packages 配置应移除，并在新终端重新加载，避免混用包。

### 4.4 检查 Python 依赖

```bash
cd "$EDOG_PROJECT_ROOT"
source .venv/bin/activate
python -m pip check
python - <<'PY'
import sys
import numpy
import mujoco
import zxingcpp
from PIL import Image
from PyQt5 import QtCore
from OpenGL import GL
print('Python:', sys.executable)
print('NumPy:', numpy.__version__)
print('MuJoCo:', mujoco.__version__)
print('ZXing:', zxingcpp.__file__)
print('Qt:', QtCore.QT_VERSION_STR)
print('基础依赖导入成功')
PY
```

Python 路径应指向项目 `.venv/bin/python`。仓库自带的 ZXing 二进制只支持 CPython 3.10、x86_64；本清单会安装适合当前 Python 的扫码包，正常启动时优先使用 venv 中的版本。

## 5. 安装 ROS 2（可选）

仅使用 `--no-ros` 模式或双排机房时，可跳过这一节。已有匹配系统的 ROS 2 时，直接进入 5.3。

### 5.1 配置 ROS 官方软件源

以下按官方 `ros2-apt-source` 方式配置软件源，见资料 [6]：

```bash
export EDOG_ROS_APT_VERSION="$(curl -fsSL \
  https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | /usr/bin/python3 -c 'import json, sys; print(json.load(sys.stdin)["tag_name"])')"
export EDOG_UBUNTU_CODENAME="$(. /etc/os-release; printf '%s' "${UBUNTU_CODENAME:-$VERSION_CODENAME}")"
curl -fL -o /tmp/edog-ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${EDOG_ROS_APT_VERSION}/ros2-apt-source_${EDOG_ROS_APT_VERSION}.${EDOG_UBUNTU_CODENAME}_all.deb"
sudo dpkg -i /tmp/edog-ros2-apt-source.deb
sudo apt update
sudo apt upgrade
```

如果 GitHub 请求失败，应先解决网络或代理问题，再继续安装软件源包。系统升级完成后，如果提示需要重启，先重启，再继续。

### 5.2 安装对应版本

**Ubuntu 24.04 执行：**

```bash
sudo apt install -y ros-jazzy-desktop ros-jazzy-tf2-ros
```

**Ubuntu 22.04 执行：**

```bash
sudo apt install -y ros-humble-desktop ros-humble-tf2-ros
```

选择与系统对应的一组即可。Desktop 安装包含 RViz2；当前项目直接使用 ROS Python 接口，不需要安装 Gazebo。

### 5.3 加载 ROS 环境

每次新开终端后执行相应版本的命令。

Ubuntu 24.04：

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/.config/e_dog_sim/env.sh"
```

Ubuntu 22.04：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/.config/e_dog_sim/env.sh"
```

相机脚本会按 Ubuntu 版本自动加载 Jazzy（24.04）或 Humble（22.04）；已有 `ROS_DISTRO` 时保留当前环境。`--no-ros` 不加载 ROS。也可按上述命令手动加载，使用自定义 ROS 安装时尤其如此。

检查接口依赖：

```bash
python - <<'PY'
import rclpy
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
print('ROS 2 Python 接口导入成功')
PY
```

## 6. 安装独立 MuJoCo：下载、解压、配置与验证

本节安装能够直接运行 `simulate scene.xml` 的独立程序。以下以 **MuJoCo 3.16.0、Ubuntu Linux x86_64** 为例，下载来源见资料 [7]。

安装顺序：**下载 → 解压 → 放到安装目录 → 修改 `.bashrc` → source → 验证**。

本节沿用当前机器实际使用的 `~/mujuco_work` 目录名。第 4 节 Python 虚拟环境位于 `~/E_dog_sim/.venv`，两者是不同目录：前者存放独立程序，后者存放 Python 依赖。如果你选择其他目录，需同步修改下面的 `MUJOCO_HOME`。

如果已经存在 `~/mujuco_work/mujoco-3.16.0/bin/simulate`，可直接从 **6.4** 开始。

### 6.1 下载压缩包

将官方 Linux x86_64 安装包下载到 `~/Downloads`：

```bash
mkdir -p "$HOME/Downloads"
curl -fL -o "$HOME/Downloads/mujoco-3.16.0-linux-x86_64.tar.gz" \
  https://github.com/google-deepmind/mujoco/releases/download/3.16.0/mujoco-3.16.0-linux-x86_64.tar.gz
```

也可以在浏览器打开文末的官方发布页，下载同名压缩包后放入上述目录。

### 6.2 解压

```bash
tar -xzf "$HOME/Downloads/mujoco-3.16.0-linux-x86_64.tar.gz" \
  -C "$HOME/Downloads"
```

解压后得到 `~/Downloads/mujoco-3.16.0` 文件夹。

### 6.3 放到安装目录

```bash
mkdir -p "$HOME/mujuco_work"
mv -T "$HOME/Downloads/mujoco-3.16.0" "$HOME/mujuco_work/mujoco-3.16.0"
```

目标目录已安装过该版本时，跳过下载、解压和移动步骤。完成后的主要目录结构应为：

```text
~/mujuco_work/mujoco-3.16.0/
├── bin/
│   └── simulate
├── lib/
│   └── libmujoco.so.3.16.0
└── include/
```

### 6.4 修改 `.bashrc`

先备份，再打开配置文件：

```bash
cp -p "$HOME/.bashrc" "$HOME/.bashrc.backup.$(date +%Y%m%d_%H%M%S)"
nano "$HOME/.bashrc"
```

在文件末尾加入以下内容。如果已有旧的 MuJoCo 配置，请替换旧配置，避免重复添加：

```bash
# MuJoCo 3.16.0
export MUJOCO_HOME="$HOME/mujuco_work/mujoco-3.16.0"
export PATH="$MUJOCO_HOME/bin:$PATH"
export LD_LIBRARY_PATH="$MUJOCO_HOME/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

在 nano 中按 `Ctrl+O` 保存、回车确认，再按 `Ctrl+X` 退出。若系统没有 nano，可先执行 `sudo apt install nano`，或使用已有的文本编辑器。

三个变量分别表示：

- `MUJOCO_HOME`：MuJoCo 安装目录，也供本项目的启动脚本查找独立程序。
- `PATH`：加入 `bin`，使终端能直接执行 `simulate`。
- `LD_LIBRARY_PATH`：加入 `lib`，使程序能找到 MuJoCo 动态库；原变量为空时不会额外引入空路径。

### 6.5 加载配置

先检查 Bash 语法，再让配置在当前终端生效：

```bash
bash -n "$HOME/.bashrc" && source "$HOME/.bashrc"
```

配置保存后，新开的交互式 Bash 终端也会读取 `.bashrc`。

### 6.6 验证

检查命令和动态库路径：

```bash
echo "$MUJOCO_HOME"
command -v simulate
ldd "$MUJOCO_HOME/bin/simulate"
```

前两项预期分别输出：

```text
/home/你的用户名/mujuco_work/mujoco-3.16.0
/home/你的用户名/mujuco_work/mujoco-3.16.0/bin/simulate
```

`ldd` 输出中应能解析 `libmujoco.so.3.16.0`，且没有 `not found`。

最后，在桌面终端打开项目场景：

```bash
cd "$HOME/E_dog_sim/JiFang_world/01_single_cabinet"
simulate scene.xml
```

项目位于其他目录时，请修改 `cd` 路径。能够打开 MuJoCo 窗口并显示机柜场景，说明独立程序的安装和路径配置正常。

Python 相机程序仍需安装第 4 节的 Python 依赖。独立压缩包提供的 `simulate` 和 `pip install mujoco` 提供的 Python 库需要分别配置。

## 7. 启动与验证

以下命令默认已完成第 4 节环境配置。直接执行 Python 命令前需激活 venv；`.sh` 会自动选择 venv。GUI 命令逐项运行，关闭当前窗口后再运行下一项。

### 7.1 单机柜：无窗口加载检查

```bash
source "$HOME/.config/e_dog_sim/env.sh"
cd "$EDOG_PROJECT_ROOT/JiFang_world/01_single_cabinet"
python run.py --check
```

成功时输出模型几何数量、body 数量和仿真时间。这个检查不创建图像，不能代替相机渲染检查。

### 7.2 单机柜：查看窗口

使用 Python viewer，不依赖独立 simulate：

```bash
cd "$EDOG_PROJECT_ROOT/JiFang_world/01_single_cabinet"
python run.py
```

或者完成第 6 节后，通过同样加载 venv 的脚本启动独立 simulate：

```bash
cd "$EDOG_PROJECT_ROOT/JiFang_world/01_single_cabinet"
bash run.sh
```

### 7.3 单相机与机器狗三相机：不接 ROS

单相机中文调参窗口：

```bash
cd "$EDOG_PROJECT_ROOT/JiFang_world/02_single_camera"
bash run.sh --no-ros
```

机器狗三相机中文窗口：

```bash
cd "$EDOG_PROJECT_ROOT/JiFang_world/03_robot_cameras"
bash run.sh --no-ros
```

无窗口采集：

```bash
cd "$EDOG_PROJECT_ROOT/JiFang_world/02_single_camera"
bash run.sh --headless --frames 1 --no-ros --output /tmp/edog-camera-check
bash "$EDOG_PROJECT_ROOT/JiFang_world/03_robot_cameras/run.sh" --headless --frames 1 --no-ros --output /tmp/edog-platform-check
```

预期生成带时间戳的结果目录。单相机保存 `image.png`、`result.json`、`settings.json`；三相机保存 `top.png`、`left.png`、`right.png`、各路结果和配置。输出目录由程序打印。

需要完整覆盖实验时，再运行：

```bash
cd "$EDOG_PROJECT_ROOT/JiFang_world/02_single_camera"
bash run.sh --scan --no-ros --output /tmp/edog-scan
```

### 7.4 双排 20 机柜中文窗口

```bash
cd "$EDOG_PROJECT_ROOT/JiFang_world/two_rows_20_racks"
bash run.sh
```

该入口不需要 sudo，自动选择 venv，并使用其中的 MuJoCo、Qt/Pillow 等依赖。该入口不依赖 ROS，也不包含机器狗和三相机平台。检查中文界面、总览、双门开关和门角度控制是否正常。

### 7.5 ROS 图像与 TF 验证

第一个终端，Ubuntu 24.04 执行：

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/.config/e_dog_sim/env.sh"
cd "$EDOG_PROJECT_ROOT/JiFang_world/03_robot_cameras"
bash run.sh
```

22.04 将上面的 Jazzy 环境改为 `source /opt/ros/humble/setup.bash`。

第二个终端加载同版本 ROS 环境后执行：

```bash
ros2 topic list
ros2 topic info /inspection_platform/left/image_raw
ros2 topic echo /inspection_platform/left/camera_info --once --qos-reliability best_effort
ros2 run tf2_ros tf2_echo world left_camera_optical_frame
```

用 `rviz2` 添加 Image，选择 `/inspection_platform/left/image_raw`，Reliability 设为 **Best Effort**。单相机平台对应话题为 `/inspection_camera/image_raw` 和 `/inspection_camera/camera_info`。

## 8. 常见问题

### 8.1 `No module named mujoco` / `zxingcpp`

激活项目 `.venv`，执行 `python -m pip install -r requirements.txt`，再运行第 4.4 节的导入检查。确认 `EDOG_VENV` / `VIRTUAL_ENV` 没有指向其他未装齐依赖的环境。

如果报错涉及 `zxingcpp.zxingcpp` 或 `cpython-310`，检查是否误用了仓库自带的 Python 3.10 二进制。确认已在选中的 venv 中安装 `zxing-cpp`；清除指向旧环境的 `PYTHONPATH` 配置后重启终端。

### 8.2 `No module named rclpy`

需要 ROS 时，安装匹配版本并 source 对应的 `/opt/ros/<版本>/setup.bash`。只运行相机实验时，加 `--no-ros`。

### 8.3 EGL / OpenGL 初始化失败

先检查显卡驱动与第 3 节的系统图形库。项目默认 `MUJOCO_GL=egl`；也可以在新进程启动前切换后端。

有桌面显示时尝试 GLFW：

```bash
cd "$EDOG_PROJECT_ROOT/JiFang_world/02_single_camera"
MUJOCO_GL=glfw bash run.sh --no-ros
```

没有可用硬件 EGL 时，尝试 OSMesa 软件渲染：

```bash
cd "$EDOG_PROJECT_ROOT/JiFang_world/02_single_camera"
MUJOCO_GL=osmesa bash run.sh --headless --frames 1 --no-ros --output /tmp/edog-osmesa-check
```

OSMesa 依赖 `libosmesa6`，已包含在第 3 节。无窗口渲染仍需要 EGL 或 OSMesa 等图形后端。更换后端后重新启动进程；Python viewer 的交互窗口仍需要桌面显示。

### 8.4 Qt 无法连接显示器或加载 `xcb`

确认在桌面终端运行，或使用 `--headless`。可用以下命令检查显示变量：

```bash
printenv DISPLAY WAYLAND_DISPLAY
```

Wayland 下遇到窗口问题时，可尝试在支持 XWayland 的桌面会话中运行：

```bash
cd "$EDOG_PROJECT_ROOT/JiFang_world/02_single_camera"
QT_QPA_PLATFORM=xcb bash run.sh --no-ros
```

本教程统一使用 venv 中通过 requirements 安装的 PyQt5。若仍有插件缺库错误，用 `QT_DEBUG_PLUGINS=1` 启动查看具体缺失库。

### 8.5 中文显示方框

确认安装 `fonts-noto-cjk`，然后刷新字体缓存并重新打开程序：

```bash
fc-cache -f
```

### 8.6 场景文件找不到或脚本不可执行

优先在对应方案目录使用 `bash run.sh`，无需脚本具有执行权限。若希望用 `./run.sh`，可恢复三个实验入口的执行权限：

```bash
cd "$EDOG_PROJECT_ROOT"
chmod +x JiFang_world/01_single_cabinet/run.sh JiFang_world/02_single_camera/run.sh JiFang_world/03_robot_cameras/run.sh
```

使用 ROS launch 时可通过 `scene_dir` 指定方案一的场景路径，见下一节。

## 9. 可选：纳入 ROS 2 工作区

直接运行上述脚本无需构建。ROS 包构建是高级可选路径；colcon 生成的入口取决于构建时解释器，应确认该入口使用的 Python 能导入全部依赖。要在已有 ROS 工作区中使用包，可安装 colcon：

```bash
sudo apt install -y python3-colcon-common-extensions
```

先加载匹配的 ROS 环境及本教程环境文件，然后执行：

```bash
mkdir -p "$HOME/jifang_ros_ws/src"
ln -s "$EDOG_PROJECT_ROOT/JiFang_world/shared/camera_lab" "$HOME/jifang_ros_ws/src/jifang_camera_lab"
cd "$HOME/jifang_ros_ws"
colcon build --symlink-install --packages-select jifang_camera_lab
source install/setup.bash
ros2 launch jifang_camera_lab robot_cameras.launch.py \
  scene_dir:="$EDOG_PROJECT_ROOT/JiFang_world/01_single_cabinet"
```

软链接只需创建一次。此包不会把外部机柜场景复制进 ROS 安装目录，因此应保留完整项目，并传入实际 `scene_dir`。

## 10. 官方资料与验证范围

本教程根据仓库当前代码、启动脚本和以下官方资料编写。官方页面地址列在代码格式中，便于复制：

1. ROS 2 Humble Ubuntu 安装文档：`https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html`；官方源码：`https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Installation/Ubuntu-Install-Debs.rst`
2. ROS 2 Jazzy Ubuntu 安装文档：`https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html`；官方源码：`https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/Installation/Ubuntu-Install-Debs.rst`
3. MuJoCo Python 安装与 viewer：`https://mujoco.readthedocs.io/en/stable/python.html`
4. MuJoCo 3.16.0 Python 包：`https://pypi.org/project/mujoco/3.16.0/`
5. ZXing-C++ 3.1.1 Python 包：`https://pypi.org/project/zxing-cpp/3.1.1/`
6. ROS 官方软件源配置说明源码：`https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/Installation/_Apt-Repositories.rst`
7. MuJoCo 官方发布页：`https://github.com/google-deepmind/mujoco/releases/tag/3.16.0`

系统版本与安装流程已核对官方 `ros2/ros2_documentation` 仓库中的 Humble/Jazzy 安装文档源码；MuJoCo 和 ZXing 的指定版本已核对官方发布页及 PyPI。

本次在项目 `.venv` 中安装了根 requirements 并通过 `pip check`，验证了启动环境选择、单相机和三相机无窗口采集、双排场景离屏渲染、Qt 初始化及 Jazzy 接口导入。没有在一台全新 Ubuntu 系统上执行完整安装；桌面交互和 ROS 实际收发仍需在目标机器按第 7 节验收。
