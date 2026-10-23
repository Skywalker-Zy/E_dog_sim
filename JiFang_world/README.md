# 机房巡检仿真：实验场景

四个阶段分别保存在以下目录，均通过各自的 `run.sh` 启动：

| 目录 | 内容 | 启动命令（在 JiFang_world 下执行） |
| --- | --- | --- |
| [01_single_cabinet](01_single_cabinet/README.md) | 单机柜、35 张二维码及高度真值，查看场景和碰撞模型 | `bash 01_single_cabinet/run.sh` |
| [02_single_camera](02_single_camera/README.md) | 可升降单相机、镜头/视角调参、扫码与覆盖率实验 | `bash 02_single_camera/run.sh --no-ros` |
| [03_robot_cameras](03_robot_cameras/README.md) | 简化机器狗、顶部升降相机与左右侧相机、机身遮挡评估 | `bash 03_robot_cameras/run.sh --no-ros` |
| [04_robot_envirments](04_robot_envirments/README.md) | 三摄机器狗与双列 20 机柜联合场景、手动门与相机视图 | `bash 04_robot_envirments/run.sh` |

```text
JiFang_world/
├── 01_single_cabinet/     # 场景、纹理、二维码、生成器、真值和预览
├── 02_single_camera/      # 单相机入口、配置、基线实验、安装高度论证和验证记录
├── 03_robot_cameras/      # 三相机入口、配置、静态模型、历史实验和验证记录
├── 04_robot_envirments/  # 三摄机器狗与双列机柜的联合场景
├── shared/
│   ├── camera_lab/       # 两种相机方案共用的 Python/ROS 代码和扫码库
│   └── scripts/          # venv 选择和 ROS 环境加载工具
├── two_rows_20_racks/    # 独立双排场景，供第四阶段复用
├── 参考照片/
└── 项目目标与实物平台需求.md
```

在仓库根目录安装 Python 依赖：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

脚本依次选择 `EDOG_VENV`、已激活的 venv、从脚本目录向上查找到的最近 `.venv`。方案二、三首次使用 ROS 前还需安装对应的 ROS 2；不接 ROS 时加 `--no-ros`。第四阶段为场景查看入口，无需 ROS。方案一的 `run.sh` 使用独立 MuJoCo simulate，需按[安装教程](../Ubuntu安装与依赖指南.md)配置该程序，也可在激活 venv 后运行 `python 01_single_cabinet/run.py`。

方案二和方案三共同读取方案一的机柜模型，共用 `shared/camera_lab` 中的成像代码；保留完整 `JiFang_world` 目录即可运行。新结果分别保存到方案二、三各自的 `experiments/`；整理前的原始结果及采集配置完整保留，历史 JSON 中的原始路径不重写。

原来的顶层 `run.sh`、`run_camera.sh`、`run_robot_cameras.sh` 已分别移到三个方案目录，统一命名为 `run.sh`。原 `camera_lab/` 中的入口、配置和实验资料已按方案分开，公共源码移到 `shared/camera_lab/`。
