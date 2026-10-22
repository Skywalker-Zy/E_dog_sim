# 单机柜 MuJoCo 机房场景

按提供的机房照片和尺寸标注制作的简易场景，只有一个机柜。原生 MJCF/XML，无需 Blender、额外模型下载或第三方插件。本机使用 MuJoCo 3.16.0 验证。

## 安装 Python 依赖与启动环境

在项目根目录执行（不需要 sudo）：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

所有 `.sh` 入口统一选择 venv：`EDOG_VENV` → 当前激活环境 → 向上查找最近的 `.venv`。独立 simulate 入口也会加载该环境，但仍需单独安装 MuJoCo 独立程序。完整步骤见根目录的 `Ubuntu安装与依赖指南.md`。

## 直接启动：独立 simulate 程序

```bash
cd /home/zy/E_dog_sim/JiFang_world
./run.sh
```

或者直接加载 XML：

```bash
~/mujoco_work/mujoco-3.16.0/bin/simulate /home/zy/E_dog_sim/JiFang_world/scene.xml
```

也可以把 `scene.xml` 拖入已经打开的 simulate 窗口。

如果只想看机柜与地面，不显示背景墙和门：

```bash
./run.sh cabinet_only.xml
```

`run.sh` 加载 venv 后，依次查找 `$MUJOCO_HOME/bin/simulate`、PATH 中的 simulate、`~/mujuco_work/mujoco-3.16.0/bin/simulate` 和 `~/mujoco_work/mujoco-3.16.0/bin/simulate`。

## Python 启动

```bash
source ~/E_dog_sim/.venv/bin/activate
python /home/zy/E_dog_sim/JiFang_world/run.py
```

无窗口检查：

```bash
python /home/zy/E_dog_sim/JiFang_world/run.py --check
```

MuJoCo 界面中：空格暂停/继续，左键拖动旋转视角，滚轮缩放，右键拖动平移，F1 查看帮助。场景只有固定物体，所以运行后机柜本身保持静止。Python 启动脚本预设了合适的观察视角；XML 还提供 `overview`、`front`、`inspection`、`qr_detail_front`、`front_panels` 五台固定摄像机，后两台分别用于查看正面二维码和正面机箱面板。

## 尺寸与照片的对应关系

长度单位为米，Z 轴向上，机柜前面朝 -Y。机柜底面中心为 `(0, 0, 0)`，地面高度为 `z=0`。

| 项目 | 尺寸 | 依据 |
| --- | --- | --- |
| 总高 | 2.177 m | 标注图 217.7 cm |
| 宽 | 0.600 m | 照片未标出，建模假设 |
| 深 | 1.000 m | 照片未标出，建模假设 |
| 底座 | 0.100 m | 标注图约 10 cm |
| 底部服务器区 | z=0.100–0.233 m | 约 3U，共 3 条面板 |
| 上方服务器区 | 8 组，每组约 4U | 每组 4 条面板，共 32 条 |
| 组间隔板 | 0.040 m | 8 块，每个上部服务器区下方一块 |
| 顶部理线区 | z=1.977–2.177 m | 约 20 cm |
| 格栅区域 | 宽 2.4 m、深 3.0 m | 背景示意，不是照片实测 |

从下到上，服务器区上边缘严格使用标注高度：

```text
0.233, 0.451, 0.669, 0.887, 1.105, 1.325, 1.541, 1.759, 1.977 m
```

标注中 4U≈17.8 cm 与个别相邻高度差存在约 2 mm 的舍入差异。本模型优先保留标注的累计高度，以及 4 cm 隔板厚度。因此八组服务器区的实际高度为：17.8、17.8、17.8、17.8、18.0、17.6、17.8、17.8 cm。底部 3U 按标注为 13.3 cm。没有把照片中的透视比例当作真实测量。

外观包括银灰框架、黑色服务器面板、散热孔纹理、把手、指示灯、黄色端口标签、顶部蓝色理线和下部蓝色线束。背景包括格栅地板、两面墙、固定门和灯条；没有额外机柜。

`dimensions.svg` 是可用浏览器打开的正面/侧面尺寸图；`previews/` 内是由本机 MuJoCo 渲染的预览图。

## 每个 1U 机箱的高度二维码

底部 3U 区分为 3 个 1U，上方八组 4U 分为 32 个 1U，共 **35 个机箱**。每个机箱只在**正面靠左**放置一张二维码，共 **35 张**；机柜左右侧面均不放二维码。

- 单张外形 **4 cm × 4 cm**，包含 4 模块宽的白色留边。
- 编码内容是该 1U 机箱**中心距地面的高度**，单位为**米**，保留 6 位小数；例如最底部机箱扫码得到 `0.122167`。
- 二维码中心为 `x=-0.170 m, y=-0.488 m`，z 与机箱中心高度一致，朝向柜前（-Y）。
- 二维码右边有相同数值和 `m` 单位的人眼可读标签。下部蓝色线束沿左边框布置，避免遮挡二维码。
- `server_heights.csv` 和 `server_heights.json` 记录每个机箱的下沿、中心、上沿高度、二维码内容及位置，可以直接作为视觉定位的真值。
- 二维码为实际生成的 QR Code，不是装饰图。`assets/qr_01.png` 到 `assets/qr_35.png` 是对应原图；预览目录包含 MuJoCo 渲染后的近距离二维码图。
- 验证时解码全部 35 张原图及全部 35 个正面二维码的近距离仿真渲染图。远景缩略图像素不足时不保证能识别，后续相机应靠近并朝向机箱正面。

这里“对应的高度”明确按机箱中心定义；若要改成机箱下沿高度，可修改生成器 `server_rows()` 中的 `qr_payload` 计算，再重新生成。

## 碰撞与机器狗接入

- 机柜固定在世界坐标系中，不含自由关节，不会翻倒或漂移。
- 机柜主体碰撞采用一个宽 0.60 m、深 1.00 m、高 2.177 m 的实心包围盒，名称为 `cabinet_collision`。
- 地面与背景墙参与碰撞。格栅是视觉纹理，脚底接触的是连续平面，没有真实孔洞。
- 服务器外观、指示灯、线缆、灯条等设为 `contype=0`、`conaffinity=0`，不产生接触，减少后续机器狗仿真的接触开销。
- 机柜不包含可抽拉服务器、可开关柜门、电缆柔性或热流仿真；背景门也固定关闭。
- 此版本不包含机器狗模型或步态控制器。后续可以向 `scene.xml` 的 `worldbody` 加入机器狗，并将初始位置设置在柜前，例如 `(0, -1.5, 合适的机身高度)`。具体高度和控制器应使用对应机器狗项目的设置。
- 提供站点 `cabinet_front_target`，位置 `(0, -0.52, 1.105)`，便于设置巡检目标；`cabinet_top` 标识机柜顶面。两站点默认隐藏。
- 视觉模型放在 geom group 1；简化机柜碰撞盒放在 group 3 并设置透明。没有将视觉用的孔、槽或线缆建成细碎碰撞体。

如果只复用 `cabinet.xml`，需要在父模型中包含 `rack_assets.xml`，定义 `decoration` 默认类，并正确配置资产路径。最省事的方式是以已有的 `scene.xml` 或 `cabinet_only.xml` 为起点接入机器人。

## 文件

| 文件 | 用途 |
| --- | --- |
| `scene.xml` | 完整机房场景入口 |
| `cabinet_only.xml` | 单机柜与地面入口 |
| `cabinet.xml` | 可复用机柜 body，包含外观与碰撞 |
| `rack_assets.xml` | 材质和纹理声明 |
| `assets/floor_grate.png` | 程序生成的格栅纹理 |
| `assets/server_face.png` | 程序生成的服务器散热孔纹理 |
| `run.sh` | 使用本机独立 simulate 打开场景 |
| `run.py` | Python viewer 和无窗口检查 |
| `build_scene.py` | 模型生成源码，内附二维码库，无需安装生成依赖 |
| `server_heights.csv` / `.json` | 35 个 1U 机箱及二维码高度对照 |
| `vendor/` | BSD 许可的 python-qrcode 8.2 与许可证 |
| `dimensions.svg` | 尺寸图 |
| `previews/overview.png` | 场景整体预览 |
| `previews/front.png` | 正面预览 |
| `previews/qr_detail_front.png` / `front_panels.png` | 正面二维码与面板渲染预览 |
| `validation.json` | 本机加载、渲染和碰撞验证记录 |

编辑现有 XML 可以直接调整外观。若修改 `build_scene.py` 并执行：

```bash
python3 /home/zy/E_dog_sim/JiFang_world/build_scene.py
```

它会重新生成 XML、尺寸图与纹理，覆盖这些生成文件；不会自动更新预览图和验证记录。宽度和深度还影响框架、面板和线缆位置，修改时请一起检查生成代码中的几何尺寸。


## 可移动巡检相机与 ROS 2

新增相机实验平台，运行：

```bash
cd /home/zy/E_dog_sim/JiFang_world
./run_camera.sh
```

提供中文滑块界面、0.60 m 固定距离轨道、0.60–1.80 m 升降、yaw/pitch、镜头/视场角/分辨率设置、实时扫码、自动巡检、ROS 2 图像/内参/TF 和位姿控制。详细说明见 `camera_lab/README.md`，原始基线数据见 `camera_lab/baseline_experiments/`。


## 机器狗尺寸模型与顶部/双侧相机

```bash
cd /home/zy/E_dog_sim/JiFang_world
./run_robot_cameras.sh
```

采用宇树 A2-W 官方站姿尺寸 900×440×625 mm 的简化模型；整机保持站立、可直接平移和转向，无关节或步态。保留顶部伸缩杆相机，并增加左右侧各一台 0.70 m 高、独立 yaw/pitch 的相机，包含真实机身遮挡、三路图像和 ROS 2 接口。

详细尺寸来源、旋转间隙、使用方法和验证记录见 `camera_lab/ROBOT_PLATFORM.md`。静态场景入口为 `robot_scene.xml`。
