# 三摄机器狗与双列机柜联合场景

将 `03_robot_cameras/standalone_robot/robot.xml` 中的三摄机器狗与 `two_rows_20_racks/scene.xml` 中的两列机柜合并。目录名按需求保留为 `04_robot_envirments`。

## 启动

在项目根目录执行：

```bash
bash JiFang_world/04_robot_envirments/run.sh
```

界面左侧为控制区，中间为场景大图，右侧竖排放置相机选择、画面和二维码识别结果。默认场景视角跟随机器狗，仍可切换通道、俯视、门口、总览或自由观察；跟随与自由视角支持拖动旋转和滚轮缩放。

- **门**：仅保留一个按钮，点击“开门”打开双扇门，按钮随即变为“关门”；再次点击关闭。下方显示实际角度。
- **相机画面**：小窗下拉切换顶部、左侧、右侧相机；勾选“识别二维码”后，按该相机选定的原始分辨率解码，再缩小显示。识别框画在图像上，下方列出二维码内容、机柜编号和高度。取消勾选立即清空结果并跳过解码。
- **相机转向**：三路均有独立水平 yaw（±180°）和俯仰 pitch（±80°）输入框。yaw 相对机身定义，0°朝左侧、±180°朝右侧；pitch 正值抬头、负值低头。
- **视场角与分辨率**：三台相机独立调整水平视场角（10–150°）和分辨率（720p / 1080p / 2560×1440 / 4K）。默认均为 **3840×2160（4K UHD）**；顶部水平视场角默认 **115°**，左右两侧各 **60°**。切换相机保留各自设置，右侧同步显示当前分辨率和视场角。
- **顶部升降**：伸缩杆长度 0.600–1.200 m，对应顶部光心高度 1.325–1.925 m；界面同步显示光心高度。
- **机器狗移动**：前进、后退、左移、右移均相对当前机身朝向；左右转按钮原地改变 yaw。平移及旋转步长可调，按住按钮可连续操作；也可直接输入世界 X/Y 和 yaw。
- **复位与保存**：“复位机器人与相机”恢复初始位姿、六个相机角度、杆高以及三路视场角和 4K 分辨率，保持门的当前目标；“保存画面与当前配置”保存场景图、所选相机原图及与该批图像对应的位姿、三路视场角/分辨率、识别结果等 JSON。

MuJoCo 渲染与识别在后台线程完成；静止时缓存图像和识别结果。三路传感器视图隐藏相机自身外壳，保留机身和支架遮挡。

## 布局与接口

- 两列各 10 个机柜，正面净距 1.20 m，沿用原有设备、二维码和双扇铰链门。
- 机器狗基座位于 `(0.9, 0, 0)`，处于 L02/R02 柜对之间，朝向 +X（通道末端的门）；+Y 为左侧机柜。
- 左右相机光心距机身侧面 50 mm，局部位置为 `(0, ±0.210, 0.700)` m；顶部默认光心高度 1.325 m。
- 包含旋转扫掠空间的横向包络为 0.520 m，居中时距每侧柜体前框保守净距 0.340 m。
- 机器狗为固定站姿 mocap 模型，位姿入口为 `robot_base`；相机名称为 `top_inspection_camera`、`left_inspection_camera`、`right_inspection_camera`。保留了第三阶段的 body/geom/camera 名称。
- 门接口为 `door_left_hinge` / `door_right_hinge` 和 `door_left_motor` / `door_right_motor`。

机器狗直接通过 mocap 更新位姿，不包含步态、轮驱动或机器人碰撞动力学；手动平移可以穿过柜体。当前入口不发布 ROS 消息。

后续程序可直接加载联合场景，并通过 `data.mocap_pos` / `data.mocap_quat` 控制整机。使用绝对路径加载，以兼容嵌套 include：

```python
from pathlib import Path
import mujoco

scene = Path('JiFang_world/04_robot_envirments/scene.xml').resolve()
model = mujoco.MjModel.from_xml_path(str(scene))
data = mujoco.MjData(model)
base = model.body('robot_base').mocapid[0]
data.mocap_pos[base] = [1.5, 0, 0]
mujoco.mj_forward(model, data)
```

## 文件与复用

- `scene.xml`：联合入口，引用原双列场景和本目录的 `robot.xml`。
- `robot.xml`：三摄机器狗快照，设置了通道初始位置。
- `build_scene.py`：重新读取第三阶段独立模型，生成联合入口、机器狗快照和布局摘要。
- `viewer.py` / `run.sh`：联合控制台与启动入口。
- `camera_config.py`：界面、仿真和场景导出共用的视场角、分辨率默认值与投影换算。
- `simulation.py`：复用双列场景物理运行代码，增加机器人位姿、三路相机转向、顶部升降与二维码识别。
- `validate_scene.py` / `validate_gui.py`：模型与离屏界面验证脚本。
- `layout.json`：布局摘要与机器狗初始位置。
- `previews/`、`verification/`：联合场景与三路相机截图、验证结果。

本目录复用相邻目录的资源，请保留完整 `JiFang_world` 目录。机器狗模型更新后，在项目根目录执行：

```bash
.venv/bin/python JiFang_world/04_robot_envirments/build_scene.py
```

柜体纹理、二维码及设备位置真值继续使用 `../two_rows_20_racks/assets/`、`equipment.json` 和 `equipment.csv`，不复制数百张纹理。

验证命令（场景、位姿、角度、升降、真实二维码解码及离屏界面操作）：

```bash
MUJOCO_GL=egl .venv/bin/python JiFang_world/04_robot_envirments/validate_scene.py
.venv/bin/python JiFang_world/04_robot_envirments/validate_gui.py
```

已通过上述检查；默认左相机识别出 L02 柜的 10 个二维码。离屏界面测试覆盖相对机身平移、左右旋转、六个相机角度、顶部升降、三路切换、独立视场角和分辨率、4K 默认值与复位、右侧竖排布局、开闭识别、快速修改参数、画面配置一致性及单按钮门控。结果见 `verification/`，当前界面截图见 `previews/gui.png`。
