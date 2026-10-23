# 独立三摄机器狗

`robot.xml` 包含固定站姿 A2-W 尺寸代理、顶部伸缩杆、左右短支架和三台相机。光心距机身侧面 50 mm（机身半宽 160 mm，光心 Y=±210 mm），左右光心高度为 700 mm。顶部默认光心高度为 1325 mm。

这是空间与成像模型；没有可活动腿关节、步态或轮驱动力学。基座通过 mocap 控制，可用于后续位姿、相机或 ROS 联动。XML 本身不发布 ROS 消息；完整三路 ROS 接口由上级目录的 `run.sh` 提供。

## 单独查看

在项目根目录运行：

```bash
.venv/bin/python -m mujoco.viewer --mjcf JiFang_world/03_robot_cameras/standalone_robot/scene.xml
```

`scene.xml` 和 `robot.xml` 可一起复制到其他目录，无需机柜文件、纹理、外部网格或项目 Python 包。预览见 `overview.png`、`top_view.png` 和 `side_view.png`。

## 引用到下一步场景

在目标场景的 worldbody 中引用（按目标文件位置调整相对路径）：

```xml
<mujoco model="next_step">
  <worldbody>
    <include file="standalone_robot/robot.xml"/>
  </worldbody>
</mujoco>
```

基座名 `robot_base`，原点在地面投影中心，+X 朝前、+Y 朝左、+Z 朝上。保留了原平台的 body、geom 和 camera 名称。mocap body 应直接位于 worldbody 下；整机移动使用 mocap 位姿：

```python
import math
import mujoco

model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)
base = model.body("robot_base").mocapid[0]
yaw = math.radians(30)
data.mocap_pos[base] = [1.0, 0.5, 0.0]
data.mocap_quat[base] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
mujoco.mj_forward(model, data)
```

| 接口 | 名称 |
| --- | --- |
| 相机安装 body | `top_camera_mount`、`left_camera_mount`、`right_camera_mount` |
| MuJoCo 相机 | `top_inspection_camera`、`left_inspection_camera`、`right_inspection_camera` |
| 伸缩杆 geom | `telescopic_pole` |
| 顶部云台支柱 geom | `top_gimbal_riser` |

每个安装 body 的局部 +X 为光轴、+Y 为画面左方、+Z 为画面上方；MuJoCo camera 自带坐标变换，其局部 -Z 为光轴。调用 `mj_forward()` 后，可由 `data.cam_xpos` 和 `data.cam_xmat` 读取世界坐标下的实际相机位姿。

相机外壳和尺寸框在 geom group 2。渲染传感器图像时将 `mujoco.MjvOption().geomgroup[2]` 设为 0，避免从相机自身外壳内成像；机器人主体、支架、杆位于 group 1，保留真实遮挡。总览可显示 group 2。

## 重新导出

在 `03_robot_cameras/` 下执行：

```bash
bash run.sh --no-ros --export-robot standalone_robot
# 指定配置中的杆高、三路姿态、分辨率及视场角
bash run.sh --no-ros --config /完整路径/settings.json --export-robot standalone_robot
```

导出只更新两个 XML，基座归零并朝向 +X；预览图片不会自动重建。动态调参可复用 `inspection_camera.robot_platform` 中的 `RobotSettings`、`robot_body_xml()` 和 `RobotCameraEngine.apply_platform()`；完整评估引擎还依赖机柜场景。
