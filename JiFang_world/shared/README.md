# 共用代码

- `camera_lab/inspection_camera/`：单相机和机器狗三相机共用的成像、解码、界面、后台线程和 ROS 接口。
- `camera_lab/vendor/`：历史扫码库及许可证；正常启动优先使用 venv 中安装的版本。
- `camera_lab/package.xml`、`setup.py`、`launch/`：ROS 2 包与 launch 入口。构建时保留完整源项目，配置文件来自 `../../02_single_camera/config/` 和 `../../03_robot_cameras/config/`。
- `scripts/venv.sh`：三个方案共用的 venv 发现及 ROS 环境加载函数。

请从各方案的 `run.sh` 启动。这里不保存实验图像和运行日志，两个相机方案会分别写入自身的 `experiments/` 和 `runtime/`。
