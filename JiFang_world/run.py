#!/usr/bin/env python3
"""Launch the cabinet scene using the official Python passive viewer."""
from pathlib import Path
import argparse
import time


def main():
    parser = argparse.ArgumentParser(description='单机柜 MuJoCo 场景')
    parser.add_argument('--model', type=Path, default=Path(__file__).resolve().parent / 'scene.xml')
    parser.add_argument('--check', action='store_true', help='编译场景并进行 100 步无窗口检查')
    args = parser.parse_args()
    try:
        import mujoco
    except ImportError:
        raise SystemExit('请先激活 MuJoCo Python 环境：source ~/mujoco_work/.venv/bin/activate')
    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    data = mujoco.MjData(model)
    if args.check:
        for _ in range(100):
            mujoco.mj_step(model, data)
        print(f'加载成功：{model.ngeom} 个 geom，{model.nbody} 个 body，仿真时间 {data.time:.3f} 秒')
        if any(data.warning.number):
            raise SystemExit(f'检测到 MuJoCo 警告：{list(data.warning.number)}')
        return
    import mujoco.viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0, -.25, 1.05]
        viewer.cam.distance = 4.0
        viewer.cam.azimuth = 120
        viewer.cam.elevation = -16
        while viewer.is_running():
            start = time.perf_counter()
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(max(0.0, model.opt.timestep - (time.perf_counter() - start)))


if __name__ == '__main__':
    main()
