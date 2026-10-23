"""Standalone MuJoCo runtime; no reference to other project scene files."""
from pathlib import Path
import math
import os
import time
import numpy as np

ROOT = Path(__file__).resolve().parent
os.environ.setdefault('MUJOCO_GL', 'egl')
import mujoco


class RoomSimulation:
    def __init__(self, directory=ROOT):
        self.directory = Path(directory).resolve()
        self.model = mujoco.MjModel.from_xml_path(str(self.directory / 'scene.xml'))
        self.data = mujoco.MjData(self.model)
        self.joints = [self.model.joint('door_' + side + '_hinge').id for side in ['left', 'right']]
        self.qpos = [int(self.model.jnt_qposadr[j]) for j in self.joints]
        self.actuators = [self.model.actuator('door_' + side + '_motor').id for side in ['left', 'right']]
        self.renderer = None
        self.renderer_size = None
        self.option = mujoco.MjvOption()
        self.option.geomgroup[3] = 0
        self.last_wall_time = time.perf_counter()
        mujoco.mj_forward(self.model, self.data)

    def set_targets(self, degrees):
        if len(degrees) != 2 or any(not math.isfinite(value) or not 0 <= value <= 100 for value in degrees):
            raise ValueError('门角度必须是 0–100°之间的两个有限数值')
        self.data.ctrl[self.actuators] = np.radians(degrees)

    def angles(self):
        return np.degrees(self.data.qpos[self.qpos]).tolist()

    def step(self, seconds=None):
        now = time.perf_counter()
        elapsed = min(max(now - self.last_wall_time, .002), .10) if seconds is None else seconds
        self.last_wall_time = now
        for _ in range(max(1, round(elapsed / self.model.opt.timestep))):
            mujoco.mj_step(self.model, self.data)

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self.last_wall_time = time.perf_counter()

    def render(self, camera='aisle', width=1200, height=750, free_camera=None):
        if self.renderer_size != (width, height):
            if self.renderer:
                self.renderer.close()
            self.model.vis.global_.offwidth = max(800, width)
            self.model.vis.global_.offheight = max(600, height)
            self.renderer = mujoco.Renderer(self.model, width=width, height=height, max_geom=12000)
            self.renderer_size = (width, height)
        self.renderer.update_scene(self.data, camera=free_camera if free_camera is not None else camera, scene_option=self.option)
        # End-to-end room gets expensive in software with many shadow lights.
        # Sensor-camera reuse can explicitly enable shadows if desired.
        self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
        return self.renderer.render().copy()

    def close(self):
        if self.renderer:
            self.renderer.close()
            self.renderer = None
