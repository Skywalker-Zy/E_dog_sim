"""Pose control and full-resolution camera capture for the combined room."""
from pathlib import Path
from dataclasses import asdict
import math
import sys
import json
import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent / 'two_rows_20_racks'))
sys.path.insert(0, str(ROOT.parent / 'shared' / 'camera_lab'))
from runtime import RoomSimulation, mujoco
from inspection_camera.robot_platform import RobotSettings, CAMERAS, HEIGHT, quaternion


def default_controls():
    settings = asdict(RobotSettings(robot_x_m=.9, robot_y_m=0.))
    return {key: value for key, value in settings.items()
            if key.startswith('robot_') or key.endswith(('_yaw_deg', '_pitch_deg')) or key == 'pole_length_m'}


class RobotRoomSimulation(RoomSimulation):
    def __init__(self, directory=ROOT):
        super().__init__(directory)
        self.sensor_renderer = None
        self.controls = default_controls()
        equipment = json.loads((ROOT.parent / 'two_rows_20_racks/equipment.json').read_text())
        self.qr_lookup = {item['payload']: dict(item, rack_id=row['rack_id'])
                          for row in equipment for item in row['qr_items']}
        self.apply_controls(self.controls)

    def apply_controls(self, controls):
        values = default_controls()
        if set(controls) - set(values):
            raise ValueError('未知机器人控制参数')
        values.update(controls)
        limits = {'robot_x_m': (-2., 9.), 'robot_y_m': (-3., 3.),
                  'robot_yaw_deg': (-180., 180.), 'pole_length_m': (.6, 1.2)}
        for name in CAMERAS:
            limits[name + '_yaw_deg'] = (-180., 180.)
            limits[name + '_pitch_deg'] = (-80., 80.)
        for key, value in values.items():
            low, high = limits[key]
            if not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f'{key} 范围 [{low}, {high}]')
        settings = RobotSettings(**values)
        base = self.model.body('robot_base').mocapid[0]
        self.data.mocap_pos[base] = [settings.robot_x_m, settings.robot_y_m, 0.]
        self.data.mocap_quat[base] = quaternion(settings.base_rotation())
        pole = self.model.geom('telescopic_pole').id
        self.model.geom_pos[pole] = [0, 0, HEIGHT + settings.pole_length_m / 2]
        self.model.geom_size[pole, 1] = settings.pole_length_m / 2
        self.model.geom_pos[self.model.geom('top_gimbal_riser').id] = [0, 0, HEIGHT + settings.pole_length_m + .025]
        for name in CAMERAS:
            body = self.model.body(name + '_camera_mount').id
            self.model.body_pos[body] = settings.camera_local_position(name)
            self.model.body_quat[body] = quaternion(settings.camera_local_rotation(name))
        self.controls = values
        mujoco.mj_forward(self.model, self.data)

    def render(self, camera='aisle', width=1200, height=750, free_camera=None):
        self.option.geomgroup[2] = not camera.endswith('_inspection_camera')
        return super().render(camera, width, height, free_camera)

    def overview(self, view, angles):
        camera = None
        if view in ('follow', 'free'):
            camera = mujoco.MjvCamera()
            camera.lookat[:] = ([self.controls['robot_x_m'], self.controls['robot_y_m'], .75]
                                if view == 'follow' else [3., 0., 1.1])
            camera.azimuth, camera.elevation, camera.distance = angles
        return self.render(view if camera is None else 'aisle', width=1000, height=650, free_camera=camera)

    def capture(self, name, detect=False):
        if name not in CAMERAS:
            raise ValueError('未知相机')
        if self.sensor_renderer is None:
            self.model.vis.global_.offwidth = max(1920, self.model.vis.global_.offwidth)
            self.model.vis.global_.offheight = max(1080, self.model.vis.global_.offheight)
            self.sensor_renderer = mujoco.Renderer(self.model, width=1920, height=1080, max_geom=12000)
        option = mujoco.MjvOption()
        option.geomgroup[2] = option.geomgroup[3] = 0
        self.sensor_renderer.update_scene(self.data, camera=name + '_inspection_camera', scene_option=option)
        self.sensor_renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
        image = self.sensor_renderer.render().copy()
        detections = []
        if detect:
            import zxingcpp
            for code in zxingcpp.read_barcodes(image):
                if code.format != zxingcpp.BarcodeFormat.QRCode:
                    continue
                p = code.position
                item = self.qr_lookup.get(code.text, {})
                detections.append(dict(payload=code.text, rack_id=item.get('rack_id'), height_m=item.get('height_m'),
                                       corners_px=[[v.x, v.y] for v in (p.top_left, p.top_right, p.bottom_right, p.bottom_left)]))
        return image, detections

    def reset(self):
        super().reset()
        self.apply_controls(default_controls())

    def close(self):
        if self.sensor_renderer is not None:
            self.sensor_renderer.close()
            self.sensor_renderer = None
        super().close()
