from dataclasses import asdict, dataclass, fields, replace
from pathlib import Path
import json
import math
import time
import xml.etree.ElementTree as ET
import numpy as np
from .bootstrap import prepare
prepare()
import mujoco
import zxingcpp

NAMESPACE = '/inspection_camera'
OPTICAL_FRAME = 'camera_optical_frame'
FRONT_Y = -0.500
CAMERA_Y = FRONT_Y - 0.600
NATIVE_IN_LINK = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
OPTICAL_IN_LINK = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


@dataclass
class Settings:
    x_m: float = -0.170
    height_m: float = 1.200
    yaw_deg: float = 0.0
    pitch_deg: float = 0.0
    width: int = 1920
    height: int = 1080
    hfov_deg: float = 60.0
    lens_mode: str = 'fov'
    sensor_width_mm: float = 6.4
    sensor_height_mm: float = 3.6
    focal_mm: float = 3.2
    fps: float = 5.0
    detect_enabled: bool = True

    def validate(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if isinstance(value, (float, int)) and not math.isfinite(value):
                raise ValueError(f'{field.name} 必须为有限数值')
        for name, low, high in [('x_m', -1., 1.), ('height_m', .6, 1.8), ('yaw_deg', -90., 90.),
                                ('pitch_deg', -80., 80.), ('hfov_deg', 10., 150.), ('fps', 1., 30.),
                                ('sensor_width_mm', .5, 50.), ('sensor_height_mm', .5, 50.), ('focal_mm', .3, 200.)]:
            if not low <= getattr(self, name) <= high:
                raise ValueError(f'{name} 超出范围 [{low}, {high}]')
        for name in ['width', 'height']:
            if type(getattr(self, name)) is not int or not 240 <= getattr(self, name) <= 4096:
                raise ValueError(f'{name} 必须是 240–4096 的整数')
        if self.lens_mode not in ['fov', 'physical']:
            raise ValueError('lens_mode 必须是 fov 或 physical')
        if type(self.detect_enabled) is not bool:
            raise ValueError('detect_enabled 必须是布尔值')
        hfov, vfov = self.fovs()
        if not (2 < hfov < 160 and 2 < vfov < 160):
            raise ValueError('计算的视场角须在 2°–160°之间，请检查镜头和传感器尺寸')
        return self

    def updated(self, values):
        unknown = set(values) - set(asdict(self))
        if unknown:
            raise ValueError('未知参数: ' + ', '.join(sorted(unknown)))
        return replace(self, **values).validate()

    def intrinsics(self):
        if self.lens_mode == 'physical':
            fx = self.width * self.focal_mm / self.sensor_width_mm
            fy = self.height * self.focal_mm / self.sensor_height_mm
        else:
            fx = fy = self.width / (2 * math.tan(math.radians(self.hfov_deg) / 2))
        return np.array([[fx, 0., self.width / 2], [0., fy, self.height / 2], [0., 0., 1.]])

    def fovs(self):
        k = self.intrinsics()
        return tuple(math.degrees(2 * math.atan(n / (2 * focal))) for n, focal in [(self.width, k[0, 0]), (self.height, k[1, 1])])

    def rotation_link(self):
        yaw, pitch = map(math.radians, [self.yaw_deg, self.pitch_deg])
        right = np.array([math.cos(yaw), math.sin(yaw), 0.])
        forward = np.array([-math.sin(yaw) * math.cos(pitch), math.cos(yaw) * math.cos(pitch), math.sin(pitch)])
        up = np.cross(right, forward)
        return np.column_stack([forward, -right, up])

    def position(self):
        return np.array([self.x_m, CAMERA_Y, self.height_m])


def quaternion(matrix):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(matrix, dtype=float).reshape(9))
    return q  # MuJoCo w,x,y,z


def load_settings(path):
    return Settings().updated(json.loads(Path(path).read_text())) if path else Settings().validate()


class CameraEngine:
    def __init__(self, scene_dir, settings=None):
        self.scene_dir = Path(scene_dir).resolve()
        self.settings = settings or Settings()
        self.settings.validate()
        self.rows = json.loads((self.scene_dir / 'server_heights.json').read_text())['servers']
        self.payload_to_row = {r['qr_payload']: r for r in self.rows}
        self.cumulative = set()
        root = ET.parse(self.scene_dir / 'scene.xml').getroot()
        root.find('compiler').set('texturedir', str(self.scene_dir / 'assets'))
        for include in root.iter('include'):
            include.set('file', str(self.scene_dir / include.get('file')))
        glob = root.find('visual/global')
        glob.set('offwidth', str(max(800, self.settings.width)))
        glob.set('offheight', str(max(400, self.settings.height)))
        world = root.find('worldbody')
        # Group 2 is only visible to the overview, never to the inspection sensor.
        ET.SubElement(world, 'geom', name='camera_horizontal_track', type='capsule', fromto=f'-1 {CAMERA_Y} .008 1 {CAMERA_Y} .008', size='.006', rgba='0.12 .9 .30 1', contype='0', conaffinity='0', group='2')
        rig = ET.SubElement(world, 'body', name='inspection_camera_mount', mocap='true', pos=f'{self.settings.x_m} {CAMERA_Y} {self.settings.height_m}')
        ET.SubElement(rig, 'geom', name='inspection_camera_housing', type='box', pos='-.045 0 0', size='.035 .026 .022', rgba='1 .68 .04 1', contype='0', conaffinity='0', group='2')
        ET.SubElement(rig, 'geom', name='inspection_camera_lens', type='sphere', size='.015', rgba='.03 .06 .09 1', contype='0', conaffinity='0', group='2')
        ET.SubElement(rig, 'camera', name='inspection_camera', quat=' '.join(map(str, quaternion(NATIVE_IN_LINK))), sensorsize='.0064 .0036', resolution=f'{self.settings.width} {self.settings.height}', focal='.0032 .0032')
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        self.data = mujoco.MjData(self.model)
        self.camera_id = self.model.camera('inspection_camera').id
        self.mocap_id = self.model.body('inspection_camera_mount').mocapid[0]
        self.renderer = None
        self.renderer_size = None
        self.render_option = mujoco.MjvOption()
        self.render_option.geomgroup[2] = 0
        self.render_option.geomgroup[3] = 0
        self.apply(self.settings)

    def apply(self, settings):
        settings.validate()
        self.settings = settings
        self.data.mocap_pos[self.mocap_id] = settings.position()
        self.data.mocap_quat[self.mocap_id] = quaternion(settings.rotation_link())
        k = settings.intrinsics()
        # Enforce the same K in MuJoCo's offscreen camera and ROS CameraInfo.
        self.model.cam_sensorsize[self.camera_id] = [settings.width / k[0, 0], settings.height / k[1, 1]]
        self.model.cam_intrinsic[self.camera_id] = [1., 1., 0., 0.]
        self.model.cam_resolution[self.camera_id] = [settings.width, settings.height]
        self.model.cam_fovy[self.camera_id] = settings.fovs()[1]
        mujoco.mj_forward(self.model, self.data)

    def aim(self, index, move=True):
        row = self.rows[index - 1]
        target = np.array(row['qr_front_xyz'])
        height = float(np.clip(target[2], .6, 1.8)) if move else self.settings.height_m
        x = float(np.clip(target[0], -1, 1)) if move else self.settings.x_m
        delta = target - np.array([x, CAMERA_Y, height])
        yaw = math.degrees(math.atan2(-delta[0], delta[1]))
        pitch = math.degrees(math.atan2(delta[2], math.hypot(delta[0], delta[1])))
        return self.settings.updated(dict(x_m=x, height_m=height, yaw_deg=yaw, pitch_deg=pitch))

    def geometry(self):
        s = self.settings
        k = s.intrinsics()
        position = s.position()
        optical = s.rotation_link() @ OPTICAL_IN_LINK
        output = []
        for row in self.rows:
            centre = np.array(row['qr_front_xyz'])
            centre[1] -= .0004
            corners = np.array([centre + [x, 0., z] for x, z in [(-.02, .02), (.02, .02), (.02, -.02), (-.02, -.02)]])
            local = (corners - position) @ optical
            in_front = bool(np.all(local[:, 2] > 0))
            pixels = np.zeros((4, 2))
            if in_front:
                pixels = local[:, :2] / local[:, 2, None] * [k[0, 0], k[1, 1]] + [k[0, 2], k[1, 2]]
            complete = bool(in_front and np.all((pixels[:, 0] >= 0) & (pixels[:, 0] < s.width) & (pixels[:, 1] >= 0) & (pixels[:, 1] < s.height)))
            clear = False
            if complete:
                clear = True
                samples = [centre] + [centre + .8 * (corner - centre) for corner in corners]
                target_geom = self.model.geom(f"qr_front_{row['index']:02}").id
                for point in samples:
                    delta = point - position
                    distance = np.linalg.norm(delta)
                    hit = np.array([-1], dtype=np.int32)
                    actual = mujoco.mj_ray(self.model, self.data, position, delta / distance, np.array([1, 1, 0, 0, 0, 0], dtype=np.uint8), 1, -1, hit)
                    if hit[0] != target_geom or abs(actual - distance) > .003:
                        clear = False
                        break
            edges = [float(np.linalg.norm(pixels[(i + 1) % 4] - pixels[i])) for i in range(4)] if in_front else [0.] * 4
            angle = math.degrees(math.acos(np.clip(abs((centre - position)[1]) / np.linalg.norm(centre - position), 0, 1)))
            output.append(dict(index=row['index'], payload=row['qr_payload'], height_m=row['centre_m'], complete_in_frame=complete, sampled_unoccluded=clear, min_edge_px=round(min(edges), 2), viewing_angle_deg=round(angle, 2), corners_px=np.round(pixels, 2).tolist()))
        return output

    def capture(self):
        started = time.perf_counter()
        s = self.settings
        size = (s.width, s.height)
        if self.renderer_size != size:
            if self.renderer:
                self.renderer.close()
            self.model.vis.global_.offwidth = max(800, s.width)
            self.model.vis.global_.offheight = max(400, s.height)
            self.renderer = mujoco.Renderer(self.model, height=s.height, width=s.width)
            self.renderer_size = size
        self.renderer.update_scene(self.data, camera='inspection_camera', scene_option=self.render_option)
        image = self.renderer.render().copy()
        detections = []
        if s.detect_enabled:
            for code in zxingcpp.read_barcodes(image):
                row = self.payload_to_row.get(code.text)
                points = [code.position.top_left, code.position.top_right, code.position.bottom_right, code.position.bottom_left]
                detections.append(dict(payload=code.text, index=row['index'] if row else None, height_m=row['centre_m'] if row else None, corners_px=[[p.x, p.y] for p in points]))
                if row:
                    self.cumulative.add(row['index'])
        metrics = self.geometry()
        result = dict(settings=asdict(s), camera_y_m=CAMERA_Y, hfov_deg=s.fovs()[0], vfov_deg=s.fovs()[1], intrinsics=s.intrinsics().tolist(), detections=detections, metrics=metrics,
                      decoded_indices=sorted({d['index'] for d in detections if d['index'] is not None}), cumulative_indices=sorted(self.cumulative),
                      missing_indices=sorted(set(range(1, len(self.rows) + 1)) - self.cumulative), processing_seconds=round(time.perf_counter() - started, 4))
        return image, result

    def add_overview_markers(self, scene):
        def line(a, b, color, width=.004):
            if scene.ngeom >= scene.maxgeom:
                return
            geom = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.eye(3).reshape(9), np.array(color))
            mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, width, np.asarray(a), np.asarray(b))
            scene.ngeom += 1
        s = self.settings
        p = s.position()
        rotation = s.rotation_link() @ OPTICAL_IN_LINK
        k = s.intrinsics()
        distance = .6
        points = []
        for x, y in [(0, 0), (s.width, 0), (s.width, s.height), (0, s.height)]:
            endpoint = p + rotation @ np.array([(x - k[0, 2]) / k[0, 0] * distance, (y - k[1, 2]) / k[1, 1] * distance, distance])
            points.append(endpoint)
            line(p, endpoint, [1, .68, .05, .7])
        for i in range(4):
            line(points[i], points[(i + 1) % 4], [1, .68, .05, .7])
        line([s.x_m, CAMERA_Y, .01], p, [.12, .9, .30, .8])

    def close(self):
        if self.renderer:
            self.renderer.close()
            self.renderer = None
