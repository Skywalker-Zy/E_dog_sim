from dataclasses import asdict
import json
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from .core import NAMESPACE, OPTICAL_FRAME, CAMERA_Y, OPTICAL_IN_LINK, Settings, quaternion, mujoco


def put_quaternion(destination, value):
    destination.w, destination.x, destination.y, destination.z = map(float, value)


class CameraNode(Node):
    def __init__(self, engine):
        super().__init__('jifang_inspection_camera')
        self.engine = engine
        self.command_locked = False
        self.internal_update = False
        self.on_settings_changed = lambda: None
        self.on_coverage_reset = lambda: None
        self.on_scan_requested = None
        for name, value in asdict(engine.settings).items():
            self.declare_parameter(name, value)
        initial = {name: self.get_parameter(name).value for name in asdict(engine.settings)}
        engine.apply(Settings().updated(initial))
        self.add_on_set_parameters_callback(self._accept_parameters)
        self.image_pub = self.create_publisher(Image, NAMESPACE + '/image_raw', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, NAMESPACE + '/camera_info', qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(PoseStamped, NAMESPACE + '/pose', 10)
        self.detection_pub = self.create_publisher(String, NAMESPACE + '/qr_detections', 10)
        self.create_subscription(PoseStamped, NAMESPACE + '/target_pose', self._target, 10)
        self.create_service(Trigger, NAMESPACE + '/reset_coverage', self._reset)
        self.create_service(Trigger, NAMESPACE + '/start_scan', self._scan)
        self.tf_pub = TransformBroadcaster(self)
        self.static_tf_pub = StaticTransformBroadcaster(self)
        optical = TransformStamped()
        optical.header.stamp = self.get_clock().now().to_msg()
        optical.header.frame_id = 'camera_link'
        optical.child_frame_id = OPTICAL_FRAME
        put_quaternion(optical.transform.rotation, quaternion(OPTICAL_IN_LINK))
        self.static_tf_pub.sendTransform(optical)
        self.get_logger().info('相机节点启动；图像、内参、位姿、TF 和二维码识别接口已就绪。')

    def _accept_parameters(self, parameters):
        values = {p.name: p.value for p in parameters if p.name in asdict(self.engine.settings)}
        if self.command_locked and not self.internal_update and values:
            return SetParametersResult(successful=False, reason='自动扫描进行中，请先停止扫描')
        if not values:
            return SetParametersResult(successful=True)
        try:
            candidate = self.engine.settings.updated(values)
            self.engine.apply(candidate)
            self.on_settings_changed()
            return SetParametersResult(successful=True)
        except (ValueError, TypeError) as error:
            return SetParametersResult(successful=False, reason=str(error))

    def set_settings(self, values, internal=False):
        self.internal_update = internal
        try:
            result = self.set_parameters_atomically([Parameter(name, value=value) for name, value in values.items()])
        finally:
            self.internal_update = False
        if not result.successful:
            raise ValueError(result.reason)

    def _target(self, message):
        try:
            if message.header.frame_id != 'world':
                raise ValueError('target_pose.header.frame_id 必须为 world')
            position, rotation = message.pose.position, message.pose.orientation
            if not all(math.isfinite(v) for v in [position.x, position.y, position.z, rotation.w, rotation.x, rotation.y, rotation.z]):
                raise ValueError('位姿必须为有限数值')
            if abs(position.y - CAMERA_Y) > .001:
                raise ValueError(f'相机必须位于固定轨道 y={CAMERA_Y:.3f} m')
            q = np.array([rotation.w, rotation.x, rotation.y, rotation.z])
            if abs(np.linalg.norm(q) - 1) > .001:
                raise ValueError('姿态四元数必须归一化')
            matrix = np.zeros(9)
            mujoco.mju_quat2Mat(matrix, q)
            matrix = matrix.reshape(3, 3)
            forward = matrix[:, 0]
            yaw = math.degrees(math.atan2(-forward[0], forward[1]))
            pitch = math.degrees(math.atan2(forward[2], math.hypot(forward[0], forward[1])))
            candidate = self.engine.settings.updated(dict(x_m=float(position.x), height_m=float(position.z), yaw_deg=yaw, pitch_deg=pitch))
            if not np.allclose(matrix, candidate.rotation_link(), atol=.001):
                raise ValueError('当前相机仅支持 yaw/pitch，不支持 roll')
            self.set_settings({k: getattr(candidate, k) for k in ['x_m', 'height_m', 'yaw_deg', 'pitch_deg']})
        except (ValueError, TypeError) as error:
            self.get_logger().warning('拒绝目标位姿: ' + str(error))

    def _reset(self, request, response):
        self.engine.cumulative.clear()
        self.on_coverage_reset()
        response.success = True
        response.message = '累计识别统计已清空'
        return response

    def _scan(self, request, response):
        if self.on_scan_requested is None:
            response.success = False
            response.message = '当前运行模式不接受扫描请求'
        else:
            response.success, response.message = self.on_scan_requested()
        return response

    def publish(self, image, result):
        # An asynchronous render may complete after a newer parameter update.
        # Image, calibration, pose and TF must all use the captured settings.
        captured = Settings().updated(result['settings'])
        stamp = self.get_clock().now().to_msg()
        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = OPTICAL_FRAME
        message.height, message.width = image.shape[:2]
        message.encoding = 'rgb8'
        message.is_bigendian = 0
        message.step = message.width * 3
        if self.image_pub.get_subscription_count():
            message.data = image.tobytes()
            self.image_pub.publish(message)
        k = captured.intrinsics()
        info = CameraInfo()
        info.header = message.header
        info.width, info.height = message.width, message.height
        info.distortion_model = 'plumb_bob'
        info.d = [0.] * 5
        info.k = k.reshape(-1).tolist()
        info.r = np.eye(3).reshape(-1).tolist()
        info.p = np.column_stack([k, np.zeros(3)]).reshape(-1).tolist()
        if self.info_pub.get_subscription_count():
            self.info_pub.publish(info)
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'world'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, captured.position())
        q = quaternion(captured.rotation_link())
        put_quaternion(pose.pose.orientation, q)
        self.pose_pub.publish(pose)
        mount = TransformStamped()
        mount.header = pose.header
        mount.child_frame_id = 'camera_mount'
        mount.transform.translation.x, mount.transform.translation.y, mount.transform.translation.z = map(float, captured.position())
        mount.transform.rotation.w = 1.
        link = TransformStamped()
        link.header.stamp = stamp
        link.header.frame_id = 'camera_mount'
        link.child_frame_id = 'camera_link'
        put_quaternion(link.transform.rotation, q)
        self.tf_pub.sendTransform([mount, link])
        self.detection_pub.publish(String(data=json.dumps(dict(stamp=dict(sec=stamp.sec, nanosec=stamp.nanosec), frame_id=OPTICAL_FRAME, **result), ensure_ascii=False)))
