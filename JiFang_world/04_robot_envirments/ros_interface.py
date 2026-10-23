"""ROS 2 Jazzy interfaces; desired state is independent of the render thread."""
from dataclasses import fields
import json
import math
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, Float64, UInt32MultiArray, String
from simulation import default_controls, validate_controls, CAMERAS, RobotSettings, quaternion, mujoco
from inspection_camera.robot_platform import rz

PREFIX = '/robot_environment'
STATE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
IMAGE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


def settings_for(controls):
    return RobotSettings(**{f.name: controls[f.name] for f in fields(RobotSettings) if f.name in controls})


def put_quaternion(target, values):
    target.w, target.x, target.y, target.z = map(float, values)


def pose_message(stamp, frame, position, rotation):
    msg = PoseStamped()
    msg.header.stamp, msg.header.frame_id = stamp, frame
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, position)
    put_quaternion(msg.pose.orientation, quaternion(rotation))
    return msg


def rotation_from_pose(msg):
    q = msg.pose.orientation
    values = np.array([q.w, q.x, q.y, q.z])
    p = msg.pose.position
    if not np.isfinite([p.x, p.y, p.z, *values]).all() or abs(np.linalg.norm(values) - 1) > 1e-3:
        raise ValueError('位姿必须有限，四元数须归一化')
    matrix = np.zeros(9)
    mujoco.mju_quat2Mat(matrix, values / np.linalg.norm(values))
    return matrix.reshape(3, 3)


class EnvironmentNode(Node):
    def __init__(self, context=None, ros_args=None):
        super().__init__('jifang_robot_environment', context=context, cli_args=ros_args)
        self.declare_parameter('velocity_timeout_s', .5)
        self.declare_parameter('state_rate_hz', 20.)
        self.declare_parameter('image_rate_hz', 2.)
        self.timeout = float(self.get_parameter('velocity_timeout_s').value)
        state_rate = float(self.get_parameter('state_rate_hz').value)
        self.image_rate = float(self.get_parameter('image_rate_hz').value)
        if not all(math.isfinite(v) for v in [self.timeout, state_rate, self.image_rate]) or not (.05 <= self.timeout <= 5 and 1 <= state_rate <= 100 and .1 <= self.image_rate <= 15):
            raise ValueError('timeout 范围 .05–5 s，state_rate 1–100 Hz，image_rate .1–15 Hz')
        # Startup-only scheduling parameters; reject runtime changes explicitly.
        from rcl_interfaces.msg import SetParametersResult
        self.add_on_set_parameters_callback(lambda params: SetParametersResult(
            successful=not any(p.name in ('velocity_timeout_s', 'state_rate_hz', 'image_rate_hz', 'use_sim_time') for p in params),
            reason='调度参数仅支持启动时设置；use_sim_time=false'))
        if self.get_parameter('use_sim_time').value:
            raise ValueError('当前接口使用系统时间，不支持 use_sim_time=true')
        self.lock = threading.RLock()
        self.controls = default_controls()
        self.detect_enabled = False
        self.command = [0., 0., 0.]
        self.actual_velocity = [0., 0., 0.]
        self.last_cmd = None
        self.last_tick = time.monotonic()
        self.status_pub = self.create_publisher(String, PREFIX + '/command_status', 10)
        self.state_pub = self.create_publisher(String, PREFIX + '/state', STATE_QOS)
        self.enabled_pub = self.create_publisher(Bool, PREFIX + '/qr_detection/enabled', STATE_QOS)
        self.robot_pose_pub = self.create_publisher(PoseStamped, PREFIX + '/robot/pose', 10)
        self.velocity_pub = self.create_publisher(TwistStamped, PREFIX + '/robot/velocity', 10)
        self.create_subscription(Twist, PREFIX + '/robot/cmd_vel', self.receive_velocity, 10)
        self.create_subscription(PoseStamped, PREFIX + '/robot/set_pose', self.receive_robot_pose, 10)
        self.create_subscription(Bool, PREFIX + '/qr_detection/set_enabled', self.receive_enabled, 10)
        self.camera_pubs = {}
        for name in CAMERAS:
            base = PREFIX + '/cameras/' + name
            self.create_subscription(Float64, base + '/set_yaw', lambda msg, n=name: self.receive_angle(n, 'yaw', msg), 10)
            self.create_subscription(Float64, base + '/set_pitch', lambda msg, n=name: self.receive_angle(n, 'pitch', msg), 10)
            if name == 'top':
                self.create_subscription(Float64, base + '/set_height', self.receive_height, 10)
            self.create_subscription(Float64, base + '/set_hfov', lambda msg, n=name: self.receive_hfov(n, msg), 10)
            self.create_subscription(UInt32MultiArray, base + '/set_resolution', lambda msg, n=name: self.receive_resolution(n, msg), 10)
            self.camera_pubs[name] = {
                'yaw': self.create_publisher(Float64, base + '/yaw', STATE_QOS),
                'pitch': self.create_publisher(Float64, base + '/pitch', STATE_QOS),
                'hfov': self.create_publisher(Float64, base + '/hfov', STATE_QOS),
                'resolution': self.create_publisher(UInt32MultiArray, base + '/resolution', STATE_QOS),
                'image_raw': self.create_publisher(Image, base + '/image_raw', IMAGE_QOS),
                'camera_info': self.create_publisher(CameraInfo, base + '/camera_info', IMAGE_QOS),
                'qr_detections': self.create_publisher(String, base + '/qr_detections', 10)}
        self.camera_pubs['top']['height'] = self.create_publisher(Float64, PREFIX + '/cameras/top/height', STATE_QOS)
        self.timer = self.create_timer(1 / state_rate, self.tick)
        self.get_logger().info('04 Jazzy 接口就绪：' + PREFIX)

    def status(self, topic, accepted, reason=''):
        stamp = self.get_clock().now().to_msg()
        self.status_pub.publish(String(data=json.dumps(dict(topic=topic, accepted=accepted, reason=reason,
            stamp=dict(sec=stamp.sec, nanosec=stamp.nanosec)), ensure_ascii=False)))
        if not accepted:
            self.get_logger().warning(f'拒绝 {topic}: {reason}')

    def apply_patch(self, values, detect=None):
        with self.lock:
            self.integrate(time.monotonic())
            updated = validate_controls(dict(self.controls, **values))
            if any(key.startswith('robot_') for key in values):
                self.command = self.actual_velocity = [0., 0., 0.]
                self.last_cmd = None
            self.controls = updated
            if detect is not None:
                self.detect_enabled = bool(detect)

    def guarded_patch(self, topic, values):
        try:
            self.apply_patch(values)
            self.status(topic, True)
        except (ValueError, TypeError) as error:
            self.status(topic, False, str(error))

    def receive_velocity(self, msg):
        topic = PREFIX + '/robot/cmd_vel'
        values = [msg.linear.x, msg.linear.y, msg.angular.z]
        unused = [msg.linear.z, msg.angular.x, msg.angular.y]
        if not np.isfinite(values + unused).all() or any(abs(v) > 1e-9 for v in unused) or max(abs(v) for v in values[:2]) > 1. or abs(values[2]) > 2.:
            self.status(topic, False, '仅支持 linear.x/y ±1 m/s、angular.z ±2 rad/s；其他轴须为0且全部有限')
            return
        with self.lock:
            now = time.monotonic()
            self.integrate(now)
            self.command, self.last_cmd = values, now
        self.status(topic, True)

    def receive_robot_pose(self, msg):
        topic = PREFIX + '/robot/set_pose'
        try:
            if msg.header.frame_id != 'world' or abs(msg.pose.position.z) > 1e-6:
                raise ValueError('robot/set_pose 需要 world 坐标系且 z=0')
            r = rotation_from_pose(msg)
            yaw = math.degrees(math.atan2(r[1, 0], r[0, 0]))
            if not np.allclose(r, rz(yaw), atol=1e-5):
                raise ValueError('机器狗只支持平面 yaw')
            self.apply_patch(dict(robot_x_m=msg.pose.position.x, robot_y_m=msg.pose.position.y, robot_yaw_deg=yaw))
            self.status(topic, True)
        except (ValueError, TypeError) as error:
            self.status(topic, False, str(error))

    def receive_angle(self, name, axis, msg):
        self.guarded_patch(PREFIX + '/cameras/' + name + '/set_' + axis,
                           {name + '_' + axis + '_deg': msg.data})

    def receive_height(self, msg):
        topic = PREFIX + '/cameras/top/set_height'
        height = msg.data
        if not math.isfinite(height) or not 1.325 <= height <= 1.925:
            self.status(topic, False, '顶部光心高度范围 [1.325, 1.925] m，且必须有限')
            return
        # Keep exact endpoint commands valid after converting height to mast length.
        self.guarded_patch(topic, {'pole_length_m': float(np.clip(height - .725, .6, 1.2))})

    def receive_hfov(self, name, msg):
        self.guarded_patch(PREFIX + '/cameras/' + name + '/set_hfov', {name + '_hfov_deg': msg.data})

    def receive_resolution(self, name, msg):
        topic = PREFIX + '/cameras/' + name + '/set_resolution'
        if len(msg.data) != 2 or msg.layout.dim or msg.layout.data_offset:
            self.status(topic, False, 'data 必须为 [width, height]，layout 必须为空')
            return
        self.guarded_patch(topic, {name + '_width': int(msg.data[0]), name + '_height': int(msg.data[1])})

    def receive_enabled(self, msg):
        self.apply_patch({}, detect=msg.data)
        self.status(PREFIX + '/qr_detection/set_enabled', True)

    def integrate(self, now):
        # Only integrate the portion of elapsed wall time covered by a fresh command.
        end = min(now, self.last_cmd + self.timeout) if self.last_cmd is not None else self.last_tick
        dt = max(0., end - self.last_tick)
        vx, vy, omega = self.command
        controls = self.controls
        yaw = math.radians(controls['robot_yaw_deg'])
        if dt:
            if abs(omega) < 1e-9:
                dx, dy = vx * dt, vy * dt
            else:
                a = omega * dt
                dx = (vx * math.sin(a) + vy * (math.cos(a) - 1)) / omega
                dy = (vx * (1 - math.cos(a)) + vy * math.sin(a)) / omega
            x = controls['robot_x_m'] + math.cos(yaw) * dx - math.sin(yaw) * dy
            y = controls['robot_y_m'] + math.sin(yaw) * dx + math.cos(yaw) * dy
            controls['robot_x_m'], controls['robot_y_m'] = float(np.clip(x, -2, 9)), float(np.clip(y, -3, 3))
            controls['robot_yaw_deg'] = (math.degrees(yaw + omega * dt) + 180) % 360 - 180
            if x != controls['robot_x_m'] or y != controls['robot_y_m']:
                self.command = [0., 0., 0.]
        fresh = self.last_cmd is not None and now - self.last_cmd < self.timeout
        self.actual_velocity = self.command.copy() if fresh else [0., 0., 0.]
        self.last_tick = now

    def snapshot(self):
        with self.lock:
            stamp = self.get_clock().now().to_msg()
            return dict(controls=self.controls.copy(), detect=self.detect_enabled, velocity=self.actual_velocity.copy(),
                        stamp=dict(sec=stamp.sec, nanosec=stamp.nanosec))

    def tick(self):
        with self.lock:
            self.integrate(time.monotonic())
            state = self.snapshot()
        from builtin_interfaces.msg import Time
        stamp = Time(**state['stamp'])
        settings = settings_for(state['controls'])
        robot = pose_message(stamp, 'world', [settings.robot_x_m, settings.robot_y_m, 0], settings.base_rotation())
        self.robot_pose_pub.publish(robot)
        velocity = TwistStamped()
        velocity.header.stamp, velocity.header.frame_id = stamp, 'robot_base'
        velocity.twist.linear.x, velocity.twist.linear.y, velocity.twist.angular.z = state['velocity']
        self.velocity_pub.publish(velocity)
        for name, pubs in self.camera_pubs.items():
            pubs['yaw'].publish(Float64(data=state['controls'][name + '_yaw_deg']))
            pubs['pitch'].publish(Float64(data=state['controls'][name + '_pitch_deg']))
            pubs['hfov'].publish(Float64(data=state['controls'][name + '_hfov_deg']))
            pubs['resolution'].publish(UInt32MultiArray(data=[state['controls'][name + '_width'], state['controls'][name + '_height']]))
        self.camera_pubs['top']['height'].publish(Float64(data=.725 + state['controls']['pole_length_m']))
        self.enabled_pub.publish(Bool(data=state['detect']))
        self.state_pub.publish(String(data=json.dumps(state, ensure_ascii=False)))

    def wants_capture(self, name):
        pubs = self.camera_pubs[name]
        return self.snapshot()['detect'] or any(pubs[key].get_subscription_count() for key in ('image_raw', 'camera_info', 'qr_detections'))

    def publish_capture(self, name, image, detections, snapshot, stamp):
        controls = snapshot['controls']
        settings = settings_for(controls)
        pubs = self.camera_pubs[name]
        height, width = image.shape[:2]
        frame = name + '_camera_optical_frame'
        if pubs['image_raw'].get_subscription_count():
            msg = Image()
            msg.header.stamp, msg.header.frame_id = stamp, frame
            msg.height, msg.width, msg.encoding, msg.step = height, width, 'rgb8', width * 3
            msg.data = image.tobytes()
            pubs['image_raw'].publish(msg)
        focal = width / (2 * math.tan(math.radians(controls[name + '_hfov_deg']) / 2))
        info = CameraInfo()
        info.header.stamp, info.header.frame_id = stamp, frame
        info.width, info.height, info.distortion_model = width, height, 'plumb_bob'
        info.d = [0.] * 5
        info.k = [focal, 0., width / 2, 0., focal, height / 2, 0., 0., 1.]
        info.r = np.eye(3).reshape(9).tolist()
        info.p = [focal, 0., width / 2, 0., 0., focal, height / 2, 0., 0., 0., 1., 0.]
        pubs['camera_info'].publish(info)
        result = dict(stamp=dict(sec=stamp.sec, nanosec=stamp.nanosec), frame_id=frame, camera=name,
                      enabled=snapshot['detect'], width=width, height=height,
                      hfov_deg=controls[name + '_hfov_deg'], detections=detections,
                      controls=controls, camera_position_world_m=settings.camera_position(name).tolist(),
                      camera_yaw_deg=controls[name + '_yaw_deg'], camera_pitch_deg=controls[name + '_pitch_deg'],
                      camera_height_m=float(settings.camera_local_position(name)[2]))
        pubs['qr_detections'].publish(String(data=json.dumps(result, ensure_ascii=False)))


class RosBridge:
    def __init__(self, ros_args=None):
        self.context = Context()
        self.node = self.executor = self.thread = None
        try:
            rclpy.init(args=[], context=self.context)
            self.node = EnvironmentNode(context=self.context, ros_args=ros_args)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            self.thread = threading.Thread(target=self.executor.spin, name='04-ros-executor', daemon=True)
            self.thread.start()
        except Exception:
            self.close()
            raise

    def close(self):
        if self.executor is not None:
            self.executor.shutdown()
        if self.thread is not None:
            self.thread.join()
        if self.node is not None:
            self.node.destroy_node()
        if self.context.ok():
            self.context.shutdown()
