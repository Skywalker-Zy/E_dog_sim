"""Two-process Jazzy acceptance checks on an isolated ROS domain."""
import os
os.environ.setdefault('ROS_DOMAIN_ID', '154')
os.environ.setdefault('ROS_LOG_DIR', str(__import__('pathlib').Path(__file__).resolve().parent / 'runtime/ros_logs'))
import json
import math
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, Float64, UInt32MultiArray, String
from ros_interface import PREFIX, IMAGE_QOS, STATE_QOS
from simulation import ROOT, default_controls


def main():
    (ROOT / 'runtime').mkdir(exist_ok=True)
    log = (ROOT / 'runtime/ros_acceptance.log').open('w')
    server = subprocess.Popen(['bash', str(ROOT / 'run.sh'), '--headless'], stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    probe = Node('robot_environment_acceptance')
    state = {}; statuses = []; images = {}; infos = {}; detections = {}; angles = {}; heights = {}; velocities = []
    probe.create_subscription(String, PREFIX + '/state', lambda msg: state.update(json.loads(msg.data)), STATE_QOS)
    probe.create_subscription(String, PREFIX + '/command_status', lambda msg: statuses.append(json.loads(msg.data)), 10)
    probe.create_subscription(TwistStamped, PREFIX + '/robot/velocity', lambda msg: velocities.append(msg), 10)
    for name in ('top', 'left', 'right'):
        base = PREFIX + '/cameras/' + name
        probe.create_subscription(CameraInfo, base + '/camera_info', lambda msg, n=name: infos.__setitem__(n, msg), IMAGE_QOS)
        probe.create_subscription(String, base + '/qr_detections', lambda msg, n=name: detections.__setitem__(n, json.loads(msg.data)), 10)
        for axis in ('yaw', 'pitch'):
            probe.create_subscription(Float64, base + '/' + axis, lambda msg, n=name, a=axis: angles.__setitem__((n, a), msg.data), STATE_QOS)
    probe.create_subscription(Float64, PREFIX + '/cameras/top/height', lambda msg: heights.__setitem__('top', msg.data), STATE_QOS)
    image_sub = probe.create_subscription(Image, PREFIX + '/cameras/left/image_raw', lambda msg: images.__setitem__('left', msg), IMAGE_QOS)
    pubs = {}
    for suffix, typ in [('robot/cmd_vel', Twist), ('robot/set_pose', PoseStamped), ('qr_detection/set_enabled', Bool)]:
        pubs[suffix] = probe.create_publisher(typ, PREFIX + '/' + suffix, 10)
    for name in ('top', 'left', 'right'):
        for suffix, typ in [('set_hfov', Float64), ('set_resolution', UInt32MultiArray), ('set_yaw', Float64), ('set_pitch', Float64)]:
            key = f'cameras/{name}/{suffix}'
            pubs[key] = probe.create_publisher(typ, PREFIX + '/' + key, 10)

    pubs['cameras/top/set_height'] = probe.create_publisher(Float64, PREFIX + '/cameras/top/set_height', 10)

    def wait(predicate, seconds=25):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(probe, timeout_sec=.02)
            if predicate():
                return
            if server.poll() is not None:
                raise AssertionError('server exited: ' + (ROOT / 'runtime/ros_acceptance.log').read_text())
        raise AssertionError('ROS wait timed out; state=' + str(state) + '; log=' + (ROOT / 'runtime/ros_acceptance.log').read_text())

    def send(key, msg):
        wait(lambda: pubs[key].get_subscription_count() > 0)
        pubs[key].publish(msg)

    checks = []
    try:
        wait(lambda: bool(state) and 'left' in images and all(name in infos for name in ('top', 'left', 'right')))
        assert state['controls'] == default_controls()
        image = images['left']
        assert (image.width, image.height, image.encoding, image.step, len(image.data)) == (3840, 2160, 'rgb8', 11520, 24883200)
        checks.append('4K rgb8 image delivered over DDS with correct byte size')
        probe.destroy_subscription(image_sub)
        for name in ('top', 'left', 'right'):
            send(f'cameras/{name}/set_resolution', UInt32MultiArray(data=[640, 360]))
        wait(lambda: all(state['controls'][name + '_width'] == 640 for name in ('top', 'left', 'right')))
        send('cameras/top/set_hfov', Float64(data=100.))
        send('cameras/left/set_hfov', Float64(data=75.))
        send('cameras/right/set_hfov', Float64(data=85.))
        wait(lambda: state['controls']['right_hfov_deg'] == 85. and all(name in detections and detections[name]['width'] == 640 for name in ('top', 'left', 'right')))
        for name in ('top', 'left', 'right'):
            hfov = state['controls'][name + '_hfov_deg']
            wait(lambda n=name, h=hfov: abs(infos[n].k[0] - 640 / (2 * math.tan(math.radians(h) / 2))) < 1e-8)
        checks.append('independent optics commands update images and camera intrinsics')
        for name, yaw, pitch in [('top', 25., -20.), ('left', -15., -30.), ('right', 150., 10.)]:
            send(f'cameras/{name}/set_yaw', Float64(data=yaw))
            send(f'cameras/{name}/set_pitch', Float64(data=pitch))
            wait(lambda n=name, y=yaw, p=pitch: angles.get((n, 'yaw')) == y and angles.get((n, 'pitch')) == p)
        send('cameras/top/set_height', Float64(data=1.725))
        wait(lambda: abs(state['controls']['pole_length_m'] - 1.) < 1e-8 and abs(heights.get('top', 0) - 1.725) < 1e-8)
        assert state['controls']['left_yaw_deg'] == -15. and state['controls']['right_pitch_deg'] == 10.
        checks.append('three camera scalar yaw/pitch commands and scalar top optical height')
        for height, yaw, pitch in [(1.325, -180., -80.), (1.925, 180., 80.)]:
            send('cameras/top/set_height', Float64(data=height))
            send('cameras/top/set_yaw', Float64(data=yaw))
            send('cameras/top/set_pitch', Float64(data=pitch))
            wait(lambda h=height, y=yaw, p=pitch: abs(heights.get('top', 0) - h) < 1e-8 and angles.get(('top', 'yaw')) == y and angles.get(('top', 'pitch')) == p)
        checks.append('exact optical height and angle endpoints accepted')
        old_angles = {key: value for key, value in state['controls'].items() if key.startswith('top_')}
        for key, value in [('set_yaw', 181.), ('set_pitch', float('nan')), ('set_height', 1.0), ('set_hfov', float('nan'))]:
            send('cameras/top/' + key, Float64(data=value))
        send('cameras/top/set_resolution', UInt32MultiArray(data=[0, 0]))
        invalid = Twist(); invalid.linear.z = 1.
        send('robot/cmd_vel', invalid)
        wait(lambda: sum(not s['accepted'] for s in statuses) >= 6)
        assert all(state['controls'][key] == value for key, value in old_angles.items())
        assert state['controls']['pole_length_m'] == 1.2
        checks.append('invalid angle, NaN, height, resolution and velocity commands rejected without changing camera state')
        subscriptions = dict(probe.get_subscriber_names_and_types_by_node('jifang_robot_environment', '/'))
        publishers = dict(probe.get_publisher_names_and_types_by_node('jifang_robot_environment', '/'))
        for name in ('top', 'left', 'right'):
            base = PREFIX + '/cameras/' + name
            assert base + '/set_pose' not in subscriptions
            assert base + '/pose' not in publishers and base + '/pose_local' not in publishers
            assert subscriptions[base + '/set_yaw'] == ['std_msgs/msg/Float64']
            assert subscriptions[base + '/set_pitch'] == ['std_msgs/msg/Float64']
            if name != 'top':
                assert base + '/set_height' not in subscriptions and base + '/height' not in publishers
        assert '/tf' not in publishers and '/tf_static' not in publishers
        checks.append('obsolete camera pose and TF endpoints absent; only top camera exposes height')
        pose = PoseStamped(); pose.header.frame_id = 'world'; pose.pose.position.x = .9
        pose.pose.orientation.z = math.sin(math.pi / 4); pose.pose.orientation.w = math.cos(math.pi / 4)
        send('robot/set_pose', pose)
        wait(lambda: abs(state['controls']['robot_yaw_deg'] - 90.) < 1e-6)
        twist = Twist(); twist.linear.x = .2; twist.linear.y = .1; twist.angular.z = .3
        end = time.monotonic() + .7
        while time.monotonic() < end:
            pubs['robot/cmd_vel'].publish(twist)
            rclpy.spin_once(probe, timeout_sec=.03)
            time.sleep(.03)
        wait(lambda: abs(state['controls']['robot_y_m']) > .05 and state['controls']['robot_yaw_deg'] > 95.)
        wait(lambda: state['velocity'] == [0., 0., 0.], seconds=4)
        stopped = state['controls'].copy()
        end = time.monotonic() + .3
        while time.monotonic() < end:
            rclpy.spin_once(probe, timeout_sec=.02)
        assert stopped == state['controls']
        assert stopped['robot_x_m'] < .9 and stopped['robot_y_m'] > .05
        checks.append('body-frame vx/vy/yaw integration and 0.5 s watchdog stop')
        # Restore a QR-facing pose for real decoding at 4K.
        pose.pose.orientation.z = 0.; pose.pose.orientation.w = 1.
        send('robot/set_pose', pose)
        send('cameras/left/set_yaw', Float64(data=0.))
        send('cameras/left/set_pitch', Float64(data=-40.))
        send('cameras/left/set_resolution', UInt32MultiArray(data=[3840, 2160]))
        send('cameras/left/set_hfov', Float64(data=60.))
        send('qr_detection/set_enabled', Bool(data=True))
        wait(lambda: detections.get('left', {}).get('enabled') and len(detections['left']['detections']) > 0)
        assert all(d['rack_id'] == 'L02' for d in detections['left']['detections'])
        result_count = len(detections['left']['detections'])
        send('qr_detection/set_enabled', Bool(data=False))
        wait(lambda: all(name in detections and not detections[name]['enabled'] and detections[name]['detections'] == [] for name in ('top', 'left', 'right')))
        checks.append('real QR detection enabled, result metadata published, all results cleared when disabled')
        # Match Image and CameraInfo/QR results by acquisition stamp.
        matched_images = {}; matched_info = {}; matched_results = {}
        stamp_key = lambda stamp: (stamp.sec, stamp.nanosec)
        sub = probe.create_subscription(Image, PREFIX + '/cameras/left/image_raw', lambda msg: matched_images.__setitem__(stamp_key(msg.header.stamp), msg), IMAGE_QOS)
        probe.create_subscription(CameraInfo, PREFIX + '/cameras/left/camera_info', lambda msg: matched_info.__setitem__(stamp_key(msg.header.stamp), msg), IMAGE_QOS)
        probe.create_subscription(String, PREFIX + '/cameras/left/qr_detections', lambda msg: matched_results.__setitem__((json.loads(msg.data)['stamp']['sec'], json.loads(msg.data)['stamp']['nanosec']), json.loads(msg.data)), 10)
        wait(lambda: bool(set(matched_images) & set(matched_info) & set(matched_results)))
        key = next(iter(set(matched_images) & set(matched_info) & set(matched_results)))
        assert matched_images[key].width == matched_info[key].width == matched_results[key]['width'] == 3840
        assert matched_images[key].header.frame_id == matched_info[key].header.frame_id == matched_results[key]['frame_id']
        assert 'camera_orientation_world_xyzw' not in matched_results[key]
        assert matched_results[key]['camera_yaw_deg'] == 0. and matched_results[key]['camera_pitch_deg'] == -40.
        assert matched_results[key]['camera_height_m'] == .7
        probe.destroy_subscription(sub)
        checks.append('image/info/QR share acquisition stamp, frame and resolution; scalar camera metadata')
        report = dict(ros_distro=os.environ.get('ROS_DISTRO'), domain_id=os.environ['ROS_DOMAIN_ID'], processes=2,
                      qr_count=result_count, checks=checks)
        (ROOT / 'verification/ros_validation.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps(report, ensure_ascii=False, indent=2))
    finally:
        probe.destroy_node()
        rclpy.shutdown()
        server.send_signal(signal.SIGINT)
        try:
            server.wait(timeout=20)
        except subprocess.TimeoutExpired:
            server.terminate(); server.wait(timeout=10)
        log.close()


if __name__ == '__main__':
    main()
