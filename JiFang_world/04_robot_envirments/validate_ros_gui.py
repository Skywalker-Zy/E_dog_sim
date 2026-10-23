"""Check ROS/UI state synchronization without feedback overwriting commands."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('ROS_DOMAIN_ID', '155')
os.environ.setdefault('ROS_LOG_DIR', str(__import__('pathlib').Path(__file__).resolve().parent / 'runtime/ros_logs'))
import json
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, UInt32MultiArray, Bool, String
from ros_interface import PREFIX, STATE_QOS
from viewer import ROOT, QtWidgets, create_window


def main():
    app = QtWidgets.QApplication([])
    rclpy.init()
    probe = Node('environment_gui_acceptance')
    state = {}
    probe.create_subscription(String, PREFIX + '/state', lambda msg: state.update(json.loads(msg.data)), STATE_QOS)
    velocity = probe.create_publisher(Twist, PREFIX + '/robot/cmd_vel', 10)
    hfov = probe.create_publisher(Float64, PREFIX + '/cameras/left/set_hfov', 10)
    resolution = probe.create_publisher(UInt32MultiArray, PREFIX + '/cameras/left/set_resolution', 10)
    yaw = probe.create_publisher(Float64, PREFIX + '/cameras/top/set_yaw', 10)
    pitch = probe.create_publisher(Float64, PREFIX + '/cameras/top/set_pitch', 10)
    height = probe.create_publisher(Float64, PREFIX + '/cameras/top/set_height', 10)
    detect = probe.create_publisher(Bool, PREFIX + '/qr_detection/set_enabled', 10)
    window = create_window(True); window.show()

    def wait(predicate):
        end = time.monotonic() + 35
        while time.monotonic() < end:
            app.processEvents(); rclpy.spin_once(probe, timeout_sec=.01)
            if '运行错误' in window.notice.text():
                raise AssertionError(window.notice.text())
            if predicate():
                return
        raise AssertionError('GUI/ROS synchronization timeout')

    try:
        wait(lambda: window.last_payload is not None and bool(state) and velocity.get_subscription_count() > 0)
        wait(lambda: yaw.get_subscription_count() and pitch.get_subscription_count() and height.get_subscription_count())
        yaw.publish(Float64(data=35.))
        pitch.publish(Float64(data=-25.))
        height.publish(Float64(data=1.825))
        wait(lambda: window.inputs['top_yaw_deg'].value() == 35. and window.inputs['top_pitch_deg'].value() == -25. and abs(window.inputs['pole_length_m'].value() - 1.1) < 1e-8)
        hfov.publish(Float64(data=80.))
        resolution.publish(UInt32MultiArray(data=[800, 600]))
        wait(lambda: window.inputs['left_hfov_deg'].value() == 80. and tuple(window.resolution_inputs['left'].currentData()) == (800, 600))
        wait(lambda: window.last_payload['sensor'].shape == (600, 800, 3))
        assert window.pending_controls == {}
        # Camera edits from UI must leave incoming robot velocities intact.
        twist = Twist(); twist.linear.x = .15
        end = time.monotonic() + .8
        while time.monotonic() < end:
            velocity.publish(twist)
            app.processEvents(); rclpy.spin_once(probe, timeout_sec=.01); time.sleep(.025)
        wait(lambda: state['controls']['robot_x_m'] > .95)
        window.inputs['top_hfov_deg'].setValue(105.)
        wait(lambda: state['controls']['top_hfov_deg'] == 105.)
        wait(lambda: state['velocity'] == [0., 0., 0.])
        wait(lambda: abs(window.inputs['robot_x_m'].value() - state['controls']['robot_x_m']) < .001)
        window.move_robot(1, 0)
        wait(lambda: abs(state['controls']['robot_x_m'] - window.inputs['robot_x_m'].value()) < .001)
        detect.publish(Bool(data=True))
        wait(lambda: window.detect_checkbox.isChecked() and window.last_payload['detect'])
        window.detect_checkbox.setChecked(False)
        wait(lambda: not state['detect'] and not window.last_payload['detect'])
        report = {'checks': ['scalar camera yaw/pitch/height commands reflected in GUI', 'ROS optics/custom resolution reflected in UI and image',
                             'ROS velocity moves robot without stale UI pose overwrite',
                             'GUI camera edit publishes to ROS state', 'GUI manual pose controls remain authoritative',
                             'ROS detection toggle reflected in checkbox', 'GUI detection toggle reflected in ROS state']}
        (ROOT / 'verification/ros_gui_validation.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))
    finally:
        window.close(); app.processEvents()
        probe.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
