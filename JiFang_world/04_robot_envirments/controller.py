"""Own the simulation on one thread and share desired controls with ROS."""
import time
import numpy as np
from simulation import ROOT, CAMERAS, RobotRoomSimulation


class EnvironmentRuntime:
    def __init__(self, ros_enabled=False, ros_args=None):
        self.simulation = RobotRoomSimulation(ROOT)
        self.bridge = None
        self.detect_enabled = False
        self.cache = {}
        self.last_state = None
        self.last_publish = -float('inf')
        try:
            if ros_enabled:
                from ros_interface import RosBridge
                self.bridge = RosBridge(ros_args)
        except Exception:
            self.simulation.close()
            raise

    def prepare(self, edits=None, detect_edit=None, door_open=False):
        sim = self.simulation
        if self.bridge:
            if edits or detect_edit is not None:
                self.bridge.node.apply_patch(edits or {}, detect_edit)
            snapshot = self.bridge.node.snapshot()
            controls, self.detect_enabled = snapshot['controls'], snapshot['detect']
        else:
            controls = dict(sim.controls, **(edits or {}))
            if detect_edit is not None:
                self.detect_enabled = detect_edit
            snapshot = dict(controls=controls, detect=self.detect_enabled, velocity=[0., 0., 0.])
        if controls != sim.controls:
            sim.apply_controls(controls)
        sim.set_targets([100., 100.] if door_open else [0., 0.])
        sim.step()
        state = (tuple(sim.controls.values()), tuple(np.round(sim.data.qpos, 4)), self.detect_enabled)
        if state != self.last_state:
            self.cache.clear()
            self.last_state = state
        return snapshot

    def capture(self, name):
        if name not in self.cache:
            self.cache[name] = self.simulation.capture(name, self.detect_enabled)
        return self.cache[name]

    def publish_due(self, snapshot):
        if not self.bridge:
            return
        node = self.bridge.node
        now = time.monotonic()
        if now - self.last_publish < 1 / node.image_rate:
            return
        self.last_publish = now
        from builtin_interfaces.msg import Time
        stamp = Time(**snapshot['stamp'])
        for name in CAMERAS:
            if node.wants_capture(name):
                image, detections = self.capture(name)
                node.publish_capture(name, image, detections, snapshot, stamp)

    def close(self):
        try:
            if self.bridge:
                self.bridge.close()
        finally:
            self.simulation.close()
