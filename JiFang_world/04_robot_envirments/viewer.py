"""Threaded control console for the three-camera robot and two-row room."""
import argparse
import json
import math
import signal
from datetime import datetime
from zoneinfo import ZoneInfo
from simulation import ROOT, RobotRoomSimulation, default_controls, CAMERAS, np
from PyQt5 import QtCore, QtGui, QtWidgets
from camera_config import RESOLUTIONS

LABELS = {'top': '顶部相机', 'left': '左侧相机', 'right': '右侧相机'}


class ImagePanel(QtWidgets.QLabel):
    moved = QtCore.pyqtSignal(float, float)
    zoomed = QtCore.pyqtSignal(float)

    def __init__(self, minimum):
        super().__init__('正在加载画面…')
        self.setMinimumSize(*minimum)
        self.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Expanding)
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setStyleSheet('background:#17232c;color:#dce6ed;')
        self.pixmap_source = None
        self.last = None

    def display(self, rgb, detections=()):
        height, width = rgb.shape[:2]
        picture = QtGui.QImage(rgb.data, width, height, rgb.strides[0], QtGui.QImage.Format_RGB888).copy()
        if detections:
            painter = QtGui.QPainter(picture)
            painter.setPen(QtGui.QPen(QtGui.QColor('#16e87b'), 4))
            for detection in detections:
                painter.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(x, y) for x, y in detection['corners_px']]))
            painter.end()
        self.pixmap_source = QtGui.QPixmap.fromImage(picture)
        self.scale_image()

    def scale_image(self):
        if self.pixmap_source is not None:
            self.setPixmap(self.pixmap_source.scaled(self.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))

    def resizeEvent(self, event):
        self.scale_image()
        super().resizeEvent(event)

    def mousePressEvent(self, event):
        self.last = event.pos()

    def mouseMoveEvent(self, event):
        if self.last is not None:
            delta = event.pos() - self.last
            self.moved.emit(delta.x(), delta.y())
            self.last = event.pos()

    def mouseReleaseEvent(self, event):
        self.last = None

    def wheelEvent(self, event):
        self.zoomed.emit(event.angleDelta().y())


class Worker(QtCore.QObject):
    ready = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    stopped = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__()
        self.simulation = None
        self.overview_key = self.sensor_key = None
        self.overview_image = self.sensor_image = None
        self.detections = []

    @QtCore.pyqtSlot(object)
    def frame(self, request):
        try:
            if self.simulation is None:
                self.simulation = RobotRoomSimulation(ROOT)
            sim = self.simulation
            if request['controls'] != sim.controls:
                sim.apply_controls(request['controls'])
            sim.set_targets([100., 100.] if request['door_open'] else [0., 0.])
            sim.step()
            # Quantize door motion below visible angular changes; cache static images.
            state = (tuple(sim.controls.values()), tuple(np.round(sim.data.qpos, 4)))
            overview_key = (state, request['view'], tuple(request['angles']))
            sensor_key = (state, request['camera'], request['detect'])
            overview_changed = overview_key != self.overview_key
            sensor_changed = sensor_key != self.sensor_key
            if overview_changed:
                self.overview_image = sim.overview(request['view'], request['angles'])
                self.overview_key = overview_key
            if sensor_changed:
                self.sensor_image, self.detections = sim.capture(request['camera'], request['detect'])
                self.sensor_key = sensor_key
            self.ready.emit(dict(revision=request['revision'], overview=self.overview_image, sensor=self.sensor_image,
                                 overview_changed=overview_changed, sensor_changed=sensor_changed,
                                 detections=self.detections, camera=request['camera'], detect=request['detect'],
                                 controls=sim.controls.copy(), door_angles=sim.angles(), door_open=request['door_open'],
                                 view=request['view'], angles=request['angles']))
        except Exception as error:
            self.failed.emit(f'{type(error).__name__}: {error}')

    @QtCore.pyqtSlot()
    def shutdown(self):
        try:
            if self.simulation is not None:
                self.simulation.close()
        finally:
            self.stopped.emit()


class RobotRoomWindow(QtWidgets.QMainWindow):
    requested = QtCore.pyqtSignal(object)

    def __init__(self):
        super().__init__()
        self.setWindowTitle('三摄机器狗 · 双列机柜控制台')
        self.resize(1550, 950)
        self.busy = self.closing = self.door_open = False
        self.revision = 0
        self.last_payload = None
        self.inputs = {}
        self.resolution_inputs = {}
        self.follow_angles = [195., -18., 2.5]
        self.free_angles = [150., -24., 8.]
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFixedWidth(350)
        controls = QtWidgets.QWidget()
        scroll.setWidget(controls)
        left = QtWidgets.QVBoxLayout(controls)
        self.door_button = QtWidgets.QPushButton('开门')
        self.door_button.setMinimumHeight(36)
        self.door_button.clicked.connect(self.toggle_door)
        left.addWidget(self.door_button)
        self.door_state = QtWidgets.QLabel('双扇门已关闭')
        left.addWidget(self.door_state)
        camera_group = QtWidgets.QGroupBox('三相机转向 / 顶部升降')
        camera_grid = QtWidgets.QGridLayout(camera_group)
        for column, text in enumerate(['相机', '水平 yaw (°)', '俯仰 pitch (°)']):
            camera_grid.addWidget(QtWidgets.QLabel(text), 0, column)
        for row, name in enumerate(CAMERAS, 1):
            camera_grid.addWidget(QtWidgets.QLabel(LABELS[name]), row, 0)
            camera_grid.addWidget(self.number(name + '_yaw_deg', -180, 180, 5, 1), row, 1)
            camera_grid.addWidget(self.number(name + '_pitch_deg', -80, 80, 5, 1), row, 2)
        camera_grid.addWidget(QtWidgets.QLabel('伸缩杆 (m)'), 4, 0)
        camera_grid.addWidget(self.number('pole_length_m', .6, 1.2, .05, 3), 4, 1, 1, 2)
        self.top_height = QtWidgets.QLabel()
        camera_grid.addWidget(self.top_height, 5, 0, 1, 3)
        hint = QtWidgets.QLabel('yaw 0° 朝机身左侧，±180° 朝右侧；\npitch 正值抬头，负值低头。')
        camera_grid.addWidget(hint, 6, 0, 1, 3)
        left.addWidget(camera_group)
        optics_group = QtWidgets.QGroupBox('各相机视场角 / 分辨率')
        optics_grid = QtWidgets.QGridLayout(optics_group)
        for column, text in enumerate(['相机', '水平视场角 (°)', '分辨率']):
            optics_grid.addWidget(QtWidgets.QLabel(text), 0, column)
        for row, name in enumerate(CAMERAS, 1):
            optics_grid.addWidget(QtWidgets.QLabel(LABELS[name]), row, 0)
            optics_grid.addWidget(self.number(name + '_hfov_deg', 10, 150, 5, 1), row, 1)
            resolution = QtWidgets.QComboBox()
            for title, size in RESOLUTIONS:
                resolution.addItem(title, size)
            resolution.setCurrentIndex(3)
            resolution.currentIndexChanged.connect(self.changed)
            self.resolution_inputs[name] = resolution
            optics_grid.addWidget(resolution, row, 2)
        left.addWidget(optics_group)
        robot_group = QtWidgets.QGroupBox('机器狗位姿')
        grid = QtWidgets.QGridLayout(robot_group)
        for row, (key, title, low, high, step) in enumerate([
                ('robot_x_m', '世界 X (m)', -2, 9, .1), ('robot_y_m', '世界 Y (m)', -3, 3, .1),
                ('robot_yaw_deg', '朝向 yaw (°)', -180, 180, 5)]):
            grid.addWidget(QtWidgets.QLabel(title), row, 0)
            grid.addWidget(self.number(key, low, high, step, 3 if row < 2 else 1), row, 1, 1, 2)
        self.move_step = QtWidgets.QDoubleSpinBox()
        self.move_step.setRange(.01, 1.)
        self.move_step.setSingleStep(.05)
        self.move_step.setValue(.1)
        grid.addWidget(QtWidgets.QLabel('平移步长 (m)'), 3, 0)
        grid.addWidget(self.move_step, 3, 1, 1, 2)
        self.turn_step = QtWidgets.QDoubleSpinBox()
        self.turn_step.setRange(1, 90)
        self.turn_step.setValue(5)
        grid.addWidget(QtWidgets.QLabel('旋转步长 (°)'), 4, 0)
        grid.addWidget(self.turn_step, 4, 1, 1, 2)
        for title, row, column, callback in [
                ('前进', 5, 1, lambda: self.move_robot(1, 0)),
                ('左移', 6, 0, lambda: self.move_robot(0, 1)),
                ('后退', 6, 1, lambda: self.move_robot(-1, 0)),
                ('右移', 6, 2, lambda: self.move_robot(0, -1)),
                ('左转', 7, 0, lambda: self.turn_robot(1)),
                ('右转', 7, 2, lambda: self.turn_robot(-1))]:
            button = QtWidgets.QPushButton(title)
            button.setAutoRepeat(True)
            button.setAutoRepeatInterval(160)
            button.clicked.connect(callback)
            grid.addWidget(button, row, column)
        grid.addWidget(QtWidgets.QLabel('平移方向相对机身；按住按钮连续移动。'), 8, 0, 1, 3)
        left.addWidget(robot_group)
        reset = QtWidgets.QPushButton('复位机器人与相机')
        reset.clicked.connect(self.reset_controls)
        left.addWidget(reset)
        save = QtWidgets.QPushButton('保存画面与当前配置')
        save.clicked.connect(self.save_capture)
        left.addWidget(save)
        self.notice = QtWidgets.QLabel('直接更新站姿模型位姿。')
        self.notice.setWordWrap(True)
        left.addWidget(self.notice)
        left.addStretch()
        # Include the new optics table and vertical scrollbar in the width.
        scroll.setFixedWidth(max(350, controls.minimumSizeHint().width() + 24))
        layout.addWidget(scroll)
        right = QtWidgets.QVBoxLayout()
        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel('场景视角'))
        self.view = QtWidgets.QComboBox()
        for title, key in [('跟随机器狗', 'follow'), ('通道入口', 'aisle'), ('俯视布局', 'top'),
                           ('外侧总览', 'overview'), ('门口近景', 'door_detail'), ('自由观察', 'free')]:
            self.view.addItem(title, key)
        self.view.currentIndexChanged.connect(self.changed)
        row.addWidget(self.view)
        row.addStretch()
        right.addLayout(row)
        self.overview_panel = ImagePanel((500, 380))
        self.overview_panel.moved.connect(self.orbit)
        self.overview_panel.zoomed.connect(self.zoom)
        right.addWidget(self.overview_panel, 1)
        sensor_box = QtWidgets.QGroupBox('相机视野 / 二维码识别')
        sensor_layout = QtWidgets.QVBoxLayout(sensor_box)
        bar = QtWidgets.QHBoxLayout()
        self.camera_select = QtWidgets.QComboBox()
        for name in CAMERAS:
            self.camera_select.addItem(LABELS[name], name)
        self.camera_select.setCurrentIndex(1)
        self.detect_checkbox = QtWidgets.QCheckBox('识别二维码')
        self.camera_select.currentIndexChanged.connect(self.sensor_changed)
        self.detect_checkbox.toggled.connect(self.sensor_changed)
        bar.addWidget(self.camera_select)
        bar.addWidget(self.detect_checkbox)
        bar.addStretch()
        sensor_layout.addLayout(bar)
        self.sensor_info = QtWidgets.QLabel()
        self.sensor_info.setWordWrap(True)
        sensor_layout.addWidget(self.sensor_info)
        self.sensor_panel = ImagePanel((350, 197))
        self.sensor_panel.setMaximumHeight(290)
        sensor_layout.addWidget(self.sensor_panel)
        sensor_layout.addWidget(QtWidgets.QLabel('二维码识别结果'))
        self.results = QtWidgets.QPlainTextEdit()
        self.results.setReadOnly(True)
        self.results.setPlainText('未启用二维码识别')
        self.results.setMinimumWidth(220)
        sensor_layout.addWidget(self.results, 1)
        sensor_box.setFixedWidth(395)
        layout.addLayout(right, 1)
        layout.addWidget(sensor_box)
        self.changed()
        self.worker = Worker()
        self.thread = QtCore.QThread(self)
        self.worker.moveToThread(self.thread)
        self.requested.connect(self.worker.frame)
        self.worker.ready.connect(self.received)
        self.worker.failed.connect(self.failed)
        self.worker.stopped.connect(self.thread.quit, QtCore.Qt.DirectConnection)
        self.thread.finished.connect(self.worker.deleteLater)
        self.thread.start()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(100)

    def number(self, key, low, high, step, decimals):
        spin = QtWidgets.QDoubleSpinBox()
        spin.setRange(low, high)
        spin.setDecimals(decimals)
        spin.setSingleStep(step)
        spin.setValue(default_controls()[key])
        spin.setKeyboardTracking(False)
        self.inputs[key] = spin
        spin.valueChanged.connect(self.changed)
        return spin

    def controls(self):
        controls = {key: spin.value() for key, spin in self.inputs.items()}
        for name, combo in self.resolution_inputs.items():
            width, height = combo.currentData()
            controls[name + '_width'] = int(width)
            controls[name + '_height'] = int(height)
        return controls

    def changed(self, *unused):
        self.revision += 1
        if hasattr(self, 'sensor_info'):
            name = self.camera_select.currentData()
            width, height = self.resolution_inputs[name].currentData()
            hfov = self.inputs[name + '_hfov_deg'].value()
            self.sensor_info.setText(f'{width}×{height} · 水平视场角 {hfov:g}°\n识别与保存均使用原始分辨率')
        if hasattr(self, 'top_height'):
            self.top_height.setText(f"顶部光心高度：{.725 + self.inputs['pole_length_m'].value():.3f} m")

    def sensor_changed(self, *unused):
        self.changed()
        self.sensor_panel.clear()
        self.sensor_panel.pixmap_source = None
        self.sensor_panel.setText('正在更新相机画面…')
        self.results.setPlainText('等待当前相机识别…' if self.detect_checkbox.isChecked() else '未启用二维码识别')

    def toggle_door(self):
        self.door_open = not self.door_open
        self.door_button.setText('关门' if self.door_open else '开门')
        self.changed()

    def move_robot(self, forward, left):
        a = math.radians(self.inputs['robot_yaw_deg'].value())
        step = self.move_step.value()
        self.inputs['robot_x_m'].setValue(self.inputs['robot_x_m'].value() + step * (forward * math.cos(a) - left * math.sin(a)))
        self.inputs['robot_y_m'].setValue(self.inputs['robot_y_m'].value() + step * (forward * math.sin(a) + left * math.cos(a)))

    def turn_robot(self, sign):
        spin = self.inputs['robot_yaw_deg']
        spin.setValue((spin.value() + sign * self.turn_step.value() + 180) % 360 - 180)

    def reset_controls(self):
        defaults = default_controls()
        for key, spin in self.inputs.items():
            spin.setValue(defaults[key])
        for name, combo in self.resolution_inputs.items():
            size = (defaults[name + '_width'], defaults[name + '_height'])
            index = next(index for index in range(combo.count()) if tuple(combo.itemData(index)) == size)
            combo.setCurrentIndex(index)
        self.changed()

    def orbit(self, dx, dy):
        if self.view.currentData() not in ('follow', 'free'):
            self.view.setCurrentIndex(self.view.findData('free'))
        angles = self.follow_angles if self.view.currentData() == 'follow' else self.free_angles
        angles[0] -= dx * .3
        angles[1] = float(np.clip(angles[1] - dy * .3, -89, 20))
        self.changed()

    def zoom(self, delta):
        if self.view.currentData() not in ('follow', 'free'):
            self.view.setCurrentIndex(self.view.findData('free'))
        angles = self.follow_angles if self.view.currentData() == 'follow' else self.free_angles
        angles[2] = float(np.clip(angles[2] * math.exp(-delta / 1200), .5, 18))
        self.changed()

    def tick(self):
        if self.busy or self.closing:
            return
        self.busy = True
        self.requested.emit(dict(revision=self.revision, controls=self.controls(), door_open=self.door_open,
                                 camera=self.camera_select.currentData(), detect=self.detect_checkbox.isChecked(),
                                 view=self.view.currentData(), angles=(self.follow_angles if self.view.currentData() == 'follow' else self.free_angles).copy()))

    @QtCore.pyqtSlot(object)
    def received(self, payload):
        self.busy = False
        if self.closing or payload['revision'] != self.revision:
            return
        previous = self.last_payload
        refresh = previous is None or previous['revision'] != payload['revision']
        self.last_payload = payload
        # A skipped stale frame may have populated the worker cache; always
        # display the latest matching snapshot even when its image was reused.
        if refresh or payload['overview_changed']:
            self.overview_panel.display(payload['overview'])
        if refresh or payload['sensor_changed']:
            self.sensor_panel.display(payload['sensor'], payload['detections'])
        if payload['detect']:
            lines = [f"{LABELS[payload['camera']]}：识别到 {len(payload['detections'])} 个二维码"]
            for item in payload['detections']:
                lines.append(item['payload'])
                if item['rack_id']:
                    lines.append(f"  机柜 {item['rack_id']}  高度 {item['height_m']:.2f} m")
            text = '\n'.join(lines)
        else:
            text = '未启用二维码识别'
        if self.results.toPlainText() != text:
            self.results.setPlainText(text)
        a, b = payload['door_angles']
        self.door_state.setText(f'双扇门角度：{a:.1f}° / {b:.1f}°')
        name = payload['camera']
        controls = payload['controls']
        self.statusBar().showMessage(f"{LABELS[name]} · {controls[name + '_width']}×{controls[name + '_height']} · 水平视场角 {controls[name + '_hfov_deg']:g}° · 机器狗 X={controls['robot_x_m']:.2f} Y={controls['robot_y_m']:.2f}")

    @QtCore.pyqtSlot(str)
    def failed(self, message):
        self.busy = False
        self.timer.stop()
        self.notice.setText('运行错误：' + message)

    def save_capture(self):
        payload = self.last_payload
        if payload is None or payload['revision'] != self.revision:
            self.notice.setText('等待当前设置的画面更新后再保存。')
            return
        from PIL import Image
        directory = ROOT / 'captures' / datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f')
        directory.mkdir(parents=True)
        Image.fromarray(payload['overview']).save(directory / 'overview.png')
        Image.fromarray(payload['sensor']).save(directory / (payload['camera'] + '.png'))
        settings = {key: value for key, value in payload.items() if key not in ('overview', 'sensor')}
        (directory / 'settings.json').write_text(json.dumps(settings, ensure_ascii=False, indent=2) + '\n')
        self.notice.setText('已保存：' + str(directory))

    def closeEvent(self, event):
        self.closing = True
        self.timer.stop()
        QtCore.QMetaObject.invokeMethod(self.worker, 'shutdown', QtCore.Qt.QueuedConnection)
        self.thread.wait()
        event.accept()


def create_window():
    return RobotRoomWindow()


def main():
    parser = argparse.ArgumentParser(description='三摄机器狗与双列机柜控制台')
    parser.parse_args()
    app = QtWidgets.QApplication([])
    app.setStyle('Fusion')
    window = create_window()
    window.show()
    signal.signal(signal.SIGINT, lambda *unused: window.close())
    timer = QtCore.QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(250)
    app.exec_()


if __name__ == '__main__':
    main()
