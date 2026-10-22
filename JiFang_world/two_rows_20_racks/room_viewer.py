#!/usr/bin/env python3
"""Chinese manual door buttons and a responsive standalone scene window."""
from pathlib import Path
import argparse
import json
import math
import signal
import time
from datetime import datetime
from runtime import ROOT, RoomSimulation, mujoco, np
from PyQt5 import QtCore, QtGui, QtWidgets


class RoomWorker(QtCore.QObject):
    ready = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    stopped = QtCore.pyqtSignal()

    def __init__(self, directory):
        super().__init__()
        self.directory = directory
        self.simulation = None
        self.last_action = -1
        self.cached_key = None
        self.cached_image = None
        self.renderer_name = ''

    @QtCore.pyqtSlot(object)
    def frame(self, request):
        try:
            if self.simulation is None:
                self.simulation = RoomSimulation(self.directory)
            sim = self.simulation
            targets = request['targets']
            if request['action_id'] != self.last_action:
                if request['action'] == 'reset':
                    sim.reset()
                elif request['action'] == 'hold':
                    targets = sim.hold()
                self.last_action = request['action_id']
            sim.set_targets(targets)
            before = sim.data.qpos.copy()
            sim.step()
            after = sim.data.qpos.copy()
            view = request['view']
            free = None
            if view == 'free':
                free = mujoco.MjvCamera()
                free.type = mujoco.mjtCamera.mjCAMERA_FREE
                free.lookat[:] = request['free_lookat']
                free.azimuth, free.elevation, free.distance = request['free_angles']
            key = (view, tuple(request['free_lookat']), tuple(request['free_angles']))
            changed = np.max(np.abs(after - before)) > .00005 or self.cached_key != key or self.cached_image is None
            if changed:
                self.cached_image = sim.render(view if free is None else 'aisle', free_camera=free)
                self.cached_key = key
                if not self.renderer_name:
                    try:
                        from OpenGL import GL
                        sim.renderer._gl_context.make_current()
                        self.renderer_name = GL.glGetString(GL.GL_RENDERER).decode()
                    except Exception:
                        self.renderer_name = 'OpenGL 渲染'
            self.ready.emit(dict(image=self.cached_image, changed=changed, angles=sim.angles(), targets=targets,
                                 action_id=request['action_id'], held=request['action'] == 'hold', renderer=self.renderer_name,
                                 contacts=sim.data.ncon, seconds=float(sim.data.time), warnings=list(map(int, sim.data.warning.number))))
        except Exception as error:
            self.failed.emit(f'{type(error).__name__}: {error}')

    @QtCore.pyqtSlot()
    def shutdown(self):
        try:
            if self.simulation:
                self.simulation.close()
        finally:
            self.stopped.emit()


class ImageView(QtWidgets.QLabel):
    moved = QtCore.pyqtSignal(float, float)
    zoomed = QtCore.pyqtSignal(float)

    def __init__(self):
        super().__init__()
        self.image = None
        self.last = None
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setMinimumSize(640, 450)
        self.setStyleSheet('background:#14202a;color:#e6eef3;')
        self.setText('正在加载 20 个机柜及门…')

    def display(self, rgb):
        height, width = rgb.shape[:2]
        self.image = QtGui.QPixmap.fromImage(QtGui.QImage(rgb.data, width, height, rgb.strides[0], QtGui.QImage.Format_RGB888).copy())
        self.scale()

    def scale(self):
        if self.image:
            self.setPixmap(self.image.scaled(self.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.FastTransformation))

    def resizeEvent(self, event):
        self.scale()
        super().resizeEvent(event)

    def mousePressEvent(self, event):
        self.last = event.pos()

    def mouseMoveEvent(self, event):
        if self.last is not None:
            delta = event.pos() - self.last
            self.moved.emit(float(delta.x()), float(delta.y()))
            self.last = event.pos()

    def mouseReleaseEvent(self, event):
        self.last = None

    def wheelEvent(self, event):
        self.zoomed.emit(float(event.angleDelta().y()))


class RoomWindow(QtWidgets.QMainWindow):
    requested = QtCore.pyqtSignal(object)

    def __init__(self, directory=ROOT):
        super().__init__()
        self.directory = Path(directory)
        layout_info = json.loads((self.directory / 'layout.json').read_text())
        self.setWindowTitle('双列 20 机柜机房 · 手动按钮开关门')
        self.resize(1480, 950)
        self.targets = [0., 0.]
        self.busy = self.closing = False
        self.action = 'none'
        self.action_id = 0
        self.last_payload = None
        self.free_lookat = [3.0, 0., 1.1]
        self.free_angles = [150., -24., 8.0]
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setMaximumWidth(325)
        controls = QtWidgets.QWidget()
        scroll.setWidget(controls)
        root.addWidget(scroll)
        left = QtWidgets.QVBoxLayout(controls)
        title = QtWidgets.QLabel('双列机房场景')
        title.setStyleSheet('font-size:21px;font-weight:bold;padding:6px;')
        left.addWidget(title)
        label = QtWidgets.QLabel(f"每列 10 个机柜，共 20 个\n机柜正面净距 1.20 m\n单柜 0.60×1.00×2.177 m\n已安装设备 {layout_info['installed_equipment_count']} 台\n二维码 {layout_info['qr_count']} 张\n缺失标签设备 {layout_info['missing_qr_on_installed_equipment']} 台\n随机种子 {layout_info['seed']}")
        label.setWordWrap(True)
        label.setStyleSheet('padding:8px;background:#e9f0f5;color:#183247;')
        left.addWidget(label)
        group = QtWidgets.QGroupBox('通道端部双扇门')
        group.setMinimumHeight(300)
        buttons = QtWidgets.QVBoxLayout(group)
        for text, callback in [('开门（双扇）', self.open_doors), ('关门（双扇）', self.close_doors), ('保持当前位置', self.hold_doors)]:
            button = QtWidgets.QPushButton(text)
            button.setMinimumHeight(38)
            button.clicked.connect(callback)
            buttons.addWidget(button)
        row = QtWidgets.QHBoxLayout()
        for index, text in enumerate(['左门切换', '右门切换']):
            button = QtWidgets.QPushButton(text)
            button.clicked.connect(lambda checked=False, i=index: self.toggle_leaf(i))
            row.addWidget(button)
        buttons.addLayout(row)
        self.angle = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.angle.setRange(0, 100)
        self.angle.valueChanged.connect(self.set_both)
        buttons.addWidget(QtWidgets.QLabel('双扇目标开角：0°关闭，100°打开'))
        buttons.addWidget(self.angle)
        self.door_state = QtWidgets.QLabel('左门 0.0° / 右门 0.0°')
        self.door_state.setWordWrap(True)
        buttons.addWidget(self.door_state)
        left.addWidget(group)
        left.addWidget(QtWidgets.QLabel('观察视角'))
        self.view = QtWidgets.QComboBox()
        for text, key in [('通道入口', 'aisle'), ('门口近景', 'door_detail'), ('俯视布局', 'top'), ('外侧总览', 'overview'), ('自由观察', 'free')]:
            self.view.addItem(text, key)
        left.addWidget(self.view)
        hint = QtWidgets.QLabel('在自由观察模式下拖动旋转，滚轮缩放。\n键盘 O 开门，C 关门。\n门为真实铰链刚体，开门时向后方前室摆动。')
        hint.setWordWrap(True)
        left.addWidget(hint)
        for text, callback in [('保存当前截图', self.save_image), ('复位场景（门关闭）', self.reset)]:
            button = QtWidgets.QPushButton(text)
            button.clicked.connect(callback)
            left.addWidget(button)
        self.notice = QtWidgets.QLabel('这里只包含机房和门，不包含机器狗或相机设备。')
        self.notice.setWordWrap(True)
        self.notice.setMaximumHeight(110)
        left.addWidget(self.notice)
        left.addStretch()
        self.image_view = ImageView()
        self.image_view.moved.connect(self.orbit)
        self.image_view.zoomed.connect(self.zoom)
        root.addWidget(self.image_view, 1)
        QtWidgets.QShortcut(QtGui.QKeySequence('O'), self, activated=self.open_doors)
        QtWidgets.QShortcut(QtGui.QKeySequence('C'), self, activated=self.close_doors)
        self.thread = QtCore.QThread(self)
        self.worker = RoomWorker(self.directory)
        self.worker.moveToThread(self.thread)
        self.requested.connect(self.worker.frame)
        self.worker.ready.connect(self.received)
        self.worker.failed.connect(self.failed)
        self.worker.stopped.connect(self.thread.quit, QtCore.Qt.DirectConnection)
        self.thread.finished.connect(self.worker.deleteLater)
        self.thread.start()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(66)

    def set_both(self, degrees):
        self.targets = [float(degrees), float(degrees)]
        self.action = 'none'
        self.action_id += 1

    def sync_slider(self):
        self.angle.blockSignals(True)
        self.angle.setValue(round(sum(self.targets) / 2))
        self.angle.blockSignals(False)

    def open_doors(self):
        self.targets = [100., 100.]
        self.action = 'none'
        self.action_id += 1
        self.sync_slider()

    def close_doors(self):
        self.targets = [0., 0.]
        self.action = 'none'
        self.action_id += 1
        self.sync_slider()

    def hold_doors(self):
        self.action = 'hold'
        self.action_id += 1

    def toggle_leaf(self, index):
        self.targets[index] = 0. if self.targets[index] > 50 else 100.
        self.action = 'none'
        self.action_id += 1
        self.sync_slider()

    def reset(self):
        self.targets = [0., 0.]
        self.action = 'reset'
        self.action_id += 1
        self.sync_slider()

    def orbit(self, dx, dy):
        if self.view.currentData() != 'free':
            self.view.setCurrentIndex(4)
        self.free_angles[0] -= dx * .35
        self.free_angles[1] = float(np.clip(self.free_angles[1] - dy * .25, -89.9, 30))

    def zoom(self, delta):
        if self.view.currentData() != 'free':
            self.view.setCurrentIndex(4)
        self.free_angles[2] = float(np.clip(self.free_angles[2] * math.exp(-delta / 1200), .5, 18))

    def tick(self):
        if self.busy or self.closing:
            return
        self.busy = True
        self.requested.emit(dict(targets=self.targets.copy(), action=self.action, action_id=self.action_id,
                                 view=self.view.currentData(), free_lookat=self.free_lookat.copy(), free_angles=self.free_angles.copy()))

    @QtCore.pyqtSlot(object)
    def received(self, payload):
        self.busy = False
        if self.closing:
            return
        self.last_payload = payload
        if payload['changed']:
            self.image_view.display(payload['image'])
        if payload['held'] and payload['action_id'] == self.action_id:
            self.targets = payload['targets']
            self.action = 'none'
            self.sync_slider()
        self.door_state.setText(f"左门 {payload['angles'][0]:.1f}° / 右门 {payload['angles'][1]:.1f}°\n目标 {self.targets[0]:.0f}° / {self.targets[1]:.0f}°")
        self.statusBar().showMessage(f"{payload['renderer']} | 仿真 {payload['seconds']:.1f} s | 接触 {payload['contacts']}")
        if any(payload['warnings']):
            self.notice.setText('仿真警告：' + str(payload['warnings']))

    @QtCore.pyqtSlot(str)
    def failed(self, message):
        self.busy = False
        self.notice.setText('运行错误：' + message)

    def save_image(self):
        if not self.last_payload:
            return
        from PIL import Image
        from zoneinfo import ZoneInfo
        directory = self.directory / 'captures'
        directory.mkdir(exist_ok=True)
        path = directory / (datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f') + '.png')
        Image.fromarray(self.last_payload['image']).save(path)
        self.notice.setText('已保存：' + str(path))

    def closeEvent(self, event):
        if self.closing:
            event.accept()
            return
        self.closing = True
        self.timer.stop()
        QtCore.QMetaObject.invokeMethod(self.worker, 'shutdown', QtCore.Qt.QueuedConnection)
        self.thread.wait()
        event.accept()


def main():
    parser = argparse.ArgumentParser(description='双列 20 机柜 + 手动按钮门')
    parser.add_argument('--scene-dir', type=Path, default=ROOT)
    args = parser.parse_args()
    app = QtWidgets.QApplication([])
    app.setStyle('Fusion')
    font = app.font()
    font.setPointSize(10)
    app.setFont(font)
    window = RoomWindow(args.scene_dir)
    window.show()
    signal.signal(signal.SIGINT, lambda *unused: window.close())
    timer = QtCore.QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(250)
    app.exec_()


if __name__ == '__main__':
    main()
