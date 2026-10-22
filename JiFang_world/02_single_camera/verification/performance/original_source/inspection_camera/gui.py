from dataclasses import asdict
from pathlib import Path
import json
import math
import time
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
from .core import Settings, mujoco
from .experiments import save_snapshot, save_scan


class ImagePane(QtWidgets.QLabel):
    def __init__(self):
        super().__init__()
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setMinimumSize(320, 240)
        self.setStyleSheet('background:#151b23;color:#ddd;border:1px solid #475463;')
        self.image = None
        self.zoom_camera = None
        self.last_point = None

    def display(self, rgb, detections=()):
        h, w = rgb.shape[:2]
        image = QtGui.QImage(rgb.data, w, h, rgb.strides[0], QtGui.QImage.Format_RGB888).copy()
        if detections:
            painter = QtGui.QPainter(image)
            painter.setPen(QtGui.QPen(QtGui.QColor('#00ff88'), max(2, w // 700)))
            font = QtGui.QFont('Sans', max(12, w // 95))
            painter.setFont(font)
            for detection in detections:
                points = [QtCore.QPointF(float(x), float(y)) for x, y in detection['corners_px']]
                painter.drawPolygon(QtGui.QPolygonF(points))
                painter.drawText(points[0] + QtCore.QPointF(0, -5), f"#{detection['index']} {detection['payload']} m")
            painter.end()
        self.image = QtGui.QPixmap.fromImage(image)
        self._scale()

    def _scale(self):
        if self.image:
            self.setPixmap(self.image.scaled(self.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))

    def resizeEvent(self, event):
        self._scale()
        super().resizeEvent(event)

    def mousePressEvent(self, event):
        self.last_point = event.pos()

    def mouseMoveEvent(self, event):
        if self.zoom_camera is not None and self.last_point is not None:
            delta = event.pos() - self.last_point
            self.zoom_camera.azimuth -= delta.x() * .4
            self.zoom_camera.elevation = float(np.clip(self.zoom_camera.elevation - delta.y() * .3, -89.9, -2))
            self.last_point = event.pos()

    def mouseReleaseEvent(self, event):
        self.last_point = None

    def wheelEvent(self, event):
        if self.zoom_camera is not None:
            self.zoom_camera.distance = float(np.clip(self.zoom_camera.distance * math.exp(-event.angleDelta().y() / 1200), .8, 12))


class Window(QtWidgets.QMainWindow):
    def __init__(self, engine, node=None, executor=None, output=None):
        super().__init__()
        self.engine, self.node, self.executor = engine, node, executor
        self.output = Path(output)
        self.last_image = self.last_result = None
        self.scan_indices = []
        self.scan_records = []
        self.scan_settings = None
        self.closing = False
        self.refreshing = False
        self.overview_renderer = None
        self.setWindowTitle('机房相机视角实验室 — MuJoCo / ROS 2')
        self.resize(1500, 980)
        self.controls = {}
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)
        panel = QtWidgets.QWidget()
        panel.setMaximumWidth(390)
        root.addWidget(panel)
        controls_layout = QtWidgets.QVBoxLayout(panel)
        title = QtWidgets.QLabel('机房巡检相机实验')
        title.setStyleSheet('font-size:20px;font-weight:bold;padding:8px')
        controls_layout.addWidget(title)
        self.parameters_box = QtWidgets.QWidget()
        form = QtWidgets.QFormLayout(self.parameters_box)
        controls_layout.addWidget(self.parameters_box)
        self.add_slider(form, 'x_m', '水平位置 / m', -1., 1., .01)
        self.add_slider(form, 'height_m', '相机高度 / m', .6, 1.8, .01)
        self.add_slider(form, 'yaw_deg', '左右 yaw / °', -90., 90., .5)
        self.add_slider(form, 'pitch_deg', '俯仰 pitch / °', -80., 80., .5)
        hint = QtWidgets.QLabel('距机柜前框 0.60 m；yaw=0 朝向机柜\nyaw 正值向左；pitch 正值向上')
        hint.setWordWrap(True)
        form.addRow(hint)
        self.mode = QtWidgets.QComboBox()
        self.mode.addItem('按视场角配置', 'fov')
        self.mode.addItem('按传感器＋焦距配置', 'physical')
        self.mode.currentIndexChanged.connect(lambda: self.change('lens_mode', self.mode.currentData()))
        form.addRow('镜头模式', self.mode)
        self.add_spin(form, 'hfov_deg', '水平视场角 / °', 10, 150, 1.)
        self.add_spin(form, 'sensor_width_mm', '有效传感器宽 / mm', .5, 50, .1)
        self.add_spin(form, 'sensor_height_mm', '有效传感器高 / mm', .5, 50, .1)
        self.add_spin(form, 'focal_mm', '焦距 / mm', .3, 200, .1)
        self.presets = QtWidgets.QComboBox()
        for label, dims in [('选择分辨率预设', None), ('1920×1080 横向', (1920,1080)), ('3840×2160 横向 / 4K', (3840,2160)), ('1080×1920 竖向', (1080,1920)), ('1280×720 快速预览', (1280,720))]:
            self.presets.addItem(label, dims)
        self.presets.currentIndexChanged.connect(self.preset)
        form.addRow('分辨率预设', self.presets)
        for name, label in [('width','图像宽 / px'), ('height','图像高 / px')]:
            spin = QtWidgets.QSpinBox()
            spin.setRange(240, 4096)
            spin.setSingleStep(16)
            spin.setKeyboardTracking(False)
            spin.valueChanged.connect(lambda value, key=name: self.change(key, int(value)))
            self.controls[name] = spin
            form.addRow(label, spin)
        self.add_spin(form, 'fps', '目标帧率 / Hz', 1, 30, 1.)
        self.decode = QtWidgets.QCheckBox('实时二维码识别')
        self.decode.toggled.connect(lambda value: self.change('detect_enabled', value))
        form.addRow(self.decode)
        aimrow = QtWidgets.QHBoxLayout()
        self.aim_index = QtWidgets.QSpinBox(); self.aim_index.setRange(1,35)
        aimrow.addWidget(QtWidgets.QLabel('机箱编号'));aimrow.addWidget(self.aim_index)
        aimbutton = QtWidgets.QPushButton('移至并对准');aimbutton.clicked.connect(self.aim_target);aimrow.addWidget(aimbutton)
        form.addRow(aimrow)
        buttons = QtWidgets.QGridLayout();controls_layout.addLayout(buttons)
        self.manual_buttons=[]
        for index, (text, handler) in enumerate([('复位相机',self.reset),('清空累计统计',self.reset_coverage),('保存图像与配置',self.snapshot),('载入配置',self.load_config)]):
            button=QtWidgets.QPushButton(text);button.clicked.connect(handler);buttons.addWidget(button,index//2,index%2)
            self.manual_buttons.append(button)
        self.scan_button=QtWidgets.QPushButton('自动巡检 35 个机箱')
        self.scan_button.clicked.connect(self.toggle_scan);controls_layout.addWidget(self.scan_button)
        self.stats=QtWidgets.QLabel();self.stats.setWordWrap(True);self.stats.setStyleSheet('padding:8px;background:#e8eff7;color:#16253b;');controls_layout.addWidget(self.stats)
        self.notice=QtWidgets.QLabel('图像为理想针孔模型；识别结果来自实际渲染图。');self.notice.setWordWrap(True);controls_layout.addWidget(self.notice)
        controls_layout.addStretch()
        right=QtWidgets.QVBoxLayout();root.addLayout(right,1)
        right.addWidget(QtWidgets.QLabel('相机画面（绿框为解码成功的二维码）'))
        self.image_pane=ImagePane();right.addWidget(self.image_pane,3)
        overview_header=QtWidgets.QHBoxLayout()
        overview_header.addWidget(QtWidgets.QLabel('场景：绿线为水平轨道，黄线为视场锥；拖动旋转，滚轮缩放'))
        view=QtWidgets.QComboBox();view.addItems(['斜视','俯视','正视']);view.currentIndexChanged.connect(self.view_changed);overview_header.addWidget(view);right.addLayout(overview_header)
        self.overview_pane=ImagePane();right.addWidget(self.overview_pane,2)
        self.overview_camera=mujoco.MjvCamera()
        self.view_changed(0)
        self.overview_pane.zoom_camera=self.overview_camera
        self.statusBar().showMessage('ROS 2 已连接' if node else '独立模式（未发布 ROS 2）')
        if self.node:
            self.node.on_settings_changed=self.sync_controls
            self.node.on_scan_requested=self.start_scan
        self.sync_controls()
        self.timer=QtCore.QTimer(self);self.timer.timeout.connect(self.tick);self.timer.start(20)
        self.next_capture=0.

    def add_spin(self, form, name, label, low, high, step):
        spin=QtWidgets.QDoubleSpinBox();spin.setRange(low,high);spin.setDecimals(3);spin.setSingleStep(step);spin.setKeyboardTracking(False)
        spin.valueChanged.connect(lambda value:self.change(name,float(value)))
        self.controls[name]=spin;form.addRow(label,spin)
        return spin

    def add_slider(self, form, name, label, low, high, step):
        row=QtWidgets.QHBoxLayout()
        slider=QtWidgets.QSlider(QtCore.Qt.Horizontal);slider.setRange(round(low/step),round(high/step));row.addWidget(slider)
        spin=QtWidgets.QDoubleSpinBox();spin.setRange(low,high);spin.setDecimals(3 if step<.1 else 1);spin.setSingleStep(step);spin.setKeyboardTracking(False);spin.setMaximumWidth(95);row.addWidget(spin)
        slider.valueChanged.connect(lambda value:spin.setValue(value*step))
        def update_slider(value):
            slider.blockSignals(True);slider.setValue(round(value/step));slider.blockSignals(False)
        spin.valueChanged.connect(update_slider)
        spin.valueChanged.connect(lambda value:self.change(name,float(value)))
        self.controls[name]=spin
        self.controls[name+'_slider']=(slider,step)
        form.addRow(label,row)

    def set_settings(self, values, internal=False):
        if self.node:
            self.node.set_settings(values,internal=internal)
        else:
            self.engine.apply(self.engine.settings.updated(values))
            self.sync_controls()
        self.next_capture=0.

    def change(self, name, value):
        if self.refreshing:return
        try:self.set_settings({name:value})
        except (ValueError,TypeError) as error:
            self.notice.setText('参数未应用：'+str(error));self.sync_controls()

    def sync_controls(self):
        self.refreshing=True
        s=self.engine.settings
        for key,widget in self.controls.items():
            if key.endswith('_slider'):
                slider,step=widget;slider.blockSignals(True);slider.setValue(round(getattr(s,key[:-7])/step));slider.blockSignals(False)
            else:
                widget.blockSignals(True);widget.setValue(getattr(s,key));widget.blockSignals(False)
        self.mode.blockSignals(True);self.mode.setCurrentIndex(0 if s.lens_mode=='fov' else 1);self.mode.blockSignals(False)
        self.decode.blockSignals(True);self.decode.setChecked(s.detect_enabled);self.decode.blockSignals(False)
        self.controls['hfov_deg'].setEnabled(s.lens_mode=='fov')
        for name in ['sensor_width_mm','sensor_height_mm','focal_mm']:self.controls[name].setEnabled(s.lens_mode=='physical')
        self.refreshing=False

    def preset(self):
        dims=self.presets.currentData()
        if not dims:return
        s=self.engine.settings
        values=dict(width=dims[0],height=dims[1])
        if (s.width>s.height)!=(dims[0]>dims[1]):
            # Portrait output stays upright; swap physical active sensor axes.
            if s.lens_mode=='physical':values.update(sensor_width_mm=s.sensor_height_mm,sensor_height_mm=s.sensor_width_mm)
            else:values['hfov_deg']=s.fovs()[1]
        try:self.set_settings(values)
        except ValueError as error:self.notice.setText(str(error))
        self.presets.blockSignals(True);self.presets.setCurrentIndex(0);self.presets.blockSignals(False)

    def aim_target(self):
        self.set_settings(asdict(self.engine.aim(self.aim_index.value())))

    def reset(self):
        self.set_settings(asdict(Settings()))

    def reset_coverage(self):
        self.engine.cumulative.clear()

    def view_changed(self,index):
        if not hasattr(self,'overview_camera'):return
        c=self.overview_camera;c.type=mujoco.mjtCamera.mjCAMERA_FREE
        c.lookat[:]=[0,-.3,1.05];c.distance=4.5
        c.azimuth,c.elevation=[(120,-18),(90,-89.9),(90,-4)][index]
        if index==1:c.lookat[:]=[0,-.3,.3]

    def snapshot(self):
        if self.last_image is None:return
        directory=save_snapshot(self.output,self.last_image,self.last_result)
        self.notice.setText('已保存：'+str(directory))

    def load_config(self):
        filename,_=QtWidgets.QFileDialog.getOpenFileName(self,'载入相机配置',str(self.output),'JSON (*.json)')
        if not filename:return
        try:
            document=json.loads(Path(filename).read_text())
            self.set_settings(document.get('settings',document))
        except (ValueError,TypeError,OSError) as error:self.notice.setText('载入失败：'+str(error))

    def start_scan(self):
        if self.scan_indices:return False,'扫描正在进行'
        if not self.engine.settings.detect_enabled:return False,'请先启用二维码识别'
        self.scan_indices=list(range(1,36));self.scan_records=[];self.scan_settings=self.engine.settings
        self.engine.cumulative.clear();self.parameters_box.setEnabled(False);self.scan_button.setText('停止自动巡检')
        for button in self.manual_buttons:button.setEnabled(False)
        if self.node:self.node.command_locked=True
        self.next_capture=0.
        return True,'已开始巡检；将沿二维码高度逐点采样'

    def toggle_scan(self):
        if self.scan_indices:self.finish_scan(cancelled=True)
        else:
            ok,message=self.start_scan();self.notice.setText(message)

    def finish_scan(self,cancelled=False):
        self.scan_indices=[];self.parameters_box.setEnabled(True);self.scan_button.setText('自动巡检 35 个机箱')
        for button in self.manual_buttons:button.setEnabled(True)
        if self.node:self.node.command_locked=False
        directory,summary=save_scan(self.output,self.scan_records,self.scan_settings,label='scan_cancelled' if cancelled else 'scan')
        self.notice.setText(f"{'已停止' if cancelled else '巡检完成'}：{len(summary['decoded_indices'])}/35；报告：{directory}")

    def tick(self):
        if self.closing:return
        if self.executor:
            for _ in range(4):self.executor.spin_once(timeout_sec=0)
        now=time.perf_counter()
        if now<self.next_capture:return
        try:
            scanning=bool(self.scan_indices)
            if scanning:self.set_settings(asdict(self.engine.aim(self.scan_indices[0])),internal=True)
            image,result=self.engine.capture()
            self.last_image,self.last_result=image,result
            if self.node:self.node.publish(image,result)
            self.image_pane.display(image,result['detections'])
            if self.overview_renderer is None:self.overview_renderer=mujoco.Renderer(self.engine.model,height=400,width=800)
            self.overview_renderer.update_scene(self.engine.data,camera=self.overview_camera)
            self.engine.add_overview_markers(self.overview_renderer.scene)
            self.overview_pane.display(self.overview_renderer.render().copy())
            count=len(result['decoded_indices']);complete=sum(m['complete_in_frame'] for m in result['metrics'])
            self.stats.setText(f"视场角 H {result['hfov_deg']:.1f}° / V {result['vfov_deg']:.1f}°\n画幅内完整二维码：{complete}/35\n当前解码：{count}/35；累计：{len(result['cumulative_indices'])}/35\n漏扫编号：{', '.join(map(str,result['missing_indices'])) or '无'}\n渲染＋识别：{result['processing_seconds']:.3f} 秒/帧")
            if scanning:
                self.scan_records.append(result);self.scan_indices.pop(0)
                if self.scan_indices:self.notice.setText(f'自动巡检：已完成 {35-len(self.scan_indices)}/35 个采样位姿')
                else:self.finish_scan()
            self.statusBar().showMessage(f"{'ROS 2 已发布' if self.node else '独立模式'} | {self.engine.settings.width}×{self.engine.settings.height} | 输出目录：{self.output}")
        except Exception as error:
            self.notice.setText('运行错误：'+str(error))
            if self.scan_indices:self.finish_scan(cancelled=True)
        self.next_capture=time.perf_counter()+max(0.,1/self.engine.settings.fps-(time.perf_counter()-now))

    def closeEvent(self,event):
        self.closing=True;self.timer.stop()
        if self.scan_indices:self.finish_scan(cancelled=True)
        if self.overview_renderer:self.overview_renderer.close()
        self.engine.close();event.accept()
