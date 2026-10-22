from dataclasses import asdict
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import json,time,math
import numpy as np
from PyQt5 import QtCore,QtGui,QtWidgets
from PIL import Image
from .gui import ImagePane
from .robot_platform import RobotCameraEngine,RobotSettings,CAMERAS,LABELS,HEIGHT,mujoco
from .robot_worker import PlatformWorker


class RobotWindow(QtWidgets.QMainWindow):
    requested=QtCore.pyqtSignal(object)
    def __init__(self,engine,node=None,executor=None,output=None):
        super().__init__();self.engine=engine;self.node=node;self.executor=executor;self.output=Path(output)
        self.busy=False;self.closing=False;self.refreshing=False;self.last_request=float('-inf');self.last_payload=None
        self.coverage={name:set() for name in CAMERAS};self.display_key=None;self.overview_key=None
        self.active='left';self.controls={};self.selected_controls={}
        self.setWindowTitle('A2-W 站姿尺寸模型 · 顶部与双侧相机');self.resize(1520,990)
        central=QtWidgets.QWidget();self.setCentralWidget(central);layout=QtWidgets.QHBoxLayout(central)
        scroll=QtWidgets.QScrollArea();scroll.setWidgetResizable(True);scroll.setMaximumWidth(415);layout.addWidget(scroll)
        panel=QtWidgets.QWidget();scroll.setWidget(panel);left=QtWidgets.QVBoxLayout(panel)
        title=QtWidgets.QLabel('机器狗三相机评估平台');title.setStyleSheet('font-size:19px;font-weight:bold');left.addWidget(title)
        dimensions=QtWidgets.QLabel('宇树 A2-W：长 0.900 × 宽 0.440 × 高 0.625 m\n简化固定站姿；无关节、无步态、整体平移')
        dimensions.setWordWrap(True);left.addWidget(dimensions)
        self.position_label=QtWidgets.QLabel();self.position_label.setWordWrap(True);left.addWidget(self.position_label)
        box=QtWidgets.QGroupBox('整机位置与伸缩杆');form=QtWidgets.QFormLayout(box);left.addWidget(box)
        self.add_slider(form,'robot_x_m','沿柜平移 X / m',-1.2,1.2,.01)
        self.add_slider(form,'robot_y_m','前后平移 Y / m',-2.4,-.8,.01)
        self.add_slider(form,'robot_yaw_deg','整机朝向 / °',-180.,180.,1.)
        self.add_slider(form,'pole_length_m','伸缩杆总高 / m',.6,1.2,.01)
        switchrow=QtWidgets.QHBoxLayout()
        for text,side in [('左侧靠柜','left'),('右侧靠柜','right')]:
            button=QtWidgets.QPushButton(text);button.clicked.connect(lambda checked=False,value=side:self.face_side(value));switchrow.addWidget(button)
        form.addRow(switchrow)
        group=QtWidgets.QGroupBox('三台相机独立调节');form=QtWidgets.QFormLayout(group);left.addWidget(group)
        self.selector=QtWidgets.QComboBox()
        for camera in CAMERAS:self.selector.addItem(LABELS[camera],camera)
        self.selector.setCurrentIndex(1);self.selector.currentIndexChanged.connect(self.select_camera);form.addRow('当前相机',self.selector)
        for key,label,low,high in [('yaw_deg','相对机身 yaw / °',-180.,180.),('pitch_deg','俯仰 pitch / °',-80.,80.),('hfov_deg','水平视场角 / °',10.,150.)]:
            spin=QtWidgets.QDoubleSpinBox();spin.setRange(low,high);spin.setDecimals(1);spin.setSingleStep(.5);spin.setKeyboardTracking(False)
            spin.valueChanged.connect(lambda value,key=key:self.set_one(self.active+'_'+key,float(value)))
            self.selected_controls[key]=spin;form.addRow(label,spin)
        info=QtWidgets.QLabel('yaw=0 看向机身左侧；yaw=±180 看向右侧\npitch 正值抬头、负值低头；侧机高度固定 0.70 m')
        info.setWordWrap(True);form.addRow(info)
        targetrow=QtWidgets.QHBoxLayout();self.target=QtWidgets.QSpinBox();self.target.setRange(1,35)
        targetrow.addWidget(QtWidgets.QLabel('二维码编号'));targetrow.addWidget(self.target)
        button=QtWidgets.QPushButton('转向此码');button.clicked.connect(self.aim);targetrow.addWidget(button);form.addRow(targetrow)
        button=QtWidgets.QPushButton('平移对齐此码并转向');button.clicked.connect(lambda:self.aim(True));form.addRow(button)
        common=QtWidgets.QGroupBox('成像设置（三路同分辨率）');form=QtWidgets.QFormLayout(common);left.addWidget(common)
        self.resolution=QtWidgets.QComboBox()
        for label,value in [('1920×1080',(1920,1080)),('3840×2160',(3840,2160)),('1280×720 快速预览',(1280,720)),('1080×1920 竖向',(1080,1920))]:self.resolution.addItem(label,value)
        self.resolution.currentIndexChanged.connect(self.set_resolution);form.addRow('图像分辨率',self.resolution)
        fps=QtWidgets.QDoubleSpinBox();fps.setRange(1,15);fps.setDecimals(1);fps.valueChanged.connect(lambda v:self.set_one('fps',float(v)));self.controls['fps']=fps;form.addRow('目标帧率 / Hz',fps)
        self.decode=QtWidgets.QCheckBox('二维码识别');self.decode.toggled.connect(lambda value:self.set_one('detect_enabled',value));form.addRow(self.decode)
        self.reuse=QtWidgets.QCheckBox('静态画面复用（减少重复计算）');self.reuse.setChecked(True);form.addRow(self.reuse)
        self.table=QtWidgets.QTableWidget(3,4);self.table.setHorizontalHeaderLabels(['相机','高度 m','当前解码','底部 1/2/3'])
        self.table.verticalHeader().hide();self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch);self.table.setMaximumHeight(125);self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers);left.addWidget(self.table)
        self.clearance=QtWidgets.QLabel();self.clearance.setWordWrap(True);left.addWidget(self.clearance)
        buttons=QtWidgets.QGridLayout();left.addLayout(buttons)
        for index,(text,slot) in enumerate([('保存三路图像/配置',self.snapshot),('载入配置',self.load_config),('复位平台',self.reset),('清空统计',self.reset_coverage)]):
            b=QtWidgets.QPushButton(text);b.clicked.connect(slot);buttons.addWidget(b,index//2,index%2)
        self.notice=QtWidgets.QLabel('左侧靠柜时，右侧相机朝通道外；可转向二维码观察机身遮挡。');self.notice.setWordWrap(True);left.addWidget(self.notice);left.addStretch()
        right=QtWidgets.QVBoxLayout();layout.addLayout(right,1)
        self.tabs=QtWidgets.QTabWidget();self.panes={}
        for name in CAMERAS:
            pane=ImagePane();self.panes[name]=pane;self.tabs.addTab(pane,LABELS[name])
        self.tabs.setCurrentIndex(1);self.tabs.currentChanged.connect(self.tab_changed);right.addWidget(self.tabs,3)
        viewrow=QtWidgets.QHBoxLayout();viewrow.addWidget(QtWidgets.QLabel('机身/支架参与遮挡；黄=顶部，青=左侧，粉=右侧'))
        self.view=QtWidgets.QComboBox();self.view.addItems(['斜视','俯视','沿机身侧视','柜前正视']);self.view.currentIndexChanged.connect(self.set_view);viewrow.addWidget(self.view);right.addLayout(viewrow)
        self.overview=ImagePane();self.overview.setMinimumSize(600,300);right.addWidget(self.overview,2)
        self.overview_camera=mujoco.MjvCamera();self.set_view(0);self.overview.zoom_camera=self.overview_camera
        self.thread=QtCore.QThread(self);self.worker=PlatformWorker(engine.scene_dir);self.worker.moveToThread(self.thread)
        self.requested.connect(self.worker.render);self.worker.ready.connect(self.received);self.worker.failed.connect(self.failed)
        self.worker.stopped.connect(self.thread.quit,QtCore.Qt.DirectConnection);self.thread.finished.connect(self.worker.deleteLater);self.thread.start()
        if node:node.changed=self.sync;node.reset=self.reset_coverage
        self.sync();self.timer=QtCore.QTimer(self);self.timer.timeout.connect(self.tick);self.timer.start(20)

    def add_slider(self,form,name,label,lo,hi,step):
        row=QtWidgets.QHBoxLayout();slider=QtWidgets.QSlider(QtCore.Qt.Horizontal);slider.setRange(round(lo/step),round(hi/step));row.addWidget(slider)
        spin=QtWidgets.QDoubleSpinBox();spin.setRange(lo,hi);spin.setDecimals(3 if step<.1 else 1);spin.setSingleStep(step);spin.setKeyboardTracking(False);spin.setMaximumWidth(100);row.addWidget(spin)
        slider.valueChanged.connect(lambda value:spin.setValue(value*step))
        def update(value):slider.blockSignals(True);slider.setValue(round(value/step));slider.blockSignals(False)
        spin.valueChanged.connect(update);spin.valueChanged.connect(lambda value:self.set_one(name,float(value)))
        self.controls[name]=spin;self.controls[name+'_slider']=(slider,step);form.addRow(label,row)

    def set_settings(self,values):
        if self.node:self.node.set_settings(values)
        else:self.engine.apply_platform(self.engine.platform.updated(values));self.sync()
    def set_one(self,key,value):
        if self.refreshing:return
        try:self.set_settings({key:value})
        except (ValueError,TypeError) as e:self.notice.setText('参数未应用：'+str(e));self.sync()
    def sync(self):
        self.refreshing=True;s=self.engine.platform
        for name,control in self.controls.items():
            if name.endswith('_slider'):
                slider,step=control;slider.blockSignals(True);slider.setValue(round(getattr(s,name[:-7])/step));slider.blockSignals(False)
            else:control.blockSignals(True);control.setValue(getattr(s,name));control.blockSignals(False)
        for name,control in self.selected_controls.items():control.blockSignals(True);control.setValue(getattr(s,self.active+'_'+name));control.blockSignals(False)
        self.decode.blockSignals(True);self.decode.setChecked(s.detect_enabled);self.decode.blockSignals(False)
        index=next((i for i in range(self.resolution.count()) if tuple(self.resolution.itemData(i))==(s.width,s.height)),-1);self.resolution.blockSignals(True);self.resolution.setCurrentIndex(index);self.resolution.blockSignals(False)
        self.position_label.setText(f'顶部光心 {s.camera_position("top")[2]:.3f} m；双侧光心 0.700 m\n光心距机柜前框：顶 {-s.camera_position("top")[1]-.5:.3f} / 左 {-s.camera_position("left")[1]-.5:.3f} / 右 {-s.camera_position("right")[1]-.5:.3f} m')
        report=self.engine.interference_report();lowest=min(v['clearance_to_body_envelope_m'] for v in report.values())
        self.clearance.setText(f'旋转扫掠半径 50 mm；机身包络最小净距 {lowest*1000:.1f} mm\n机身与机柜接近时会拒绝相交位姿。')
        self.refreshing=False
    def select_camera(self):
        self.active=self.selector.currentData();self.tabs.blockSignals(True);self.tabs.setCurrentIndex(CAMERAS.index(self.active));self.tabs.blockSignals(False);self.sync()
    def tab_changed(self,index):
        self.active=CAMERAS[index];self.selector.blockSignals(True);self.selector.setCurrentIndex(index);self.selector.blockSignals(False);self.sync()
    def set_resolution(self):
        if self.refreshing:return
        dims=self.resolution.currentData()
        if dims:
            try:self.set_settings(dict(width=dims[0],height=dims[1]))
            except ValueError as e:self.notice.setText(str(e));self.sync()
    def face_side(self,side):
        try:
            values=dict(robot_yaw_deg=0. if side=='left' else 180.,top_yaw_deg=0. if side=='left' else 180.)
            self.set_settings(values)
            self.selector.setCurrentIndex(CAMERAS.index(side))
        except ValueError as e:self.notice.setText(str(e))
    def aim(self,align=False):
        try:self.set_settings(asdict(self.engine.aim(self.active,self.target.value(),align_robot=align)))
        except ValueError as e:self.notice.setText(str(e))
    def set_view(self,index):
        if not hasattr(self,'overview_camera'):return
        c=self.overview_camera;c.azimuth,c.elevation,c.distance=[(120,-20,4.5),(90,-89.9,4.),(0,-12,4.),(90,-8,4.)][index]
    def reset(self):self.set_settings(asdict(RobotSettings()))
    def reset_coverage(self):
        for values in self.coverage.values():values.clear()
    def load_config(self):
        file,_=QtWidgets.QFileDialog.getOpenFileName(self,'载入平台配置',str(self.output),'JSON (*.json)')
        if file:
            try:
                values=json.loads(Path(file).read_text());self.set_settings(values.get('platform_settings',values))
            except (ValueError,TypeError,OSError) as e:self.notice.setText('载入失败：'+str(e))
    def snapshot(self):
        if not self.last_payload:return
        stamp=datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f')
        directory=self.output/('platform_'+stamp);directory.mkdir(parents=True)
        payload=self.last_payload
        for name in CAMERAS:
            Image.fromarray(payload['images'][name]).save(directory/(name+'.png'))
            (directory/(name+'_result.json')).write_text(json.dumps(payload['results'][name],ensure_ascii=False,indent=2)+'\n')
        (directory/'settings.json').write_text(json.dumps(payload['results']['top']['platform_settings'],indent=2)+'\n')
        Image.fromarray(payload['overview']).save(directory/'overview.png')
        self.notice.setText('已保存：'+str(directory))
    def tick(self):
        if self.closing:return
        if self.executor:
            for _ in range(4):self.executor.spin_once(timeout_sec=0)
        now=time.perf_counter()
        if self.busy or now-self.last_request<1/self.engine.platform.fps:return
        c=self.overview_camera
        request=dict(settings=asdict(self.engine.platform),reuse=self.reuse.isChecked(),overview=dict(azimuth=float(c.azimuth),elevation=float(c.elevation),distance=float(c.distance)))
        self.busy=True;self.last_request=now;self.requested.emit(request)
    @QtCore.pyqtSlot(object)
    def received(self,payload):
        self.busy=False
        if self.closing:return
        try:
            self.last_payload=payload
            for name in CAMERAS:
                result=payload['results'][name];self.coverage[name].update(result['decoded_indices']);result['cumulative_indices']=sorted(self.coverage[name])
            if self.node:self.node.publish(payload['images'],payload['results'])
            key=tuple(payload['request']['settings'].items())
            if key!=self.display_key or not payload['cached']:
                for name in CAMERAS:self.panes[name].display(payload['images'][name],payload['results'][name]['detections'])
                self.display_key=key
            if self.overview_key!=payload['overview_key']:self.overview.display(payload['overview']);self.overview_key=payload['overview_key']
            for row,name in enumerate(CAMERAS):
                result=payload['results'][name]
                values=[LABELS[name],f"{result['camera_position_m'][2]:.3f}",str(len(result['decoded_indices'])),','.join(map(str,result['bottom_three_decoded'])) or '未读出']
                for col,value in enumerate(values):self.table.setItem(row,col,QtWidgets.QTableWidgetItem(value))
            self.statusBar().showMessage(f"{payload['renderer']} | {'静态复用' if payload['cached'] else '三路新图像'} | 后台 {payload['seconds']:.3f} s | {'ROS 2 已连接' if self.node else '独立模式'}")
        except Exception as e:self.failed(str(e))
    @QtCore.pyqtSlot(str)
    def failed(self,message):
        self.busy=False;self.notice.setText('运行错误：'+message);self.last_request=time.perf_counter()+1
    def closeEvent(self,event):
        if self.closing:
            event.accept();return
        self.closing=True;self.timer.stop();QtCore.QMetaObject.invokeMethod(self.worker,'shutdown',QtCore.Qt.QueuedConnection);self.thread.wait();self.engine.close();event.accept()
