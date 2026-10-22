"""Standing A2-W dimensional proxy and three cameras (no articulated joints)."""
from dataclasses import dataclass, asdict, fields, replace
from pathlib import Path
import json
import math
import xml.etree.ElementTree as ET
import numpy as np
from .core import CameraEngine, Settings, NATIVE_IN_LINK, OPTICAL_IN_LINK, quaternion, mujoco

CAMERAS = ('top', 'left', 'right')
LABELS = {'top':'顶部升降相机', 'left':'左侧低位相机', 'right':'右侧低位相机'}
LENGTH, WIDTH, HEIGHT = .900, .440, .625
SIDE_OFFSET = .320
SIDE_HEIGHT = .700
CAMERA_SWEEP_RADIUS = .050
SOURCES = {'manufacturer':'Unitree', 'model':'A2-W', 'url':'https://www.unitree.com/A2-W/',
           'standing_size_m':[LENGTH,WIDTH,HEIGHT], 'tyre_diameter_m':.190, 'tyre_width_m':.051,
           'checked_on':'2026-10-21', 'geometry_note':'Only overall size and tyre size are manufacturer dimensions; torso, legs, brackets and camera housing are simplified design assumptions.'}


def rz(degrees):
    a=math.radians(degrees)
    return np.array([[math.cos(a),-math.sin(a),0.],[math.sin(a),math.cos(a),0.],[0.,0.,1.]])


@dataclass
class RobotSettings:
    robot_x_m: float = -.170
    robot_y_m: float = -1.420
    robot_yaw_deg: float = 0.0
    pole_length_m: float = .600
    top_yaw_deg: float = 0.0
    top_pitch_deg: float = -35.0
    left_yaw_deg: float = 0.0
    left_pitch_deg: float = -40.0
    right_yaw_deg: float = 180.0
    right_pitch_deg: float = -40.0
    top_hfov_deg: float = 60.0
    left_hfov_deg: float = 60.0
    right_hfov_deg: float = 60.0
    width: int = 1920
    height: int = 1080
    fps: float = 3.0
    detect_enabled: bool = True

    def validate(self):
        for field in fields(self):
            value=getattr(self,field.name)
            if isinstance(value,(float,int)) and not math.isfinite(value):raise ValueError(f'{field.name} 必须为有限数值')
        for key,low,high in [('robot_x_m',-1.2,1.2),('robot_y_m',-2.4,-.80),('robot_yaw_deg',-180.,180.),('pole_length_m',.6,1.2),('fps',1.,15.)]:
            if not low<=getattr(self,key)<=high:raise ValueError(f'{key} 范围 [{low}, {high}]')
        for camera in CAMERAS:
            for suffix,low,high in [('yaw_deg',-180.,180.),('pitch_deg',-80.,80.),('hfov_deg',10.,150.)]:
                key=f'{camera}_{suffix}'
                if not low<=getattr(self,key)<=high:raise ValueError(f'{key} 范围 [{low}, {high}]')
        if type(self.detect_enabled) is not bool:raise ValueError('detect_enabled 必须为布尔值')
        for key in ['width','height']:
            if type(getattr(self,key)) is not int or not 240<=getattr(self,key)<=4096:raise ValueError(f'{key} 必须为 240–4096 的整数')
        # Keep the platform and camera swept volumes clear of cabinet and room.
        r=rz(self.robot_yaw_deg)
        points=np.array([[x,y,0.] for x in [-LENGTH/2,LENGTH/2] for y in [-WIDTH/2,WIDTH/2]])@r.T
        points[:,:2]+=[self.robot_x_m,self.robot_y_m]
        if min(points[:,0])< -2.04:raise ValueError('机器狗包络与左侧墙体干涉')
        xlo,xhi=float(min(points[:,0])),float(max(points[:,0]))
        ylo,yhi=float(min(points[:,1])),float(max(points[:,1]))
        if xlo<.30 and xhi>-.30 and yhi>-.515 and ylo<.50:raise ValueError('机器狗站姿包络与机柜干涉，请后退或平移')
        for camera in CAMERAS:
            p=self.camera_position(camera)
            if p[0]-CAMERA_SWEEP_RADIUS<-.2-1.84:raise ValueError('相机旋转包络与墙体干涉')
            distance=np.linalg.norm(np.maximum(np.abs(p[:2])-[.3,.5],0.))
            if p[2]<=2.177 and distance<CAMERA_SWEEP_RADIUS+.01:raise ValueError('相机旋转包络与机柜干涉')
        return self

    def updated(self,values):
        unknown=set(values)-set(asdict(self))
        if unknown:raise ValueError('未知参数: '+', '.join(sorted(unknown)))
        return replace(self,**values).validate()

    def base_rotation(self):return rz(self.robot_yaw_deg)
    def camera_local_position(self,camera):
        return np.array([0.,0. if camera=='top' else (SIDE_OFFSET if camera=='left' else -SIDE_OFFSET),
                         HEIGHT+self.pole_length_m+.1 if camera=='top' else SIDE_HEIGHT])
    def camera_position(self,camera):return np.array([self.robot_x_m,self.robot_y_m,0.])+self.base_rotation()@self.camera_local_position(camera)
    def camera_local_rotation(self,camera):
        return Settings(yaw_deg=getattr(self,camera+'_yaw_deg'),pitch_deg=getattr(self,camera+'_pitch_deg')).rotation_link()
    def camera_rotation(self,camera):return self.base_rotation()@self.camera_local_rotation(camera)


class ViewSettings(Settings):
    def __init__(self,platform,camera):
        p=platform.camera_position(camera)
        super().__init__(x_m=float(p[0]),height_m=float(p[2]),yaw_deg=getattr(platform,camera+'_yaw_deg'),pitch_deg=getattr(platform,camera+'_pitch_deg'),
                         width=platform.width,height=platform.height,hfov_deg=getattr(platform,camera+'_hfov_deg'),fps=platform.fps,detect_enabled=platform.detect_enabled)
        self._position=p
        self._rotation=platform.camera_rotation(camera)
    def position(self):return self._position.copy()
    def rotation_link(self):return self._rotation.copy()


def element(parent,tag,**values):
    return ET.SubElement(parent,tag,{k:(' '.join(map(str,v)) if isinstance(v,(tuple,list,np.ndarray)) else str(v)) for k,v in values.items()})


def platform_xml(scene_dir,settings):
    scene_dir=Path(scene_dir).resolve()
    root=ET.parse(scene_dir/'scene.xml').getroot()
    root.set('model','JiFang_A2W_three_cameras')
    root.find('compiler').set('texturedir',str(scene_dir/'assets'))
    for include in root.iter('include'):include.set('file',str(scene_dir/include.get('file')))
    visual=root.find('visual/global');visual.set('offwidth',str(max(settings.width,800)));visual.set('offheight',str(max(settings.height,450)))
    root.find('statistic').set('center','0 -0.65 1.0');root.find('statistic').set('extent','2.5')
    world=root.find('worldbody')
    element(world,'geom',name='robot_translation_track',type='capsule',fromto='-1.2 -1.42 .008 1.2 -1.42 .008',size='.004',rgba='.1 .9 .3 1',contype=0,conaffinity=0,group=2)
    body=element(world,'body',name='robot_base',mocap='true',pos=[settings.robot_x_m,settings.robot_y_m,0.],quat=quaternion(settings.base_rotation()))
    def box(name,pos,size,color,group=1):
        return element(body,'geom',name=name,type='box',pos=pos,size=size,rgba=color,contype=0,conaffinity=0,group=group)
    box('robot_torso',[0,0,.503],[.37,.16,.098],'.24 .28 .34 1')
    box('robot_roof',[0,0,.613],[.32,.155,.012],'.55 .60 .64 1')
    box('robot_front_marker',[.374,0,.52],[.004,.10,.035],'.12 .64 .9 1')
    for x in [-.355,.355]:
        for y in [-.1945,.1945]:
            name=('front' if x>0 else 'rear')+('_left' if y>0 else '_right')
            element(body,'geom',name='wheel_'+name,type='cylinder',pos=[x,y,.095],size=[.095,.0255],quat=[.707106781,.707106781,0,0],rgba='.035 .043 .052 1',contype=0,conaffinity=0,group=1)
            knee=[x*.84,y*.8,.27]
            for index,(a,b) in enumerate([([x*.8,y*.74,.455],knee),(knee,[x,y,.11])]):
                element(body,'geom',name=f'fixed_leg_{name}_{index}',type='capsule',fromto=[*a,*b],size='.022',rgba='.35 .40 .45 1',contype=0,conaffinity=0,group=1)
    corners=[[x,y,z] for x in [-LENGTH/2,LENGTH/2] for y in [-WIDTH/2,WIDTH/2] for z in [0.,HEIGHT]]
    edge=0
    for i,a in enumerate(corners):
        for b in corners[i+1:]:
            if sum(abs(x-y)>1e-8 for x,y in zip(a,b))==1:
                element(body,'geom',name=f'robot_dimension_edge_{edge}',type='capsule',fromto=[*a,*b],size='.0015',rgba='.7 .76 .82 .7',contype=0,conaffinity=0,group=2)
                edge+=1
    # These struts are real occluders, not hidden diagram elements.
    for side,sign in [('left',1),('right',-1)]:
        element(body,'geom',name=side+'_mount_arm',type='capsule',fromto=[0,sign*.14,.59,0,sign*SIDE_OFFSET,.59],size='.012',rgba='.7 .73 .76 1',contype=0,conaffinity=0,group=1)
        element(body,'geom',name=side+'_mount_riser',type='capsule',fromto=[0,sign*SIDE_OFFSET,.59,0,sign*SIDE_OFFSET,.64],size='.012',rgba='.7 .73 .76 1',contype=0,conaffinity=0,group=1)
    box('mast_base',[0,0,.64],[.06,.055,.015],'.15 .17 .2 1')
    element(body,'geom',name='telescopic_pole',type='cylinder',pos=[0,0,HEIGHT+settings.pole_length_m/2],size=[.022,settings.pole_length_m/2],rgba='.63 .67 .72 1',contype=0,conaffinity=0,group=1)
    element(body,'geom',name='top_gimbal_riser',type='cylinder',pos=[0,0,HEIGHT+settings.pole_length_m+.025],size=[.025,.025],rgba='.22 .25 .3 1',contype=0,conaffinity=0,group=1)
    for camera,color in [('top','1 .68 .04 1'),('left','.08 .8 .9 1'),('right','.9 .35 .5 1')]:
        mount=element(body,'body',name=camera+'_camera_mount',pos=settings.camera_local_position(camera),quat=quaternion(settings.camera_local_rotation(camera)))
        # The entire rotating package fits inside a 50 mm radius sphere. Hidden
        # from sensors to avoid self-imaging; robot torso/legs/mast remain visible.
        element(mount,'geom',name=camera+'_camera_housing',type='box',pos=[-.019,0,0],size=[.019,.020,.020],rgba=color,contype=0,conaffinity=0,group=2)
        element(mount,'geom',name=camera+'_camera_lens',type='cylinder',pos=[.002,0,0],size=[.012,.004],quat=[.707106781,0,.707106781,0],rgba='.02 .04 .06 1',contype=0,conaffinity=0,group=2)
        k=ViewSettings(settings,camera).intrinsics()
        element(mount,'camera',name=camera+'_inspection_camera',quat=quaternion(NATIVE_IN_LINK),sensorsize=[settings.width/k[0,0],settings.height/k[1,1]],resolution=[settings.width,settings.height],focal=[1.,1.])
    return root


class RobotCameraEngine(CameraEngine):
    def __init__(self,scene_dir,settings=None):
        self.scene_dir=Path(scene_dir).resolve();self.platform=settings or RobotSettings();self.platform.validate()
        self.rows=json.loads((self.scene_dir/'server_heights.json').read_text())['servers']
        self.payload_to_row={r['qr_payload']:r for r in self.rows}
        self.cumulative=set();self.camera_cumulative={name:set() for name in CAMERAS}
        self.model=mujoco.MjModel.from_xml_string(ET.tostring(platform_xml(self.scene_dir,self.platform),encoding='unicode'))
        self.data=mujoco.MjData(self.model)
        self.renderer=None;self.renderer_size=None
        self.render_option=mujoco.MjvOption();self.render_option.geomgroup[2]=0;self.render_option.geomgroup[3]=0
        self.active_camera='top'
        self.apply_platform(self.platform)

    def apply_platform(self,settings):
        settings.validate();self.platform=settings
        mocap=self.model.body('robot_base').mocapid[0]
        self.data.mocap_pos[mocap]=[settings.robot_x_m,settings.robot_y_m,0.]
        self.data.mocap_quat[mocap]=quaternion(settings.base_rotation())
        pole=self.model.geom('telescopic_pole').id
        self.model.geom_pos[pole]=[0,0,HEIGHT+settings.pole_length_m/2]
        self.model.geom_size[pole,1]=settings.pole_length_m/2
        riser=self.model.geom('top_gimbal_riser').id
        self.model.geom_pos[riser]=[0,0,HEIGHT+settings.pole_length_m+.025]
        for name in CAMERAS:
            body=self.model.body(name+'_camera_mount').id
            self.model.body_pos[body]=settings.camera_local_position(name)
            self.model.body_quat[body]=quaternion(settings.camera_local_rotation(name))
            view=ViewSettings(settings,name);k=view.intrinsics();camera=self.model.camera(name+'_inspection_camera').id
            self.model.cam_sensorsize[camera]=[view.width/k[0,0],view.height/k[1,1]]
            self.model.cam_intrinsic[camera]=[1,1,0,0]
            self.model.cam_resolution[camera]=[view.width,view.height];self.model.cam_fovy[camera]=view.fovs()[1]
        self.select_camera(self.active_camera)
        mujoco.mj_forward(self.model,self.data)

    def select_camera(self,name):
        if name not in CAMERAS:raise ValueError(name)
        self.active_camera=name;self.settings=ViewSettings(self.platform,name)
        self.camera_id=self.model.camera(name+'_inspection_camera').id

    def capture_camera(self,name):
        self.select_camera(name)
        # CameraEngine's camera name is fixed, but geometry/decoder are reusable.
        image,result=self._capture()
        self.camera_cumulative[name].update(result['decoded_indices'])
        result.update(camera=name,platform_settings=asdict(self.platform),camera_position_m=self.settings.position().tolist(),
                      camera_rotation_world=self.settings.rotation_link().tolist(),
                      camera_quaternion_wxyz=quaternion(self.settings.rotation_link()).tolist(),
                      local_position_m=self.platform.camera_local_position(name).tolist(),
                      local_rotation=self.platform.camera_local_rotation(name).tolist(),
                      camera_y_m=float(self.settings.position()[1]), bottom_three_decoded=[i for i in [1,2,3] if i in result['decoded_indices']],
                      body_dimensions_m=[LENGTH,WIDTH,HEIGHT])
        return image,result

    def _capture(self):
        # Temporarily use the selected camera ID, retaining original full-resolution
        # image/ZXing pipeline and the same projection/occlusion metrics.
        import time
        start=time.perf_counter();s=self.settings
        if self.renderer_size!=(s.width,s.height):
            if self.renderer:self.renderer.close()
            self.model.vis.global_.offwidth=max(800,s.width);self.model.vis.global_.offheight=max(450,s.height)
            self.renderer=mujoco.Renderer(self.model,height=s.height,width=s.width);self.renderer_size=(s.width,s.height)
        self.renderer.update_scene(self.data,camera=self.camera_id,scene_option=self.render_option)
        image=self.renderer.render().copy();rendered=time.perf_counter();detections=[]
        if s.detect_enabled:
            for code in __import__('zxingcpp').read_barcodes(image):
                row=self.payload_to_row.get(code.text)
                p=code.position
                detections.append(dict(payload=code.text,index=row['index'] if row else None,height_m=row['centre_m'] if row else None,
                                       corners_px=[[a.x,a.y] for a in [p.top_left,p.top_right,p.bottom_right,p.bottom_left]]))
        decoded=time.perf_counter();metrics=self.geometry();end=time.perf_counter()
        ids=sorted({d['index'] for d in detections if d['index'] is not None})
        return image,dict(settings=asdict(s),intrinsics=s.intrinsics().tolist(),hfov_deg=s.fovs()[0],vfov_deg=s.fovs()[1],detections=detections,metrics=metrics,decoded_indices=ids,
                          processing_seconds=round(end-start,4),timings_ms=dict(render=round((rendered-start)*1000,2),decode=round((decoded-rendered)*1000,2),geometry=round((end-decoded)*1000,2)))

    def aim(self,camera,index,align_robot=False):
        settings=self.platform
        target=np.array(self.rows[index-1]['qr_front_xyz'])
        if align_robot:
            # Align horizontally; changing height is only allowed on the mast.
            offset=settings.base_rotation()@settings.camera_local_position(camera)
            values={'robot_x_m':float(target[0]-offset[0])}
            if camera=='top':values['pole_length_m']=float(np.clip(target[2]-HEIGHT-.1,.6,1.2))
            settings=settings.updated(values)
        delta=settings.base_rotation().T@(target-settings.camera_position(camera))
        yaw=math.degrees(math.atan2(-delta[0],delta[1]))
        pitch=math.degrees(math.atan2(delta[2],math.hypot(delta[0],delta[1])))
        return settings.updated({camera+'_yaw_deg':yaw,camera+'_pitch_deg':pitch})

    def clear_coverage(self):
        self.cumulative.clear()
        for values in self.camera_cumulative.values():values.clear()

    def interference_report(self):
        # Conservative rotating camera envelope vs full manufacturer's robot AABB.
        results={}
        for name in CAMERAS:
            p=self.platform.camera_local_position(name)
            closest=np.clip(p,[-LENGTH/2,-WIDTH/2,0.],[LENGTH/2,WIDTH/2,HEIGHT])
            clearance=float(np.linalg.norm(p-closest)-CAMERA_SWEEP_RADIUS)
            results[name]=dict(centre_local_m=p.tolist(),swept_radius_m=CAMERA_SWEEP_RADIUS,clearance_to_body_envelope_m=clearance,interference=clearance<0)
        return results

    def add_overview_markers(self,scene):
        first=scene.ngeom
        super().add_overview_markers(scene)
        if scene.ngeom>first:scene.ngeom-=1
        colors={'top':[1.,.68,.04,.8],'left':[.08,.8,.9,.8],'right':[.9,.35,.5,.8]}
        for i in range(first,scene.ngeom):scene.geoms[i].rgba[:]=colors[self.active_camera]
