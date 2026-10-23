"""Robot dimensions, camera transforms and self-contained MuJoCo geometry."""
from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from camera_config import DEFAULT_HFOV, camera_projection

CAMERAS = ('top', 'left', 'right')
LENGTH, WIDTH, HEIGHT = .900, .440, .625
SIDE_OFFSET, SIDE_HEIGHT = .210, .700
NATIVE_IN_LINK = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])


def rz(degrees):
    angle = math.radians(degrees)
    return np.array([[math.cos(angle), -math.sin(angle), 0.],
                     [math.sin(angle), math.cos(angle), 0.], [0., 0., 1.]])


def quaternion(matrix):
    result = np.zeros(4)
    mujoco.mju_mat2Quat(result, np.asarray(matrix, dtype=float).reshape(9))
    return result


@dataclass
class RobotSettings:
    robot_x_m: float = .9
    robot_y_m: float = 0.
    robot_yaw_deg: float = 0.
    pole_length_m: float = .6
    top_yaw_deg: float = 0.
    top_pitch_deg: float = -35.
    left_yaw_deg: float = 0.
    left_pitch_deg: float = -40.
    right_yaw_deg: float = 180.
    right_pitch_deg: float = -40.

    def base_rotation(self):
        return rz(self.robot_yaw_deg)

    def camera_local_position(self, camera):
        if camera not in CAMERAS:
            raise ValueError(camera)
        return np.array([0., 0. if camera == 'top' else (SIDE_OFFSET if camera == 'left' else -SIDE_OFFSET),
                         HEIGHT + self.pole_length_m + .1 if camera == 'top' else SIDE_HEIGHT])

    def camera_position(self, camera):
        return np.array([self.robot_x_m, self.robot_y_m, 0.]) + self.base_rotation() @ self.camera_local_position(camera)

    def camera_local_rotation(self, camera):
        yaw, pitch = map(math.radians, [getattr(self, camera + '_yaw_deg'), getattr(self, camera + '_pitch_deg')])
        right = np.array([math.cos(yaw), math.sin(yaw), 0.])
        forward = np.array([-math.sin(yaw) * math.cos(pitch), math.cos(yaw) * math.cos(pitch), math.sin(pitch)])
        return np.column_stack([forward, -right, np.cross(right, forward)])

    def camera_rotation(self, camera):
        return self.base_rotation() @ self.camera_local_rotation(camera)


def element(parent,tag,**values):
    return ET.SubElement(parent,tag,{k:(' '.join(map(str,v)) if isinstance(v,(tuple,list,np.ndarray)) else str(v)) for k,v in values.items()})


def robot_body_xml(settings):
    """Reusable robot subtree, with no cabinet assets or scene defaults."""
    parent=ET.Element('mujocoinclude')
    body=element(parent,'body',name='robot_base',mocap='true',pos=[settings.robot_x_m,settings.robot_y_m,0.],quat=quaternion(settings.base_rotation()))
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
        element(body,'geom',name=side+'_mount_riser',type='capsule',fromto=[0,sign*SIDE_OFFSET,.59,0,sign*SIDE_OFFSET,SIDE_HEIGHT-.025],size='.012',rgba='.7 .73 .76 1',contype=0,conaffinity=0,group=1)
    box('mast_base',[0,0,.64],[.06,.055,.015],'.15 .17 .2 1')
    element(body,'geom',name='telescopic_pole',type='cylinder',pos=[0,0,HEIGHT+settings.pole_length_m/2],size=[.022,settings.pole_length_m/2],rgba='.63 .67 .72 1',contype=0,conaffinity=0,group=1)
    element(body,'geom',name='top_gimbal_riser',type='cylinder',pos=[0,0,HEIGHT+settings.pole_length_m+.025],size=[.025,.025],rgba='.22 .25 .3 1',contype=0,conaffinity=0,group=1)
    for camera,color in [('top','1 .68 .04 1'),('left','.08 .8 .9 1'),('right','.9 .35 .5 1')]:
        mount=element(body,'body',name=camera+'_camera_mount',pos=settings.camera_local_position(camera),quat=quaternion(settings.camera_local_rotation(camera)))
        # The entire rotating package fits inside a 50 mm radius sphere. Hidden
        # from sensors to avoid self-imaging; robot torso/legs/mast remain visible.
        element(mount,'geom',name=camera+'_camera_housing',type='box',pos=[-.019,0,0],size=[.019,.020,.020],rgba=color,contype=0,conaffinity=0,group=2)
        element(mount,'geom',name=camera+'_camera_lens',type='cylinder',pos=[.002,0,0],size=[.012,.004],quat=[.707106781,0,.707106781,0],rgba='.02 .04 .06 1',contype=0,conaffinity=0,group=2)
        sx,sy,_=camera_projection(3840,2160,DEFAULT_HFOV[camera])
        element(mount,'camera',name=camera+'_inspection_camera',quat=quaternion(NATIVE_IN_LINK),sensorsize=[sx,sy],resolution=[3840,2160],focal=[1.,1.])
    return body


