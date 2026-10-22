from dataclasses import asdict
import math,json
import numpy as np
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image,CameraInfo
from geometry_msgs.msg import PoseStamped,TransformStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from .robot_platform import CAMERAS,RobotSettings,OPTICAL_IN_LINK,quaternion,rz,mujoco

PREFIX='/inspection_platform'

def putq(message,q):message.w,message.x,message.y,message.z=map(float,q)

class PlatformNode(Node):
    def __init__(self,engine):
        super().__init__('jifang_robot_cameras');self.engine=engine;self.changed=lambda:None;self.reset=lambda:None
        for key,value in asdict(engine.platform).items():self.declare_parameter(key,value)
        engine.apply_platform(RobotSettings().updated({key:self.get_parameter(key).value for key in asdict(engine.platform)}))
        self.add_on_set_parameters_callback(self.accept_parameters)
        self.images={name:self.create_publisher(Image,f'{PREFIX}/{name}/image_raw',qos_profile_sensor_data) for name in CAMERAS}
        self.infos={name:self.create_publisher(CameraInfo,f'{PREFIX}/{name}/camera_info',qos_profile_sensor_data) for name in CAMERAS}
        self.poses={name:self.create_publisher(PoseStamped,f'{PREFIX}/{name}/pose',10) for name in CAMERAS}
        self.detections={name:self.create_publisher(String,f'{PREFIX}/{name}/qr_detections',10) for name in CAMERAS}
        self.base_pub=self.create_publisher(PoseStamped,f'{PREFIX}/robot_pose',10)
        self.tf=TransformBroadcaster(self);self.static_tf=StaticTransformBroadcaster(self)
        self.create_subscription(PoseStamped,f'{PREFIX}/robot_target_pose',self.target_pose,10)
        self.create_service(Trigger,f'{PREFIX}/reset_coverage',self.reset_coverage)
        transforms=[]
        for name in CAMERAS:
            t=TransformStamped();t.header.stamp=self.get_clock().now().to_msg();t.header.frame_id=f'{name}_camera_link';t.child_frame_id=f'{name}_camera_optical_frame'
            putq(t.transform.rotation,quaternion(OPTICAL_IN_LINK));transforms.append(t)
        self.static_tf.sendTransform(transforms)
        self.get_logger().info('A2-W 三相机平台已启动：固定站姿、整体平移、三路图像/内参/TF。')
    def accept_parameters(self,parameters):
        try:
            values={p.name:p.value for p in parameters if p.name in asdict(self.engine.platform)}
            self.engine.apply_platform(self.engine.platform.updated(values));self.changed()
            return SetParametersResult(successful=True)
        except (ValueError,TypeError) as e:return SetParametersResult(successful=False,reason=str(e))
    def set_settings(self,values):
        response=self.set_parameters_atomically([Parameter(key,value=value) for key,value in values.items()])
        if not response.successful:raise ValueError(response.reason)
    def reset_coverage(self,request,response):
        self.reset();response.success=True;response.message='三路累计统计已清空';return response
    def target_pose(self,message):
        try:
            if message.header.frame_id!='world':raise ValueError('frame_id 必须为 world')
            p,q=message.pose.position,message.pose.orientation
            if not all(math.isfinite(v) for v in [p.x,p.y,p.z,q.w,q.x,q.y,q.z]):raise ValueError('位姿必须为有限数值')
            if abs(p.z)>.001:raise ValueError('robot_base 为地面投影坐标，z 必须为 0')
            quat=np.array([q.w,q.x,q.y,q.z]);r=np.zeros(9)
            if abs(np.linalg.norm(quat)-1)>.001:raise ValueError('四元数必须归一化')
            mujoco.mju_quat2Mat(r,quat);r=r.reshape(3,3)
            yaw=math.degrees(math.atan2(r[1,0],r[0,0]))
            if not np.allclose(r,rz(yaw),atol=.001):raise ValueError('仅允许平移和 yaw，保持站立')
            self.set_settings(dict(robot_x_m=float(p.x),robot_y_m=float(p.y),robot_yaw_deg=yaw))
        except ValueError as error:self.get_logger().warning('拒绝目标位姿：'+str(error))
    def publish(self,images,results):
        stamp=self.get_clock().now().to_msg();settings=RobotSettings().updated(results['top']['platform_settings'])
        base=PoseStamped();base.header.stamp=stamp;base.header.frame_id='world'
        base.pose.position.x=settings.robot_x_m;base.pose.position.y=settings.robot_y_m;putq(base.pose.orientation,quaternion(settings.base_rotation()));self.base_pub.publish(base)
        transform=TransformStamped();transform.header=base.header;transform.child_frame_id='robot_base'
        transform.transform.translation.x=settings.robot_x_m;transform.transform.translation.y=settings.robot_y_m;transform.transform.rotation=base.pose.orientation
        transforms=[transform]
        for name in CAMERAS:
            image=images[name];result=results[name];height,width=image.shape[:2]
            raw=Image();raw.header.stamp=stamp;raw.header.frame_id=f'{name}_camera_optical_frame';raw.width=width;raw.height=height;raw.encoding='rgb8';raw.step=width*3
            if self.images[name].get_subscription_count():raw.data=image.tobytes();self.images[name].publish(raw)
            k=np.array(result['intrinsics']);info=CameraInfo();info.header=raw.header;info.width=width;info.height=height;info.distortion_model='plumb_bob';info.d=[0.]*5;info.k=k.reshape(9).tolist();info.r=np.eye(3).reshape(9).tolist();info.p=np.column_stack([k,np.zeros(3)]).reshape(12).tolist()
            if self.infos[name].get_subscription_count():self.infos[name].publish(info)
            pose=PoseStamped();pose.header=base.header;pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,result['camera_position_m']);putq(pose.pose.orientation,result['camera_quaternion_wxyz']);self.poses[name].publish(pose)
            t=TransformStamped();t.header.stamp=stamp;t.header.frame_id='robot_base';t.child_frame_id=f'{name}_camera_link';t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=map(float,result['local_position_m']);putq(t.transform.rotation,quaternion(np.array(result['local_rotation'])));transforms.append(t)
            self.detections[name].publish(String(data=json.dumps(dict(stamp=dict(sec=stamp.sec,nanosec=stamp.nanosec),**result),ensure_ascii=False)))
        self.tf.sendTransform(transforms)
