from pathlib import Path
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    args=['--scene-dir',LaunchConfiguration('scene_dir').perform(context)]
    if LaunchConfiguration('headless').perform(context).lower() in ['true','yes','1']:args.append('--headless')
    return [Node(package='jifang_camera_lab',executable='robot_camera_lab',output='screen',arguments=args)]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('scene_dir',default_value=str(Path(os.environ.get('EDOG_PROJECT_ROOT',Path.home()/'E_dog_sim'))/'JiFang_world/01_single_cabinet')),
                              DeclareLaunchArgument('headless',default_value='false'),OpaqueFunction(function=start)])
