from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    args=['--scene-dir',LaunchConfiguration('scene_dir').perform(context)]
    if LaunchConfiguration('headless').perform(context).lower() in ['true','1','yes']:
        args.append('--headless')
    return [Node(package='jifang_camera_lab',executable='camera_lab',output='screen',arguments=args)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scene_dir',default_value=str(Path.home()/'E_dog/JiFang_world')),
        DeclareLaunchArgument('headless',default_value='false'),
        OpaqueFunction(function=start)])
