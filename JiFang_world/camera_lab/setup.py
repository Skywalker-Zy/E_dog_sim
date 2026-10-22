from pathlib import Path
from setuptools import setup
root=Path(__file__).parent
files=[('share/ament_index/resource_index/packages',['resource/jifang_camera_lab']),('share/jifang_camera_lab',['package.xml']),('share/jifang_camera_lab/config',['config/default.json']),('share/jifang_camera_lab/launch',['launch/inspection_camera.launch.py','launch/robot_cameras.launch.py']),('share/jifang_camera_lab/config',['config/robot_platform.json'])]
for directory in sorted((root/'vendor').rglob('*')):
    if directory.is_dir() and directory.name!='__pycache__':
        children=[str(p.relative_to(root)) for p in directory.iterdir() if p.is_file() and p.suffix!='.pyc']
        if children:files.append(('share/jifang_camera_lab/'+str(directory.relative_to(root)),children))
vendor_files=[str(p.relative_to(root)) for p in (root/'vendor').iterdir() if p.is_file()]
files.append(('share/jifang_camera_lab/vendor',vendor_files))
setup(name='jifang_camera_lab',version='0.1.0',packages=['inspection_camera'],data_files=files,
      install_requires=['setuptools'],zip_safe=False,maintainer='JiFang lab',maintainer_email='local@example.com',
      description='MuJoCo single-cabinet inspection camera and ROS 2 interface',license='MIT',
      entry_points={'console_scripts':['camera_lab=inspection_camera.app:main','robot_camera_lab=inspection_camera.robot_app:main']})
