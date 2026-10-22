"""Prepare bundled resources without mixing other venvs into an active venv."""
from pathlib import Path
import os
import sys


def prepare():
    package_root = Path(__file__).resolve().parent.parent
    vendor = package_root / 'vendor'
    if vendor.is_dir():
        sys.path.append(str(vendor))
    else:
        try:
            from ament_index_python.packages import get_package_share_directory
            installed_vendor = Path(get_package_share_directory('jifang_camera_lab')) / 'vendor'
            if installed_vendor.is_dir():sys.path.append(str(installed_vendor))
        except (ImportError, LookupError):
            pass
    runtime = os.environ.get('JIFANG_MUJOCO_SITE', '')
    candidates = []
    if sys.prefix == sys.base_prefix:
        python_directory = f'python{sys.version_info.major}.{sys.version_info.minor}'
        candidates = [Path(runtime)] if runtime else [Path.home() / 'mujoco_work/.venv/lib' / python_directory / 'site-packages']
    for candidate in candidates:
        if candidate.is_dir():
            sys.path.append(str(candidate))
    os.environ.setdefault('MUJOCO_GL', 'egl')
    return package_root


def default_scene_dir():
    """Resolve the source scene or the standard checkout after ROS installation."""
    source_world = Path(__file__).resolve().parents[3]
    source_scene = source_world / '01_single_cabinet'
    if (source_scene / 'scene.xml').exists():
        return source_scene
    checkout = Path(os.environ.get('EDOG_PROJECT_ROOT', Path.home() / 'E_dog_sim'))
    return checkout / 'JiFang_world/01_single_cabinet'
