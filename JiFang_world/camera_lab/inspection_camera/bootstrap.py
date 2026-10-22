"""Use system Python/ROS NumPy; append the existing MuJoCo environment last."""
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
    candidates = [Path(runtime)] if runtime else sorted((Path.home() / 'mujoco_work/.venv/lib').glob('python*/site-packages'))
    for candidate in candidates:
        if candidate.is_dir():
            sys.path.append(str(candidate))
    os.environ.setdefault('MUJOCO_GL', 'egl')
    return package_root
