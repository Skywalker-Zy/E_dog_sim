"""Check that copied sources/assets rebuild and run without the original tree."""
from pathlib import Path
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
from PIL import Image

ROOT = Path(__file__).resolve().parent


def main():
    environment = dict(os.environ, PYTHONPATH='', MUJOCO_GL='egl', EDOG_VENV=sys.prefix)
    with tempfile.TemporaryDirectory(prefix='edog_portable_') as temporary:
        destination = Path(temporary)
        for pattern in ('*.py', '*.xml', '*.json', '*.sh', 'requirements.txt'):
            for path in ROOT.glob(pattern):
                shutil.copy2(path, destination / path.name)
        for name in ('assets', 'scripts'):
            shutil.copytree(ROOT / name, destination / name)
        subprocess.run([sys.executable, 'build_scene.py'], cwd=destination, env=environment, check=True, capture_output=True)
        for name in ('scene.xml', 'robot.xml'):
            assert (destination / name).read_bytes() == (ROOT / name).read_bytes(), name
        script = '''
import hashlib,json
from simulation import RobotRoomSimulation
sim=RobotRoomSimulation()
try:
    image,result=sim.capture('left',True)
    assert image.shape==(2160,3840,3)
    assert len(result)==10
    print(json.dumps({'geometry_count':sim.model.ngeom,'image_sha256':hashlib.sha256(image.tobytes()).hexdigest(),'qr_count':len(result)}))
finally:sim.close()
'''
        result = subprocess.run([sys.executable, '-c', script], cwd=destination, env=environment,
                                check=True, capture_output=True, text=True)
        report = json.loads(result.stdout)
        assert report['geometry_count'] == 5712
        with Image.open(ROOT / 'previews/left_inspection_camera.png') as preview:
            assert report['image_sha256'] == hashlib.sha256(preview.convert('RGB').tobytes()).hexdigest()
        subprocess.run(['bash', 'run.sh', '--headless', '--no-ros', '--frames', '3'],
                       cwd=destination, env=environment, check=True, capture_output=True)
    report['checks'] = ['copy uses only local source files and assets', 'empty PYTHONPATH',
                        'rebuild matches delivered XML bytes', 'independent 4K capture and actual QR decoding',
                        'headless CLI starts without ROS or old directories']
    (ROOT / 'verification').mkdir(exist_ok=True)
    (ROOT / 'verification/portability_validation.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
