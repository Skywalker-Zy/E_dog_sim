"""Combine the existing two-row room and the exported three-camera robot."""
from pathlib import Path
import json
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent
ROOM = ROOT.parent / 'two_rows_20_racks'
ROBOT = ROOT.parent / '03_robot_cameras' / 'standalone_robot'


def build():
    robot = ET.parse(ROBOT / 'robot.xml').getroot()
    robot.find('body').set('pos', '0.9 0 0')
    ET.indent(robot, space='  ')
    ET.ElementTree(robot).write(ROOT / 'robot.xml', encoding='utf-8', xml_declaration=True)
    # MuJoCo merges the included room first, then applies the local compiler
    # paths and visual settings relative to this scene's directory.
    root = ET.Element('mujoco', model='A2W_two_rows_20_racks')
    ET.SubElement(root, 'include', file='../two_rows_20_racks/scene.xml')
    ET.SubElement(root, 'compiler', texturedir='../two_rows_20_racks/assets')
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', offwidth='1920', offheight='1080')
    world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'include', file='robot.xml')
    ET.SubElement(world, 'camera', name='robot_overview', pos='-0.65 -0.42 1.25',
                  xyaxes='.261 -.965 0 .227 .061 .972', fovy='62')
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(ROOT / 'scene.xml', encoding='utf-8', xml_declaration=True)
    source = json.loads((ROOM / 'layout.json').read_text())
    keys = ['seed', 'installed_equipment_count', 'qr_count', 'missing_qr_on_installed_equipment', 'dimensions', 'rack_count']
    layout = {key: source[key] for key in keys}
    layout['robot'] = {'base_position_m': [.9, 0, 0], 'yaw_deg': 0,
                       'side_camera_centre_to_torso_side_m': .05,
                       'source': '../03_robot_cameras/standalone_robot/robot.xml'}
    (ROOT / 'layout.json').write_text(json.dumps(layout, ensure_ascii=False, indent=2) + '\n')
    print('已生成：', ROOT / 'scene.xml')


if __name__ == '__main__':
    build()
