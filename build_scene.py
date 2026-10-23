"""Rebuild the combined entry and robot model from local sources only."""
from pathlib import Path
import xml.etree.ElementTree as ET
from robot_model import RobotSettings, robot_body_xml

ROOT = Path(__file__).resolve().parent


def build():
    robot = ET.Element('mujocoinclude')
    robot.append(robot_body_xml(RobotSettings()))
    root = ET.Element('mujoco', model='A2W_two_rows_20_racks')
    ET.SubElement(root, 'include', file='room.xml')
    ET.SubElement(root, 'compiler', texturedir='assets')
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', offwidth='3840', offheight='2160')
    world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'include', file='robot.xml')
    ET.SubElement(world, 'camera', name='robot_overview', pos='-0.65 -0.42 1.25',
                  xyaxes='.261 -.965 0 .227 .061 .972', fovy='62')
    for filename, tree in [('robot.xml', robot), ('scene.xml', root)]:
        ET.indent(tree, space='  ')
        ET.ElementTree(tree).write(ROOT / filename, encoding='utf-8', xml_declaration=True)
    print('已生成：', ROOT / 'scene.xml')


if __name__ == '__main__':
    build()
