#!/usr/bin/env python3
"""Build a portable two-row server room; all assets are generated locally."""
from pathlib import Path
import argparse
import csv
import json
import math
import random
import sys
import xml.etree.ElementTree as ET
from collections import Counter
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'vendor'))
import qrcode

U = 0.04445
RACK_WIDTH, RACK_DEPTH, RACK_HEIGHT = 0.600, 1.000, 2.177
AISLE_WIDTH = 1.200
N_PER_ROW = 10
DOOR_X = 6.650
MAX_DOOR_DEG = 100.0


def fmt(v):
    if isinstance(v, (list, tuple)):
        return ' '.join(fmt(x) for x in v)
    if isinstance(v, float):
        return f'{v:.9f}'.rstrip('0').rstrip('.')
    return str(v)


def add(parent, tag, **values):
    return ET.SubElement(parent, tag, {k: fmt(v) for k, v in values.items()})


def box(parent, name, pos, size, material, **values):
    return add(parent, 'geom', name=name, type='box', pos=pos, size=size, material=material, **values)


def capsule(parent, name, a, b, radius, material, **values):
    return add(parent, 'geom', name=name, type='capsule', fromto=[*a, *b], size=radius, material=material, **values)


def write_xml(path, root):
    ET.indent(root, space='  ')
    path.write_text('<?xml version="1.0" encoding="utf-8"?>\n' + ET.tostring(root, encoding='unicode') + '\n', encoding='utf-8')


def text_image(path, text, size=(384, 80), foreground='#192b3b', background='#edf1ee'):
    image = Image.new('RGB', size, background)
    draw = ImageDraw.Draw(image)
    try:
        font = ImageFont.truetype('DejaVuSans.ttf', round(size[1] * .63))
    except OSError:
        font = ImageFont.load_default()
    bounds = draw.textbbox((0, 0), text, font=font)
    draw.text(((size[0] - (bounds[2] - bounds[0])) / 2 - bounds[0], (size[1] - (bounds[3] - bounds[1])) / 2 - bounds[1]), text, font=font, fill=foreground)
    image.save(path)


def shared_textures(directory):
    image = Image.new('RGB', (512, 512), '#232c32')
    draw = ImageDraw.Draw(image)
    for x in range(12, 500, 16):
        draw.line((x, 12, x, 500), fill='#9aa6ac', width=4)
    for y in range(12, 500, 72):
        draw.line((12, y, 500, y), fill='#87949c', width=5)
    draw.rectangle((0, 0, 511, 511), outline='#b5bec2', width=7)
    draw.rectangle((7, 7, 504, 504), outline='#4b565f', width=5)
    image.save(directory / 'floor_grate.png')
    image = Image.new('RGB', (512, 96), '#303a43')
    draw = ImageDraw.Draw(image)
    for x in range(130, 480, 12):
        for y in range(14, 85, 12):
            draw.ellipse((x, y, x + 5, y + 4), fill='#080e15')
    draw.rectangle((1, 1, 510, 94), outline='#141e28', width=4)
    image.save(directory / 'server_face.png')
    text_image(directory / 'exit_sign.png', 'EXIT', (256, 96), '#f0fff3', '#188447')
    text_image(directory / 'door_sign.png', 'MANUAL ACCESS', (512, 80))


class Builder:
    def __init__(self, output, seed):
        self.output, self.seed = output, seed
        self.rng = random.Random(seed)
        self.asset_root = ET.Element('mujocoinclude')
        self.asset = add(self.asset_root, 'asset')
        self.qr_values = {}
        self.racks = []
        self.equipment = []
        self.make_assets()

    def texture_material(self, name, file, flip=False, **extra):
        add(self.asset, 'texture', name=name + '_tex', type='2d', file=file, hflip='true' if flip else 'false')
        add(self.asset, 'material', name=name, texture=name + '_tex', texrepeat=[1, 1], texuniform='false', rgba=[1, 1, 1, 1], specular=0, **extra)

    def make_assets(self):
        add(self.asset, 'texture', name='sky', type='skybox', builtin='gradient', rgb1=[.16, .22, .29], rgb2=[.60, .67, .72], width=256, height=1536)
        add(self.asset, 'texture', name='tiles_tex', type='2d', builtin='checker', rgb1=[.55, .59, .61], rgb2=[.60, .63, .65], width=256, height=256)
        add(self.asset, 'texture', name='grate_tex', type='2d', file='floor_grate.png')
        add(self.asset, 'material', name='floor', texture='tiles_tex', texrepeat=[15, 6], texuniform='false', specular=.1)
        add(self.asset, 'material', name='grate', texture='grate_tex', texrepeat=[10, 2], texuniform='false', specular=.15)
        self.texture_material('server_front', 'server_face.png')
        self.texture_material('exit_sign', 'exit_sign.png', flip=True, emission=.3)
        self.texture_material('door_sign', 'door_sign.png', flip=True)
        materials = {
            'silver': [.62, .68, .72, 1], 'silver_bright': [.77, .81, .83, 1],
            'dark': [.045, .06, .08, 1], 'panel': [.10, .13, .16, 1],
            'side_panel': [.22, .26, .30, 1], 'black': [.01, .018, .025, 1],
            'wall': [.69, .73, .76, 1], 'trim': [.36, .41, .46, 1],
            'blue': [.015, .29, .64, 1], 'yellow': [.9, .64, .06, 1],
            'label': [.82, .86, .84, 1], 'glass': [.08, .16, .20, .62],
            'door_panel': [.15, .20, .23, 1], 'red': [.7, .04, .03, 1]
        }
        for name, rgba in materials.items():
            add(self.asset, 'material', name=name, rgba=rgba, specular=.25 if name != 'glass' else .5, shininess=.3)
        for name, rgba in [('led_green', [.08, .95, .25, 1]), ('led_blue', [.04, .5, 1, 1]), ('lamp', [.96, .98, 1, 1])]:
            add(self.asset, 'material', name=name, rgba=rgba, emission=.6)

    def qr_material(self, value):
        if value in self.qr_values:
            return self.qr_values[value]
        name = f'qr_{len(self.qr_values) + 1:03d}'
        file = name + '.png'
        qr = qrcode.QRCode(version=1, error_correction=qrcode.constants.ERROR_CORRECT_M, box_size=8, border=4)
        qr.add_data(value)
        qr.make(fit=True)
        matrix = qr.get_matrix()
        n = len(matrix)
        image = Image.new('RGB', (n, n), 'white')
        image.putdata([(0, 0, 0) if bit else (255, 255, 255) for row in matrix for bit in row])
        image.resize((n * 8, n * 8), Image.NEAREST).save(self.output / 'assets' / file)
        self.texture_material(name, file)
        self.qr_values[value] = name
        return name

    def rack(self, world, side, index):
        rng = self.rng
        rack_id = f'{side}{index + 1:02d}'
        x = .3 + index * .6
        y = 1.1 if side == 'L' else -1.1
        sign = 1 if side == 'L' else -1
        body = add(world, 'body', name='rack_' + rack_id, pos=[x, y, 0], quat=[1, 0, 0, 0] if side == 'L' else [0, 0, 0, 1])
        add(body, 'geom', name=rack_id + '_collision', type='box', pos=[0, 0, RACK_HEIGHT / 2], size=[.3, .5, RACK_HEIGHT / 2], group=3, rgba=[0, 0, 0, 0], contype=1, conaffinity=1, mass=0)
        visual = add(body, 'body', name=rack_id + '_visual', childclass='decoration')
        box(visual, rack_id + '_base', [0, 0, .05], [.3, .5, .05], 'silver')
        box(visual, rack_id + '_base_inset', [0, -.499, .05], [.245, .0007, .023], 'dark')
        box(visual, rack_id + '_roof', [0, 0, 2.162], [.3, .5, .015], 'silver')
        for dx in [-.285, .285]:
            for dy in [-.485, .485]:
                box(visual, f'{rack_id}_post_{dx}_{dy}', [dx, dy, RACK_HEIGHT / 2], [.015, .015, RACK_HEIGHT / 2], 'silver')
        for dx in [-.294, .294]:
            box(visual, f'{rack_id}_side_{dx}', [dx, .01, 1.13], [.005, .458, 1.0], 'side_panel')
        box(visual, rack_id + '_rear', [0, .492, 1.13], [.27, .005, 1.0], 'dark')
        # An internal dark backing makes genuinely empty slots obvious.
        box(visual, rack_id + '_empty_background', [0, -.36, 1.035], [.246, .012, .935], 'black')
        for dx in [-.258, .258]:
            box(visual, f'{rack_id}_rail_{dx}', [dx, -.477, 1.035], [.012, .016, .935], 'silver')
        text_image(self.output / 'assets' / (rack_id + '_id.png'), rack_id, (192, 64))
        self.texture_material(rack_id + '_id_material', rack_id + '_id.png')
        box(visual, rack_id + '_id_plaque', [0, -.482, 2.135], [.055, .001, .018], rack_id + '_id_material')
        # Sparse cabinets keep all 20 shells but contain more vacant mounting space.
        sparse = (side, index) in [('L', 3), ('R', 7)]
        vacancy_probability = .40 if sparse else .17
        slot = 0
        segments = []
        qr_specs = []
        equipment_count = qr_count = empty_u = 0
        while slot < 42:
            remaining = 42 - slot
            units = min(rng.choices([1, 2, 3, 4], weights=[.22, .31, .27, .20])[0], remaining)
            vacant = rng.random() < vacancy_probability
            if index == 0 and slot == 0:
                vacant = False
                units = 3 if side == 'L' else 2
            bottom = .1 + slot * U
            top = bottom + units * U
            centre = (bottom + top) / 2
            segment = dict(start_u=slot + 1, units=units, bottom_m=round(bottom, 9), top_m=round(top, 9), vacant=vacant)
            if vacant:
                empty_u += units
                # A few silver support ledges, with no server and no QR.
                if rng.random() < .45:
                    box(visual, f'{rack_id}_empty_shelf_{slot}', [0, -.03, bottom + .003], [.245, .44, .003], 'silver')
            else:
                equipment_count += 1
                eq_id = f'{rack_id}_S{equipment_count:02d}'
                face_depth = -.477 + rng.choice([-.003, 0, .002])
                box(visual, eq_id + '_chassis', [0, -.015, centre], [.2413, .43, (top - bottom) / 2 - .0015], 'dark')
                box(visual, eq_id + '_face', [0, face_depth, centre], [.240, .006, (top - bottom) / 2 - .0015], 'server_front')
                for u in range(units):
                    z = bottom + (u + .5) * U
                    # Two long ventilation slits per U; inexpensive geometry.
                    for dz in [-.006, .006]:
                        box(visual, f'{eq_id}_slit_{u}_{dz}', [.054, face_depth - .0065, z + dz], [.115, .0005, .0013], 'black')
                for dx in [-.216, .214]:
                    box(visual, f'{eq_id}_handle_{dx}', [dx, face_depth - .009, centre], [.004, .003, min(.024, units * U / 2 - .006)], 'silver_bright')
                for led in range(2):
                    add(visual, 'geom', name=f'{eq_id}_led_{led}', type='sphere', pos=[.181 + .012 * led, face_depth - .007, centre + .006], size=.0021, material='led_green' if led == 0 else 'led_blue')
                if units >= 3 or rng.random() < .35:
                    box(visual, eq_id + '_divider', [0, -.08, bottom + .001], [.263, .40, .002], 'silver')
                # Every occupied U gets one QR. Empty slots have no QR.
                front_y = face_depth - .007
                qr_specs.append(dict(equipment_id=eq_id, rack_id=rack_id, side=side, sign=sign,
                                     start_u=slot + 1, units=units, front_y=front_y,
                                     unit_z=[round(bottom + (u + .5) * U, 9) for u in range(units)]))
                box(visual, eq_id + '_serial_label', [-.10, front_y, centre + .004], [.032, .00035, .004], 'label')
                self.equipment.append(dict(equipment_id=eq_id, rack_id=rack_id, side=side, rack_index=index + 1,
                             start_u=slot + 1, units=units, bottom_m=round(bottom, 9), top_m=round(top, 9),
                             centre_height_m=round(centre, 9), qr_present=True, qr_payloads=[], qr_items=[],
                             qr_size_m=[.04, .04], front_normal_world=[0, -sign, 0]))
                segment.update(equipment_id=eq_id, qr_present=True, qr_count=units)
            segments.append(segment)
            slot += units
        # Assign QR sequence bottom-to-top within this cabinet, then add each 1U label.
        qr_sequence = []
        for spec in sorted(qr_specs, key=lambda item: item['unit_z'][0]):
            for unit_offset, z in enumerate(spec['unit_z']):
                qr_sequence.append((spec, unit_offset, z))
        rack_qr_total = len(qr_sequence)
        for qr_index, (spec, unit_offset, z) in enumerate(qr_sequence, start=1):
            payload = f"{rack_id}-{qr_index}/{rack_qr_total}-{z:.2f}"
            material = self.qr_material(payload)
            geom_name = f"{spec['equipment_id']}_qr_{unit_offset + 1:02d}"
            box(visual, geom_name, [-.169, spec['front_y'], z], [.020, .00035, .020], material)
            world_pos = [round(x + spec['sign'] * (-.169), 9), round(y + spec['sign'] * spec['front_y'], 9), round(z, 9)]
            item = dict(sequence=qr_index, total=rack_qr_total, payload=payload, height_m=round(z, 2),
                        equipment_id=spec['equipment_id'], unit_in_equipment=unit_offset + 1,
                        position_world_m=world_pos, qr_texture=material + '.png', qr_geom=geom_name,
                        size_m=[.04, .04])
            entry = next(e for e in self.equipment if e['equipment_id'] == spec['equipment_id'])
            entry['qr_payloads'].append(payload)
            entry['qr_items'].append(item)
            qr_count += 1
        for entry in self.equipment:
            if entry['rack_id'] == rack_id:
                entry['qr_payload'] = entry['qr_payloads'][0] if entry['qr_payloads'] else None
                entry['qr_count'] = len(entry['qr_items'])
        # Top patch area and independent, slightly varied cable management.
        box(visual, rack_id + '_top_recess', [0, -.414, 2.042], [.245, .030, .062], 'black')
        for port in range(rng.randint(7, 11)):
            dx = -.211 + port * .041
            box(visual, f'{rack_id}_top_port_{port}', [dx, -.451, 2.016], [.011, .006, .018], 'yellow')
            capsule(visual, f'{rack_id}_top_cable_{port}_a', [dx, -.465, 2.029], [dx, -.476, 2.071], .0025, 'blue')
            capsule(visual, f'{rack_id}_top_cable_{port}_b', [dx, -.476, 2.071], [.226, -.478, 2.078 + port * .0015], .0025, 'blue')
        if rng.random() < .72:
            z0 = rng.uniform(.44, .90)
            for cable in range(rng.randint(3, 5)):
                points = []
                for k in range(9):
                    t = k / 8
                    points.append([-.247 + .026 * math.sin(math.pi * t), -.488, z0 - .23 * t + cable * .007])
                for k in range(8):
                    capsule(visual, f'{rack_id}_blue_loop_{cable}_{k}', points[k], points[k + 1], .0022, 'blue')
        rack = dict(rack_id=rack_id, side=side, index=index + 1, origin_world_m=[round(x, 9), y, 0], yaw_deg=0 if side == 'L' else 180,
                    dimensions_m=[RACK_WIDTH, RACK_DEPTH, RACK_HEIGHT], slots_u=42, sparse=sparse,
                    equipment_count=equipment_count, qr_count=rack_qr_total, vacant_u=empty_u, segments=segments)
        self.racks.append(rack)

    def architecture(self, world):
        add(world, 'geom', name='floor_collision', type='plane', size=[12, 5, .1], material='floor', friction=[.9, .01, .001])
        deco = add(world, 'body', name='room_decoration', childclass='decoration')
        box(deco, 'aisle_grate_surface', [3.0, 0, .0003], [3.0, .6, .0002], 'grate')
        # Ground remains flat; this is a visual grille, not a mesh of foot traps.
        for y in [-1.77, 1.77]:
            box(world, f'side_wall_{y}', [3.4, y, 1.4], [4.5, .04, 1.4], 'wall')
            box(deco, f'skirting_{y}', [3.4, y - math.copysign(.047, y), .05], [4.5, .008, .05], 'trim')
        for x in [-1.1, 7.9]:
            for y in [-1.185, 1.185]:
                box(world, f'end_wall_{x}_{y}', [x, y, 1.4], [.045, .545, 1.4], 'wall')
            box(world, f'end_lintel_{x}', [x, 0, 2.53], [.045, .64, .27], 'wall')
        # High side soffits and ceiling lights suggest a room but leave overview open.
        for y in [-1.06, 1.06]:
            box(deco, f'ceiling_soffit_{y}', [3.0, y, 2.46], [3.05, .55, .045], 'wall')
        for i in range(6):
            x = .15 + i * 1.15
            box(deco, f'ceiling_light_frame_{i}', [x, 0, 2.67], [.42, .085, .020], 'silver')
            box(deco, f'ceiling_light_{i}', [x, 0, 2.644], [.39, .062, .004], 'lamp')
            if i < 5:  # Seven world lights plus the headlight fit OpenGL's eight fixed lights.
                add(world, 'light', name=f'aisle_light_{i}', pos=[x, 0, 2.60], dir=[0, 0, -1], diffuse=[.18, .19, .20], specular=[.12, .12, .12], castshadow='false')
        self.door(world, deco)

    def door(self, world, decoration):
        # Two real hinge bodies, swinging into the rear vestibule (+X).
        # Jamb opening = 1.300 m, wider than the 1.200 m aisle to clear leaf rotation.
        for y in [-.705, .705]:
            box(world, f'door_jamb_{y}', [DOOR_X, y, 1.10], [.065, .055, 1.10], 'silver_bright')
        box(world, 'door_header', [DOOR_X, 0, 2.18], [.065, .76, .04], 'silver_bright')
        for y in [-1.26, 1.26]:
            box(world, f'door_side_wall_{y}', [DOOR_X, y, 1.11], [.045, .50, 1.11], 'wall')
            box(decoration, f'door_side_glass_{y}', [DOOR_X - .049, y, 1.24], [.003, .29, .74], 'door_panel')
        for name, hinge_y, direction, axis in [('left', .6, -1, [0, 0, 1]), ('right', -.6, 1, [0, 0, -1])]:
            body = add(world, 'body', name='door_' + name, pos=[DOOR_X, hinge_y, 0])
            add(body, 'joint', name='door_' + name + '_hinge', type='hinge', axis=axis, range=[0, MAX_DOOR_DEG], damping=8, armature=.02)
            centre_y = direction * .295
            # A single solid collision proxy, including the glazed opening.
            add(body, 'geom', name='door_' + name + '_collision', type='box', pos=[0, centre_y, 1.07], size=[.020, .295, 1.05], rgba=[0, 0, 0, 0], group=3, mass=15, contype=1, conaffinity=1)
            detail = add(body, 'body', name='door_' + name + '_visual', childclass='decoration')
            for y in [direction * .025, direction * .565]:
                box(detail, f'{name}_door_upright_{y}', [0, y, 1.07], [.0205, .025, 1.05], 'silver')
            for z in [.05, 2.09, .52]:
                box(detail, f'{name}_door_crossbar_{z}', [0, centre_y, z], [.0205, .295, .025], 'silver')
            box(detail, name + '_door_lower', [0, centre_y, .27], [.018, .244, .19], 'door_panel')
            box(detail, name + '_door_glass', [0, centre_y, 1.31], [.010, .243, .748], 'glass')
            handle_y = direction * .525
            for x in [-.048, .048]:
                capsule(detail, f'{name}_door_handle_{x}', [x, handle_y, .86], [x, handle_y, 1.12], .010, 'silver_bright')
            add(detail, 'site', name=name + '_door_free_edge', pos=[0, direction * .59, 1.07], size=.006, rgba=[0, 0, 0, 0], group=4)
        box(decoration, 'exit_sign_frame', [DOOR_X - .07, 0, 2.36], [.015, .16, .07], 'dark')
        box(decoration, 'exit_sign_face', [DOOR_X - .086, 0, 2.36], [.001, .145, .055], 'exit_sign')
        # The physical wall switch is visual; actual commands are explicit UI buttons.
        box(decoration, 'wall_door_button_frame', [DOOR_X - .10, -.80, 1.10], [.02, .035, .055], 'silver')
        box(decoration, 'wall_door_button', [DOOR_X - .122, -.80, 1.10], [.003, .022, .030], 'led_green')

    def build(self):
        root = ET.Element('mujoco', model='two_rows_20_racks_manual_door')
        add(root, 'compiler', angle='degree', texturedir='assets', autolimits='true')
        add(root, 'option', timestep=.002, gravity=[0, 0, -9.81], integrator='implicitfast')
        visual = add(root, 'visual')
        add(visual, 'global', offwidth=1600, offheight=1000, azimuth=180, elevation=-12)
        add(visual, 'headlight', ambient=[.35, .35, .35], diffuse=[.6, .6, .6], specular=[.15, .15, .15])
        add(visual, 'quality', shadowsize=2048)
        add(visual, 'map', znear=.01, zfar=50)
        add(root, 'statistic', center=[3.0, 0, 1.10], extent=4.3)
        default = add(root, 'default')
        add(default, 'geom', friction=[.8, .01, .001], solref=[.02, 1])
        decoration = add(default, 'default', **{'class': 'decoration'})
        add(decoration, 'geom', contype=0, conaffinity=0, group=1, mass=0)
        add(root, 'include', file='assets.xml')
        world = add(root, 'worldbody')
        add(world, 'light', name='entrance_key', pos=[-1.5, -.2, 3.6], dir=[1, 0, -1], diffuse=[.55, .55, .55], castshadow='true')
        add(world, 'light', name='rear_fill', pos=[7.5, 0, 3.2], dir=[-1, 0, -1], diffuse=[.45, .45, .45], castshadow='false')
        # Looking along +X: screen-right is -Y, camera up is +Z.
        add(world, 'camera', name='aisle', pos=[-1.02, 0, 1.38], xyaxes=[0, -1, 0, .055, 0, .998486], fovy=66)
        add(world, 'camera', name='door_detail', pos=[5.23, 0, 1.20], xyaxes=[0, -1, 0, 0, 0, 1], fovy=68)
        add(world, 'camera', name='top', pos=[3.1, 0, 10.5], xyaxes=[1, 0, 0, 0, 1, 0], fovy=48)
        add(world, 'camera', name='overview', pos=[-1.9, -5.5, 5.0], xyaxes=[.746, -.666, 0, .315, .353, .881], fovy=55)
        self.architecture(world)
        for side in ['L', 'R']:
            for index in range(N_PER_ROW):
                self.rack(world, side, index)
        actuator = add(root, 'actuator')
        for side in ['left', 'right']:
            add(actuator, 'position', name='door_' + side + '_motor', joint='door_' + side + '_hinge', kp=600, kv=80,
                ctrllimited='true', ctrlrange=[0, math.radians(MAX_DOOR_DEG)], forcelimited='true', forcerange=[-120, 120])
        write_xml(self.output / 'scene.xml', root)
        write_xml(self.output / 'assets.xml', self.asset_root)
        self.metadata()
        self.floor_plan()

    def floor_plan(self):
        parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="1100" height="620" viewBox="0 0 1100 620">',
                 '<rect width="1100" height="620" fill="#f3f6fa"/>',
                 '<g font-family="sans-serif" fill="#183247">',
                 '<text x="42" y="45" font-size="24">双列 20 机柜 / Two rows · 20 racks</text>',
                 '<rect x="50" y="123" width="900" height="354" fill="#e2e8ef" stroke="#8396a7" stroke-width="3"/>',
                 '<rect x="160" y="240" width="600" height="120" fill="#fafcff"/>']
        for rack in self.racks:
            x = 160 + (rack['index'] - 1) * 60
            y = 140 if rack['side'] == 'L' else 360
            parts.append(f'<rect x="{x}" y="{y}" width="60" height="100" fill="{"#758d9f" if rack["sparse"] else "#315b7b"}" stroke="white" stroke-width="2"/>')
            parts.append(f'<text x="{x+30}" y="{y+45}" text-anchor="middle" font-size="16" fill="white">{rack["rack_id"]}</text>')
            parts.append(f'<text x="{x+30}" y="{y+67}" text-anchor="middle" font-size="10" fill="white">QR {rack["qr_count"]}</text>')
        parts += ['<path d="M 110 240 V 360 M 104 240 H 116 M 104 360 H 116" stroke="#16886a" stroke-width="2" fill="none"/>',
                  '<text x="98" y="310" text-anchor="middle" font-size="15" transform="rotate(-90 98 310)">净距 1.20 m</text>',
                  '<path d="M 160 105 H 760 M 160 99 V 111 M 760 99 V 111" stroke="#315b7b" fill="none"/>',
                  '<text x="460" y="92" text-anchor="middle" font-size="16">每列 10 × 0.60 = 6.00 m</text>',
                  '<path d="M 825 240 V 299 M 825 301 V 360" stroke="#b34848" stroke-width="5"/>',
                  '<path d="M 825 299 A 59 59 0 0 0 884 240 M 825 301 A 59 59 0 0 1 884 360" stroke="#2b9a71" stroke-width="2" fill="none" stroke-dasharray="5 4"/>',
                  '<path d="M 825 240 H 884 M 825 360 H 884" stroke="#2b9a71" stroke-width="4" stroke-dasharray="5 3"/>',
                  '<text x="840" y="217" font-size="15">手动双扇门</text>',
                  '<text x="830" y="395" font-size="13">向后方前室开启</text>',
                  '<text x="72" y="300" font-size="15">入口 →</text>',
                  '<text x="165" y="512" font-size="16">单柜宽 0.60 m × 深 1.00 m × 高 2.177 m；正面朝向通道。</text>',
                  '<text x="165" y="542" font-size="15">浅色柜体表示配置较稀疏；空槽保留柜壳；红色为关门，绿色为开门示意。</text>',
                  f'<text x="165" y="572" font-size="15">随机种子 {self.seed}；所有几何、纹理和元数据均随本目录保存。</text>',
                  '</g></svg>']
        (self.output / 'floor_plan.svg').write_text('\n'.join(parts), encoding='utf-8')

    def metadata(self):
        all_units = Counter(str(e['units']) + 'U' for e in self.equipment)
        missing = [e for e in self.equipment if not e.get('qr_items')]
        all_qr = [item for e in self.equipment for item in e.get('qr_items', [])]
        manifest = dict(seed=self.seed, units='m', description='20 cabinet shells, heterogeneous 1U–4U equipment, vacant slots and missing QR labels',
                        dimensions=dict(rack=[.6, 1.0, 2.177], aisle_clear_width=1.2, row_length=6.0, rack_centres_y=[1.1, -1.1],
                                        rack_front_planes_y=[.6, -.6], room_inside_width=3.46, entry_plane_x=-1.1, exit_plane_x=7.9),
                        door=dict(type='double_leaf_hinged', hinge_plane_x=DOOR_X, opening_width=1.3, leaf_width=.59, leaf_thickness=.04,
                                  leaf_bottom_z=.02, leaf_top_z=2.12, max_angle_deg=MAX_DOOR_DEG, opening_direction='+X rear vestibule',
                                  joints=['door_left_hinge', 'door_right_hinge'], actuators=['door_left_motor', 'door_right_motor']),
                        rack_count=len(self.racks), racks_per_row=10, installed_equipment_count=len(self.equipment), equipment_unit_counts=dict(all_units),
                        qr_count=len(all_qr), missing_qr_on_installed_equipment=len(missing), vacant_u_total=sum(r['vacant_u'] for r in self.racks),
                        qr_format='RACK-y/z-height; y is the bottom-to-top QR sequence within that rack, z is the rack QR total, height is metres with two decimals',
                        qr_outer_size_m=[.04, .04], qr_quiet_zone_modules=4, unique_qr_textures=len(self.qr_values),
                        racks=self.racks)
        (self.output / 'layout.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
        (self.output / 'equipment.json').write_text(json.dumps(self.equipment, ensure_ascii=False, indent=2) + '\n')
        columns = ['equipment_id', 'rack_id', 'side', 'rack_index', 'start_u', 'units', 'bottom_m', 'top_m', 'centre_height_m', 'qr_count', 'qr_payloads']
        with (self.output / 'equipment.csv').open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=columns)
            writer.writeheader()
            for equipment in self.equipment:
                writer.writerow({**{key: equipment.get(key) for key in columns}, 'qr_payloads': ';'.join(equipment.get('qr_payloads', []))})
        print(json.dumps({k: v for k, v in manifest.items() if k not in ['racks', 'dimensions', 'door', 'description', 'qr_format']}, ensure_ascii=False, indent=2))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=ROOT)
    parser.add_argument('--seed', type=int, default=20261022)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'assets').mkdir(exist_ok=True)
    shared_textures(args.output / 'assets')
    Builder(args.output, args.seed).build()


if __name__ == '__main__':
    main()
