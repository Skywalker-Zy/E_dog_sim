#!/usr/bin/env python3
"""Generate an MJCF cabinet scene with the bundled QR encoder."""
from pathlib import Path
import argparse
import csv
import json
import sys
import math
import struct
import zlib
import xml.etree.ElementTree as ET

WIDTH = 0.600
DEPTH = 1.000
HEIGHT = 2.177
# Heights transcribed from the annotated reference photograph, in metres.
LEVELS = [0.233, 0.451, 0.669, 0.887, 1.105, 1.325, 1.541, 1.759, 1.977]
BASE_HEIGHT = 0.100
SEPARATOR_HEIGHT = 0.040
QR_SIZE = 0.040  # Includes the required white quiet zone.
QR_FRONT_X = -0.170
QR_FRONT_Y = -0.488


def server_rows():
    sections = [(BASE_HEIGHT, LEVELS[0], 3)]
    sections.extend((LEVELS[i - 1] + SEPARATOR_HEIGHT, LEVELS[i], 4) for i in range(1, len(LEVELS)))
    result = []
    for bank, (low, high, units) in enumerate(sections):
        for row in range(units):
            h = (high - low) / units
            centre = low + (row + .5) * h
            index = len(result) + 1
            result.append(dict(index=index, name=f'server_{bank:02}_{row:02}', bank=bank, row=row,
                               bottom_m=round(low + row * h, 9), top_m=round(low + (row + 1) * h, 9),
                               centre_m=round(centre, 9), qr_payload=f'{centre:.6f}',
                               qr_width_m=QR_SIZE, qr_height_m=QR_SIZE,
                               qr_front_xyz=[QR_FRONT_X, QR_FRONT_Y, round(centre, 9)],
                               texture=f'qr_{index:02}.png'))
    return result


def make_qr_textures(directory):
    # Vendored BSD-licensed python-qrcode: no installation or network is needed.
    sys.path.insert(0, str(Path(__file__).resolve().parent / 'vendor'))
    import qrcode
    font = {
        '0': ['111','101','101','101','111'], '1': ['010','110','010','010','111'],
        '2': ['111','001','111','100','111'], '3': ['111','001','111','001','111'],
        '4': ['101','101','111','001','001'], '5': ['111','100','111','001','111'],
        '6': ['111','100','111','101','111'], '7': ['111','001','010','010','010'],
        '8': ['111','101','111','101','111'], '9': ['111','101','111','001','111'],
        '.': ['000','000','000','000','010'], 'm': ['000','111','111','101','101'],
    }
    for item in server_rows():
        qr = qrcode.QRCode(version=1, error_correction=qrcode.constants.ERROR_CORRECT_M, box_size=12, border=4)
        qr.add_data(item['qr_payload'])
        qr.make(fit=True)
        matrix = qr.get_matrix()
        n = len(matrix) * 12
        data = bytearray()
        for y in range(n):
            for x in range(n):
                value = 0 if matrix[y // 12][x // 12] else 255
                data.extend((value, value, value))
        png_rgb(directory / item['texture'], n, n, data)
        # A separate human-readable label. It does not intrude into the QR area.
        label = item['qr_payload'] + 'm'
        w, h = len(label) * 16 + 8, 28
        data = bytearray([245] * (w * h * 3))
        for i, char in enumerate(label):
            for gy, line in enumerate(font[char]):
                for gx, bit in enumerate(line):
                    if bit == '1':
                        for dy in range(4):
                            for dx in range(4):
                                x, y = 4 + i * 16 + gx * 4 + dx, 4 + gy * 4 + dy
                                offset = (y * w + x) * 3
                                data[offset:offset+3] = bytes((20, 28, 36))
        png_rgb(directory / f"height_{item['index']:02}.png", w, h, data)


def png_rgb(path, width, height, pixels):
    def chunk(kind, data):
        return struct.pack('!I', len(data)) + kind + data + struct.pack('!I', zlib.crc32(kind + data) & 0xffffffff)
    raw = b''.join(b'\x00' + bytes(pixels[y * width * 3:(y + 1) * width * 3]) for y in range(height))
    path.write_bytes(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('!2I5B', width, height, 8, 2, 0, 0, 0)) + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))


def make_textures(directory):
    # Grating is a visual texture over a continuous, flat collision plane.
    n = 512
    data = bytearray()
    for y in range(n):
        for x in range(n):
            border = min(x, y, n - 1 - x, n - 1 - y)
            if border < 6:
                rgb = (148, 155, 158)
            elif border < 12:
                rgb = (48, 54, 59)
            elif x % 16 < 4 or y % 72 < 5:
                rgb = (127, 136, 141)
            else:
                rgb = (30, 36, 40)
            data.extend(rgb)
    png_rgb(directory / 'floor_grate.png', n, n, data)
    data = bytearray()
    for y in range(128):
        for x in range(512):
            # Repeated perforations on the front of each rack unit.
            if x < 8 or x > 503 or y < 8 or y > 119:
                rgb = (33, 39, 45)
            elif 100 < x < 435 and ((x % 12 - 6) ** 2 + (y % 12 - 6) ** 2 < 7):
                rgb = (5, 8, 12)
            else:
                rgb = (49, 55, 62)
            data.extend(rgb)
    png_rgb(directory / 'server_face.png', 512, 128, data)


def add(parent, tag, **attrs):
    def fmt(v):
        if isinstance(v, (list, tuple)):
            return ' '.join(fmt(i) for i in v)
        return f'{v:.6f}'.rstrip('0').rstrip('.') if isinstance(v, float) else str(v)
    return ET.SubElement(parent, tag, {k: fmt(v) for k, v in attrs.items()})


def box(parent, name, pos, size, material, **attrs):
    return add(parent, 'geom', name=name, type='box', pos=pos, size=size, material=material, **attrs)


def capsule(parent, name, a, b, radius, material):
    return add(parent, 'geom', name=name, type='capsule', fromto=(*a, *b), size=radius, material=material)


def write_xml(path, root):
    ET.indent(root, space='  ')
    path.write_text('<?xml version="1.0" encoding="utf-8"?>\n' + ET.tostring(root, encoding='unicode') + '\n', encoding='utf-8')


def assets():
    root = ET.Element('mujocoinclude')
    asset = add(root, 'asset')
    add(asset, 'texture', name='sky', type='skybox', builtin='gradient', rgb1='0.28 0.34 0.41', rgb2='0.66 0.7 0.75', width=512, height=3072)
    add(asset, 'texture', name='floor_tiles', type='2d', builtin='checker', rgb1='0.51 0.54 0.56', rgb2='0.55 0.58 0.60', width=512, height=512)
    add(asset, 'texture', name='grate_texture', type='2d', file='floor_grate.png')
    add(asset, 'texture', name='server_texture', type='2d', file='server_face.png')
    materials = {
        'silver': ('0.62 0.66 0.70 1', .45),
        'dark_metal': ('0.065 0.08 0.10 1', .25),
        'panel': ('0.11 0.13 0.16 1', .2),
        'side_panel': ('0.24 0.27 0.30 1', .3),
        'black': ('0.018 0.023 0.03 1', .05),
        'blue': ('0.015 0.31 0.66 1', .35),
        'yellow': ('0.88 0.65 0.08 1', .2),
        'label': ('0.86 0.88 0.85 1', .1),
        'wall': ('0.72 0.75 0.76 1', .05),
        'wall_trim': ('0.39 0.44 0.48 1', .1),
        'door': ('0.18 0.23 0.27 1', .3),
    }
    for name, (rgba, specular) in materials.items():
        add(asset, 'material', name=name, rgba=rgba, specular=specular, shininess=.3)
    add(asset, 'material', name='led_green', rgba='0.1 0.9 0.27 1', emission=.65)
    add(asset, 'material', name='led_blue', rgba='0.04 0.5 1 1', emission=.6)
    add(asset, 'material', name='lamp', rgba='0.95 0.97 1 1', emission=.8)
    add(asset, 'material', name='floor', texture='floor_tiles', texrepeat='8 10', texuniform='false', reflectance=.03)
    add(asset, 'material', name='grate', texture='grate_texture', texrepeat='4 5', texuniform='false', specular=.25)
    add(asset, 'material', name='server_front', texture='server_texture', texrepeat='1 1', texuniform='false', specular=.2)
    for item in server_rows():
        idx = item['index']
        for prefix, filename in [('qr', item['texture']), ('height', f'height_{idx:02}.png')]:
            add(asset, 'texture', name=f'{prefix}_texture_{idx:02}', type='2d', file=filename, hflip='false')
            add(asset, 'material', name=f'{prefix}_material_{idx:02}', texture=f'{prefix}_texture_{idx:02}', texrepeat='1 1', texuniform='false', rgba='1 1 1 1', specular=0, shininess=0)
    return root


def cabinet():
    root = ET.Element('mujocoinclude')
    body = add(root, 'body', name='cabinet', pos='0 0 0')
    # One static collision proxy; decorative geometry belongs to group 1.
    add(body, 'geom', name='cabinet_collision', type='box', pos=(0, 0, HEIGHT / 2), size=(WIDTH / 2, DEPTH / 2, HEIGHT / 2), contype=1, conaffinity=1, group=3, rgba='0 0 0 0', friction='0.8 0.01 0.001', mass=0)
    visual = add(body, 'body', name='cabinet_visual', childclass='decoration')
    box(visual, 'base', (0, 0, .05), (.3, .5, .05), 'silver')
    box(visual, 'base_front_inset', (0, -.501, .052), (.244, .002, .025), 'dark_metal')
    box(visual, 'roof', (0, 0, HEIGHT - .015), (.3, .5, .015), 'silver')
    for x in [-.285, .285]:
        for y in [-.485, .485]:
            suffix = ('left' if x < 0 else 'right') + ('_front' if y < 0 else '_rear')
            box(visual, 'post_' + suffix, (x, y, HEIGHT / 2), (.015, .015, HEIGHT / 2), 'silver')
    for x in [-.294, .294]:
        side = 'left' if x < 0 else 'right'
        box(visual, side + '_side', (x, .01, 1.127), (.005, .458, 1.0), 'side_panel')
        for z in [.24, 1.94]:
            box(visual, side + '_side_seam_' + str(z), (x * 1.021, .01, z), (.0008, .425, .002), 'black')
    box(visual, 'rear_panel', (0, .492, 1.127), (.269, .005, 1.0), 'dark_metal')
    # Internal 19-inch faces; open front with no door.
    for x in [-.258, .258]:
        side = 'left' if x < 0 else 'right'
        box(visual, 'mounting_rail_' + side, (x, -.48, 1.045), (.012, .014, .945), 'silver')
        for i in range(43):
            z = .145 + i * .04445
            if z < 2.04:
                box(visual, f'rail_hole_{side}_{i:02}', (x, -.495, z), (.003, .0008, .0035), 'black')
    sections = [(BASE_HEIGHT, LEVELS[0], 3)]
    for i in range(1, len(LEVELS)):
        low = LEVELS[i - 1]
        high = LEVELS[i]
        box(visual, f'separator_{i:02}', (0, -.012, low + SEPARATOR_HEIGHT / 2), (.27, .478, SEPARATOR_HEIGHT / 2), 'silver')
        box(visual, f'separator_shadow_{i:02}', (0, -.492, low + .006), (.242, .002, .002), 'dark_metal')
        sections.append((low + SEPARATOR_HEIGHT, high, 4))
    for i, (low, high, units) in enumerate(sections):
        h = high - low
        box(visual, f'server_bank_{i:02}', (0, -.02, (low + high) / 2), (.2413, .45, h / 2 - .001), 'black')
        for row in range(units):
            z = low + (row + .5) * h / units
            unit_h = h / units - .003
            name = f'server_{i:02}_{row:02}'
            box(visual, name + '_face', (0, -.479, z), (.240, .006, unit_h / 2), 'server_front')
            # Long horizontal dark ventilation slots, two drive groups and handles.
            for dz in [-.006, .006]:
                box(visual, name + '_slot_' + str(dz), (.008, -.4854, z + dz), (.151, .0007, .0016), 'black')
            for x in [-.216, .214]:
                box(visual, name + '_handle_' + str(x), (x, -.490, z), (.005, .004, .013), 'silver')
            for k in range(2):
                add(visual, 'geom', name=f'{name}_led_{k}', type='sphere', pos=(.183 + k * .011, -.487, z + .004), size=.0022, material='led_green' if k == 0 else 'led_blue')
    # Top cable-management region: annotated 1.977 to 2.177 m.
    box(visual, 'top_recess', (0, -.421, 2.069), (.245, .025, .077), 'black')
    for i in range(11):
        x = -.218 + i * .0435
        box(visual, f'top_patch_port_{i}', (x, -.451, 2.037), (.012, .006, .023), 'yellow')
        # Each cable is an individual polyline, giving the blue bundle depth.
        points = [(x, -.466, 2.059), (x, -.482, 2.092), (x * .9, -.484, 2.12), (.229, -.483 + .0007 * i, 2.12 + i * .0016)]
        for k in range(len(points) - 1):
            capsule(visual, f'top_cable_{i}_{k}', points[k], points[k + 1], .0028, 'blue')
    for side, x in [('left', -.247), ('right', .247)]:
        for i, z in enumerate(LEVELS[:-1]):
            box(visual, f'patch_tag_{side}_{i}', (x, -.503, z + .016), (.008, .004, .012), 'yellow')
    # Cable loops in front of the lower rack: all stay inside the 0.60 x 1.00 m envelope.
    for cable in range(7):
        points = []
        for k in range(17):
            t = k / 16
            points.append((-.246 + .035 * math.sin(math.pi * t), -.491 - .004 * math.sin(math.pi * t), .69 - .36 * t + cable * .007))
        for k in range(16):
            capsule(visual, f'lower_cable_{cable}_{k}', points[k], points[k + 1], .0022, 'blue')
    for z in [.379, .535, .67]:
        box(visual, 'cable_strap_' + str(z), (-.235, -.497, z), (.012, .001, .005), 'black')
    # One QR on the left of each front panel, with a height label to its right.
    for item in server_rows():
        idx, z = item['index'], item['centre_m']
        box(visual, f'qr_front_{idx:02}', (QR_FRONT_X, QR_FRONT_Y, z), (QR_SIZE / 2, .0004, QR_SIZE / 2), f'qr_material_{idx:02}')
        box(visual, f'height_label_front_{idx:02}', (-.107, QR_FRONT_Y, z), (.038, .0004, .007), f'height_material_{idx:02}')
    # Named sites for downstream inspection/navigation tasks (hidden by default).
    add(body, 'site', name='cabinet_front_target', pos='0 -0.52 1.105', type='sphere', size=.012, rgba='0 0 0 0', group=4)
    add(body, 'site', name='cabinet_top', pos=(0, 0, HEIGHT), size=.01, rgba='0 0 0 0', group=4)
    return root


def scene(room=True):
    root = ET.Element('mujoco', model='JiFang_single_cabinet' if room else 'JiFang_cabinet_only')
    add(root, 'compiler', angle='degree', texturedir='assets', autolimits='true')
    add(root, 'option', timestep=.002, gravity='0 0 -9.81', integrator='implicitfast')
    visual = add(root, 'visual')
    add(visual, 'global', azimuth=120, elevation=-16, offwidth=1600, offheight=1200)
    add(visual, 'headlight', ambient='.35 .35 .35', diffuse='.65 .65 .65', specular='.25 .25 .25')
    add(visual, 'quality', shadowsize=2048)
    add(visual, 'map', znear=.01, zfar=40)
    default = add(root, 'default')
    add(default, 'geom', friction='.8 .01 .001', solref='.02 1')
    deco = add(default, 'default', **{'class':'decoration'})
    add(deco, 'geom', contype=0, conaffinity=0, group=1, mass=0)
    add(root, 'include', file='rack_assets.xml')
    add(root, 'statistic', center='0 -0.25 1.05', extent=2.1)
    world = add(root, 'worldbody')
    add(world, 'light', name='key', pos='-1.5 -2 4', dir='.2 .3 -1', diffuse='.65 .65 .65', castshadow='true')
    add(world, 'light', name='fill', pos='2 -1 3', dir='-.3 .2 -1', diffuse='.4 .43 .48', castshadow='false')
    add(world, 'camera', name='overview', pos='2.8 -4.5 2.45', xyaxes='.849 .528 0 -.145 .234 .961', fovy=44)
    add(world, 'camera', name='front', pos='0 -4.2 1.23', xyaxes='1 0 0 0 0 1', fovy=36)
    add(world, 'camera', name='qr_detail_front', pos=(-.1265, -.74, server_rows()[17]['centre_m']), xyaxes='1 0 0 0 0 1', fovy=24)
    add(world, 'camera', name='front_panels', pos=(0, -1.30, server_rows()[17]['centre_m']), xyaxes='1 0 0 0 0 1', fovy=24)
    add(world, 'camera', name='inspection', pos='0 -1.4 .75', xyaxes='1 0 0 0 0 1', fovy=65)
    add(world, 'geom', name='floor_collision', type='plane', size='3 3 .1', material='floor', group=0)
    add(world, 'include', file='cabinet.xml')
    if room:
        environment = add(world, 'body', name='room')
        box(environment, 'back_wall', (0, 1.5, 1.4), (2.1, .04, 1.4), 'wall')
        box(environment, 'left_wall', (-2.1, -.5, 1.4), (.04, 2.04, 1.4), 'wall')
        deco_body = add(environment, 'body', name='room_decoration', childclass='decoration')
        box(deco_body, 'grating_walkway', (0, -.9, .0003), (1.2, 1.5, .0002), 'grate')
        box(deco_body, 'back_skirting', (0, 1.453, .05), (2.06, .008, .05), 'wall_trim')
        box(deco_body, 'left_skirting', (-2.052, -.5, .05), (.008, 2, .05), 'wall_trim')
        for x in [.55, 1.55]:
            box(deco_body, 'door_frame_' + str(x), (x, 1.441, 1.075), (.023, .018, 1.075), 'silver')
        box(deco_body, 'door_header', (1.05, 1.441, 2.15), (.523, .018, .023), 'silver')
        box(deco_body, 'closed_door', (1.05, 1.445, 1.06), (.475, .008, 1.06), 'door')
        box(deco_body, 'door_handle', (1.40, 1.42, 1.0), (.012, .012, .11), 'silver')
        for x in [-.45, .45]:
            box(deco_body, 'ceiling_light_frame_' + str(x), (x, -.6, 2.75), (.08, .65, .018), 'silver')
            box(deco_body, 'ceiling_light_' + str(x), (x, -.6, 2.728), (.058, .60, .003), 'lamp')
    return root


def dimension_svg():
    # Standalone editable dimension drawing; no raster assets needed.
    scale = 260
    x0, y0 = 150, 65
    w, h = WIDTH * scale, HEIGHT * scale
    base = y0 + h
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="850" height="760" viewBox="0 0 850 760">', '<rect width="850" height="760" fill="#f4f7fb"/>', '<g font-family="sans-serif" fill="#203040">', '<text x="40" y="35" font-size="23">单机柜尺寸 / Single cabinet (m)</text>', f'<rect x="{x0}" y="{y0}" width="{w}" height="{h}" fill="#aeb8c1" stroke="#34495e" stroke-width="2"/>']
    sections = [(BASE_HEIGHT, LEVELS[0], '3U')]
    sections.extend((LEVELS[i - 1] + SEPARATOR_HEIGHT, LEVELS[i], '4U') for i in range(1, len(LEVELS)))
    for low, high, label in sections:
        yy = base - high * scale
        hh = (high - low) * scale
        parts.append(f'<rect x="{x0+12}" y="{yy}" width="{w-24}" height="{hh}" fill="#23303d"/>')
        parts.append(f'<text x="{x0+w/2}" y="{yy+hh/2+4}" fill="white" font-size="12" text-anchor="middle">{label}</text>')
    for value in [0, .1] + LEVELS + [HEIGHT]:
        yy = base - value * scale
        parts.append(f'<path d="M {x0+w+5} {yy} H 375" stroke="#71899e"/><text x="383" y="{yy+4}" font-size="12">{value:.3f}</text>')
    parts.append(f'<path d="M 100 {y0} V {base} M 94 {y0} H 106 M 94 {base} H 106" stroke="#1684b8" fill="none"/><text x="90" y="360" font-size="17" transform="rotate(-90 90 360)">H = 2.177 m</text>')
    parts.append(f'<text x="150" y="{base+32}" font-size="16">W = 0.600 m (assumed)</text>')
    parts.append('<rect x="520" y="65" width="260" height="566.02" fill="#56626f" stroke="#34495e" stroke-width="2"/><text x="550" y="663" font-size="16">D = 1.000 m (assumed)</text>')
    parts.append('<text x="40" y="706" font-size="15">标注高度优先；4U 区含约 4 cm 隔板，详见 README。</text><text x="40" y="735" font-size="15">Front: -Y / Up: +Z / Cabinet centre: (0, 0)</text></g></svg>')
    return '\n'.join(parts)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parent)
    out = parser.parse_args().output
    out.mkdir(parents=True, exist_ok=True)
    (out / 'assets').mkdir(exist_ok=True)
    make_textures(out / 'assets')
    make_qr_textures(out / 'assets')
    write_xml(out / 'rack_assets.xml', assets())
    write_xml(out / 'cabinet.xml', cabinet())
    write_xml(out / 'scene.xml', scene(True))
    write_xml(out / 'cabinet_only.xml', scene(False))
    (out / 'dimensions.svg').write_text(dimension_svg(), encoding='utf-8')
    (out / 'server_heights.json').write_text(json.dumps({'height_reference':'centre of each 1U chassis above floor', 'unit':'m', 'qr_payload_format':'decimal number, six decimal places, unit m', 'qr_size_m':[QR_SIZE, QR_SIZE], 'qr_placement':'front panel left, facing -Y, one per 1U chassis', 'servers':server_rows()}, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    with (out / 'server_heights.csv').open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=['index','name','bottom_m','centre_m','top_m','qr_payload','qr_front_x_m','qr_front_y_m','qr_front_z_m','texture'])
        writer.writeheader()
        for row in server_rows():
            coordinates = dict(zip(['qr_front_x_m','qr_front_y_m','qr_front_z_m'], row['qr_front_xyz']))
            writer.writerow({k:row[k] if k in row else coordinates[k] for k in writer.fieldnames})
    print(f'Generated scene in {out}')


if __name__ == '__main__':
    main()
