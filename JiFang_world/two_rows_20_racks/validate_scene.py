#!/usr/bin/env python3
"""Validate layout, movable door, physical passage and package portability."""
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json
import math
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
import numpy as np
from runtime import ROOT, RoomSimulation, mujoco


def probe_passage(scene_dir, open_door):
    root = ET.parse(scene_dir / 'scene.xml').getroot()
    root.find('compiler').set('texturedir', str(scene_dir / 'assets'))
    for include in root.iter('include'):
        include.set('file', str(scene_dir / include.get('file')))
    root.find('option').set('gravity', '0 0 0')
    body = ET.SubElement(root.find('worldbody'), 'body', name='passage_probe', pos='6.05 0 0.4')
    ET.SubElement(body, 'freejoint', name='probe_free')
    ET.SubElement(body, 'geom', name='probe_ball', type='sphere', size='.18', mass='2', rgba='1 0 0 1')
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    data = mujoco.MjData(model)
    data.ctrl[:] = math.radians(100) if open_door else 0
    for _ in range(2000):
        mujoco.mj_step(model, data)
    joint = model.joint('probe_free').id
    qa, va = int(model.jnt_qposadr[joint]), int(model.jnt_dofadr[joint])
    data.qvel[va] = .5
    touched = False
    maximum = float(data.qpos[qa])
    for _ in range(1500):
        mujoco.mj_step(model, data)
        maximum = max(maximum, float(data.qpos[qa]))
        for contact in data.contact:
            names = {model.geom(int(contact.geom1)).name, model.geom(int(contact.geom2)).name}
            if 'probe_ball' in names and any(name.startswith('door_') and name.endswith('_collision') for name in names):
                touched = True
    assert not any(data.warning.number)
    if open_door:
        assert maximum > 7.4 and not touched, (maximum, touched)
    else:
        assert maximum < 6.55 and touched, (maximum, touched)
    return dict(door_open=open_door, probe_diameter_m=.36, maximum_x_m=round(maximum, 6), touched_door=touched, passed=True)


def verify_qr(scene_dir, equipment, decoder_site):
    if decoder_site:
        sys.path.insert(0, str(decoder_site))
    if importlib.util.find_spec('zxingcpp') is None:
        return dict(skipped=True, reason='optional zxing-cpp decoder not installed')
    import zxingcpp
    from PIL import Image
    items = [item for entry in equipment for item in entry.get('qr_items', [])]
    for item in items:
        codes = zxingcpp.read_barcodes(np.array(Image.open(scene_dir / 'assets' / item['qr_texture'])))
        assert len(codes) == 1 and codes[0].text == item['payload'], (item['qr_texture'], item['payload'], [c.text for c in codes])
    sim = RoomSimulation(scene_dir)
    rendered = []
    try:
        for rack_id in sorted({e['rack_id'] for e in equipment}):
            entry = next(e for e in equipment if e['rack_id'] == rack_id and e.get('qr_items'))
            item = entry['qr_items'][0]
            cid = sim.model.camera('door_detail').id
            centre = np.array(item['position_world_m'], dtype=float)
            normal = np.array(entry['front_normal_world'], dtype=float)
            sim.model.cam_pos[cid] = centre + normal * .35
            right = np.cross([0., 0., 1.], normal)
            matrix = np.column_stack([right, [0., 0., 1.], normal])
            q = np.zeros(4); mujoco.mju_mat2Quat(q, matrix.reshape(9)); sim.model.cam_quat[cid] = q; sim.model.cam_fovy[cid] = 12
            mujoco.mj_forward(sim.model, sim.data)
            image = sim.render('door_detail', width=256, height=256)
            codes = zxingcpp.read_barcodes(image)
            assert any(code.text == item['payload'] for code in codes), (rack_id, item['payload'], [c.text for c in codes])
            rendered.append(dict(rack_id=rack_id, equipment_id=item['equipment_id'], payload=item['payload'], passed=True))
    finally:
        sim.close()
    return dict(skipped=False, source_images_decoded=len(items), rendered_checks=rendered, passed=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene-dir', type=Path, default=ROOT)
    parser.add_argument('--decoder-site', type=Path)
    parser.add_argument('--report', type=Path)
    parser.add_argument('--check-rebuild', action='store_true')
    args = parser.parse_args()
    directory = args.scene_dir.resolve()
    layout = json.loads((directory / 'layout.json').read_text())
    equipment = json.loads((directory / 'equipment.json').read_text())
    assert layout['rack_count'] == 20
    assert len(layout['racks']) == 20
    assert {side: sum(r['side'] == side for r in layout['racks']) for side in ['L', 'R']} == {'L': 10, 'R': 10}
    assert {e['units'] for e in equipment} == {1, 2, 3, 4}
    assert len({e['equipment_id'] for e in equipment}) == len(equipment)
    all_qr = [item for e in equipment for item in e.get('qr_items', [])]
    assert len(all_qr) == layout['qr_count']
    assert layout['qr_count'] == sum(e['units'] for e in equipment)
    assert layout['missing_qr_on_installed_equipment'] == 0 and layout['vacant_u_total'] > 0
    plans = []
    for rack in layout['racks']:
        slots = []
        for segment in rack['segments']:
            slots.extend(range(segment['start_u'], segment['start_u'] + segment['units']))
            assert segment['top_m'] <= 1.977
        assert slots == list(range(1, 43)), rack['rack_id']
        plans.append(tuple((s['units'], s['vacant']) for s in rack['segments']))
    assert len(set(plans)) == 20, 'All 20 racks should have distinct layouts'
    for rack in layout['racks']:
        rack_qr = [item for e in equipment if e['rack_id'] == rack['rack_id'] for item in e.get('qr_items', [])]
        assert [item['sequence'] for item in rack_qr] == list(range(1, len(rack_qr) + 1))
        assert all(item['payload'].startswith(f"{rack['rack_id']}-") and item['payload'].split('/')[0].split('-')[1] == str(item['sequence']) and item['payload'].split('/')[0].split('-')[0] == rack['rack_id'] for item in rack_qr)
        assert all(item['payload'].split('/')[1].split('-')[0] == str(len(rack_qr)) for item in rack_qr)
    sim = RoomSimulation(directory)
    try:
        assert sim.model.nq == 2 and sim.model.nu == 2
        assert sim.model.nlight <= 7, 'Leave one fixed OpenGL light for the headlight'
        assert all('robot' not in sim.model.body(i).name for i in range(sim.model.nbody))
        assert len([i for i in range(sim.model.ngeom) if '_qr_' in sim.model.geom(i).name]) == layout['qr_count']
        left = sim.model.geom('L01_collision')
        right = sim.model.geom('R01_collision')
        li = sim.model.geom_bodyid[left.id]
        ri = sim.model.geom_bodyid[right.id]
        clear = sim.data.xpos[li, 1] - left.size[1] - (sim.data.xpos[ri, 1] + right.size[1])
        assert abs(clear - 1.2) < 1e-9
        for target in [100, 0, 100, 0]:
            sim.set_targets([target, target])
            sim.step(4)
            assert np.allclose(sim.angles(), [target, target], atol=.3), sim.angles()
            assert sim.data.ncon == 0, 'Door should not jam against its frame'
            assert not any(sim.data.warning.number)
        sim.set_targets([100, 0])
        sim.step(4)
        assert np.allclose(sim.angles(), [100, 0], atol=.3)
        sim.set_targets([0, 100])
        sim.step(.35)
        held = sim.hold()
        sim.step(2)
        assert np.allclose(sim.angles(), held, atol=.3)
        model_summary = dict(geoms=sim.model.ngeom, bodies=sim.model.nbody, dof=sim.model.nv, actuators=sim.model.nu, world_lights=sim.model.nlight,
                             clear_aisle_m=round(float(clear), 6), repeated_open_close=True, independent_leaves=True, hold_position=True)
    finally:
        sim.close()
    # Real copy, not a symlink: scene must load after changing both path and cwd.
    with tempfile.TemporaryDirectory(prefix='room_portable_') as temporary:
        portable = Path(temporary) / 'relocated_room'
        portable.mkdir()
        for name in ['scene.xml', 'assets.xml']:
            shutil.copy2(directory / name, portable / name)
        shutil.copytree(directory / 'assets', portable / 'assets')
        moved = mujoco.MjModel.from_xml_path(str(portable / 'scene.xml'))
        assert moved.ngeom == model_summary['geoms']
    for path in [directory / 'scene.xml', directory / 'assets.xml']:
        for element in ET.parse(path).iter():
            filename = element.get('file')
            if filename:
                assert not Path(filename).is_absolute() and '..' not in Path(filename).parts
    report = dict(passed=True, seed=layout['seed'], layout=dict(racks=20, distinct_layouts=len(set(plans)), equipment=len(equipment),
                  qr_count=layout['qr_count'], missing_qr=layout['missing_qr_on_installed_equipment'], vacant_u=layout['vacant_u_total']),
                  model=model_summary, collision_tests=[probe_passage(directory, False), probe_passage(directory, True)],
                  portable_copy_loads=True, all_scene_asset_paths_relative=True,
                  qr=verify_qr(directory, equipment, args.decoder_site))
    if args.check_rebuild:
        with tempfile.TemporaryDirectory(prefix='room_rebuild_') as temporary:
            dest = Path(temporary)
            subprocess.run([sys.executable, str(directory / 'build_scene.py'), '--output', str(dest), '--seed', str(layout['seed'])], check=True, stdout=subprocess.DEVNULL)
            for name in ['scene.xml', 'assets.xml', 'layout.json', 'equipment.json', 'equipment.csv']:
                assert hashlib.sha256((dest / name).read_bytes()).digest() == hashlib.sha256((directory / name).read_bytes()).digest(), name
        report['same_seed_deterministic_rebuild'] = True
    destination = args.report or directory / 'validation.json'
    destination.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'qr'}, ensure_ascii=False, indent=2))
    if not report['qr']['skipped']:
        print('QR verified:', report['qr']['source_images_decoded'], 'source textures;', len(report['qr']['rendered_checks']), 'rack views')


if __name__ == '__main__':
    main()
