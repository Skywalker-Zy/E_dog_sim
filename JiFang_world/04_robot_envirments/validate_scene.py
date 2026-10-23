"""Check the combined scene, manual doors and three camera renders."""
from pathlib import Path
import json
import numpy as np
from PIL import Image
from simulation import ROOT, RobotRoomSimulation, default_controls, CAMERAS
from runtime import mujoco


def main():
    previews = ROOT / 'previews'
    verification = ROOT / 'verification'
    previews.mkdir(exist_ok=True)
    verification.mkdir(exist_ok=True)
    sim = RobotRoomSimulation(ROOT)
    try:
        model, data = sim.model, sim.data
        assert model.njnt == 2 and model.nu == 2 and model.nmocap == 1
        for side in ['L', 'R']:
            for index in range(1, 11):
                model.geom(f'{side}{index:02d}_collision')
        np.testing.assert_allclose(data.xpos[model.body('robot_base').id], [.9, 0, 0])
        positions = {}
        for name, expected in [('top', [.9, 0, 1.325]), ('left', [.9, .21, .7]), ('right', [.9, -.21, .7])]:
            p = data.cam_xpos[model.camera(name + '_inspection_camera').id]
            np.testing.assert_allclose(p, expected, atol=1e-12)
            positions[name] = p.tolist()
        for camera in ['robot_overview', 'aisle', 'top', 'top_inspection_camera', 'left_inspection_camera', 'right_inspection_camera']:
            sensor = camera.endswith('_inspection_camera')
            image = sim.render(camera, width=1920 if sensor else 1200, height=1080 if sensor else 750)
            assert image.std() > 1
            assert bool(sim.option.geomgroup[2]) == (not sensor)
            Image.fromarray(image).save(previews / (camera + '.png'))
        # Decode actual rendered codes; disabled recognition must skip the decoder.
        from unittest.mock import patch
        import zxingcpp
        image, detected = sim.capture('left', True)
        assert detected and all(item['rack_id'] == 'L02' for item in detected)
        with patch.object(zxingcpp, 'read_barcodes', side_effect=AssertionError('decoder called while disabled')):
            _, disabled = sim.capture('left', False)
        assert disabled == []
        controls = default_controls()
        controls.update(robot_x_m=1.5, robot_y_m=.05, robot_yaw_deg=30, pole_length_m=1.2)
        for index, name in enumerate(CAMERAS):
            controls[name + '_yaw_deg'] = 20 + index * 50
            controls[name + '_pitch_deg'] = -15 - index * 10
        sim.apply_controls(controls)
        angle = np.radians(30)
        rotation = np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
        for name, local in [('top', [0, 0, 1.925]), ('left', [0, .21, .7]), ('right', [0, -.21, .7])]:
            camera_id = model.camera(name + '_inspection_camera').id
            np.testing.assert_allclose(data.cam_xpos[camera_id], np.array([1.5, .05, 0]) + rotation @ local, atol=1e-12)
            yaw, pitch = np.radians([controls[name + '_yaw_deg'], controls[name + '_pitch_deg']])
            forward = rotation @ [-np.sin(yaw) * np.cos(pitch), np.cos(yaw) * np.cos(pitch), np.sin(pitch)]
            np.testing.assert_allclose(-data.cam_xmat[camera_id].reshape(3, 3)[:, 2], forward, atol=1e-12)
        np.testing.assert_allclose(model.geom_size[model.geom('telescopic_pole').id, 1], .6)
        sim.reset()
        assert sim.controls == default_controls()
        sim.set_targets([100, 100])
        sim.step(3.)
        opened = sim.angles()
        assert min(opened) > 98
        sim.set_targets([0, 0])
        sim.step(3.)
        closed = sim.angles()
        assert max(abs(angle) for angle in closed) < 2
        base = model.body('robot_base').mocapid[0]
        data.mocap_pos[base] = [1.5, 0, 0]
        data.mocap_quat[base] = [np.cos(np.pi/4), 0, 0, np.sin(np.pi/4)]
        mujoco.mj_forward(model, data)
        np.testing.assert_allclose(data.cam_xpos[model.camera('left_inspection_camera').id], [1.29, 0, .7], atol=1e-12)
        sim.reset()
        np.testing.assert_allclose(data.xpos[model.body('robot_base').id], [.9, 0, 0])
        result = dict(default_left_decoded=[item['payload'] for item in detected], rack_count=20, camera_positions_m=positions, side_camera_to_torso_side_m=.05,
                      swept_width_m=.52, clearance_to_each_row_m=.34, door_open_angles_deg=opened,
                      door_closed_angles_deg=closed, geometry_count=model.ngeom,
                      checks=['20 rack collision proxies', 'three camera positions', 'six views rendered',
                              'camera self-housing hidden in sensor views', 'doors open and close',
                              'mocap translation/yaw camera following', 'reset restores robot pose', 'all camera yaw/pitch optical axes',
                              'mast extension with fixed side camera height', 'actual QR recognition and metadata',
                              'disabled recognition skips decoder'])
        (verification / 'validation.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2))
    finally:
        sim.close()


if __name__ == '__main__':
    main()
