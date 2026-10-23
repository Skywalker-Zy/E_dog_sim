"""Exercise pose, camera, QR and single-button door controls offscreen."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
os.environ.setdefault('MUJOCO_GL', 'egl')
import json
import time
from viewer import create_window, QtWidgets, ROOT, default_controls, CAMERAS


def main():
    app = QtWidgets.QApplication([])
    app.setStyle('Fusion')
    window = create_window()
    window.show()

    def wait_current():
        deadline = time.monotonic() + 40
        while time.monotonic() < deadline:
            app.processEvents()
            if window.last_payload is not None and window.last_payload['revision'] == window.revision:
                return window.last_payload
            if '运行错误' in window.notice.text():
                raise AssertionError(window.notice.text())
            time.sleep(.02)
        raise AssertionError('GUI frame timeout: ' + window.notice.text())

    def click(text):
        buttons = [button for button in window.findChildren(QtWidgets.QPushButton) if button.text() == text]
        assert len(buttons) == 1, text
        buttons[0].click()

    try:
        wait_current()
        window.inputs['robot_yaw_deg'].setValue(90)
        click('前进')
        assert abs(window.inputs['robot_y_m'].value() - .1) < 1e-12
        click('左移')
        assert abs(window.inputs['robot_x_m'].value() - .8) < 1e-12
        click('后退')
        click('右移')
        assert abs(window.inputs['robot_x_m'].value() - .9) < 1e-12
        assert abs(window.inputs['robot_y_m'].value()) < 1e-12
        click('左转'); assert window.inputs['robot_yaw_deg'].value() == 95
        click('右转'); assert window.inputs['robot_yaw_deg'].value() == 90
        for index, name in enumerate(CAMERAS):
            window.inputs[name + '_yaw_deg'].setValue(index * 30)
            window.inputs[name + '_pitch_deg'].setValue(-20 - index * 5)
        window.inputs['pole_length_m'].setValue(1.2)
        payload = wait_current()
        assert payload['controls'] == window.controls()
        assert '1.925' in window.top_height.text()
        for name in CAMERAS:
            window.camera_select.setCurrentIndex(window.camera_select.findData(name))
            assert wait_current()['camera'] == name
        # Test fast edits while an older frame may still be rendering.
        for x in [.95, 1.0, 1.05, 1.1]:
            window.inputs['robot_x_m'].setValue(x)
            app.processEvents()
        assert wait_current()['controls']['robot_x_m'] == 1.1
        window.reset_controls()
        assert wait_current()['controls'] == default_controls()
        window.camera_select.setCurrentIndex(window.camera_select.findData('left'))
        window.detect_checkbox.setChecked(True)
        payload = wait_current()
        assert payload['detections'] and 'L02-' in window.results.toPlainText()
        window.grab().save(str(ROOT / 'previews/gui.png'))
        window.save_capture()
        directory = max((ROOT / 'captures').iterdir(), key=lambda path: path.name)
        saved = json.loads((directory / 'settings.json').read_text())
        assert saved['controls'] == payload['controls'] and saved['detections'] == payload['detections']
        assert (directory / 'left.png').is_file()
        window.detect_checkbox.setChecked(False)
        assert wait_current()['detections'] == []
        assert window.results.toPlainText() == '未启用二维码识别'
        click('开门')
        payload = wait_current()
        assert payload['door_open'] and min(payload['door_angles']) > 0
        click('关门')
        assert not wait_current()['door_open']
        checks = ['single door toggle button', 'body-relative forward/back/left/right', 'both rotation buttons',
                  'six camera angles and mast controls', 'three preview camera selections',
                  'rapid edits keep matching image revision', 'reset pose/camera controls',
                  'QR overlay/results enabled and cleared when disabled', 'saved images match capture settings',
                  'background renderer shuts down cleanly']
        (ROOT / 'verification/gui_validation.json').write_text(json.dumps({'checks': checks}, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps(checks, ensure_ascii=False, indent=2))
    finally:
        window.close()
        app.processEvents()


if __name__ == '__main__':
    main()
