"""Camera defaults shared by the console, simulation and scene exporter."""
import math

DEFAULT_HFOV = {'top': 115., 'left': 60., 'right': 60.}
RESOLUTIONS = [('HD · 1280×720', (1280, 720)), ('1080p · 1920×1080', (1920, 1080)),
               ('2K · 2560×1440', (2560, 1440)), ('4K · 3840×2160', (3840, 2160))]


def default_camera_controls():
    return {key: value for name, hfov in DEFAULT_HFOV.items()
            for key, value in [(name + '_hfov_deg', hfov), (name + '_width', 3840), (name + '_height', 2160)]}


def camera_projection(width, height, hfov):
    sensor_width = 2 * math.tan(math.radians(hfov) / 2)
    sensor_height = sensor_width * height / width
    return sensor_width, sensor_height, math.degrees(2 * math.atan(sensor_height / 2))
