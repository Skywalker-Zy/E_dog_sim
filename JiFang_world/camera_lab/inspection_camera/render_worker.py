"""All OpenGL contexts and decoding live on one dedicated worker thread."""
from copy import deepcopy
from dataclasses import asdict
import time
from PyQt5 import QtCore
from .core import CameraEngine, Settings, mujoco


class RenderWorker(QtCore.QObject):
    frame_ready = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    stopped = QtCore.pyqtSignal()

    def __init__(self, scene_dir):
        super().__init__()
        self.scene_dir = scene_dir
        self.engine = None
        self.overview_renderer = None
        self.cached_key = self.cached_frame = None
        self.overview_key = self.overview_image = None
        self.previous_token = None
        self.renderer_name = ''

    @QtCore.pyqtSlot(object)
    def render(self, request):
        started = time.perf_counter()
        try:
            settings = Settings().updated(request['settings'])
            if self.engine is None:
                self.engine = CameraEngine(self.scene_dir, settings)
            self.engine.apply(settings)
            key = tuple((k, v) for k, v in asdict(settings).items() if k != 'fps')
            reused = bool(request['reuse_static_frame'] and self.cached_key == key)
            if reused:
                image, base_result = self.cached_frame
                result = deepcopy(base_result)
                result['settings'] = asdict(settings)
            else:
                image, result = self.engine.capture()
                # Snapshot copies: only this worker owns the underlying cache.
                self.cached_key, self.cached_frame = key, (image, deepcopy(result))
                if not self.renderer_name:
                    from OpenGL import GL
                    self.engine.renderer._gl_context.make_current()
                    self.renderer_name = GL.glGetString(GL.GL_RENDERER).decode()
            overview = None
            overview_state = request['overview']
            overview_key = (key, tuple(overview_state['lookat']), overview_state['distance'], overview_state['azimuth'], overview_state['elevation'])
            if self.overview_key != overview_key:
                if self.overview_renderer is None:
                    # Allocate a small framebuffer for the diagram, even when
                    # the inspection sensor is configured for 4K.
                    visual = self.engine.model.vis.global_
                    previous = (visual.offwidth, visual.offheight)
                    try:
                        visual.offwidth, visual.offheight = 600, 300
                        self.overview_renderer = mujoco.Renderer(self.engine.model, height=300, width=600)
                    finally:
                        visual.offwidth, visual.offheight = previous
                camera = mujoco.MjvCamera()
                camera.type = mujoco.mjtCamera.mjCAMERA_FREE
                camera.lookat[:] = overview_state['lookat']
                for name in ['distance', 'azimuth', 'elevation']:
                    setattr(camera, name, overview_state[name])
                self.overview_renderer.update_scene(self.engine.data, camera=camera)
                # Diagram shadows/reflections do not affect the inspection image.
                self.overview_renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
                self.overview_renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
                self.engine.add_overview_markers(self.overview_renderer.scene)
                self.overview_image = self.overview_renderer.render().copy()
                self.overview_key = overview_key
                overview = self.overview_image
            token = request['display_token']
            result['cached_frame'] = reused
            result['opengl_renderer'] = self.renderer_name
            result['worker_seconds'] = round(time.perf_counter() - started, 5)
            self.frame_ready.emit(dict(request=request, image=image, result=result, overview=self.overview_image, overview_token=overview_key,
                                       display_changed=(token != self.previous_token or not reused)))
            self.previous_token = token
        except Exception as error:
            self.failed.emit(f'{type(error).__name__}: {error}')

    @QtCore.pyqtSlot()
    def shutdown(self):
        try:
            if self.overview_renderer:
                self.overview_renderer.close()
                self.overview_renderer = None
            if self.engine:
                self.engine.close()
                self.engine = None
        finally:
            self.stopped.emit()
