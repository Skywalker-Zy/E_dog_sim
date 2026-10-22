from copy import deepcopy
from dataclasses import asdict
import time
from PyQt5 import QtCore
from .robot_platform import RobotCameraEngine,RobotSettings,CAMERAS,mujoco


class PlatformWorker(QtCore.QObject):
    ready=QtCore.pyqtSignal(object)
    failed=QtCore.pyqtSignal(str)
    stopped=QtCore.pyqtSignal()
    def __init__(self,scene_dir):
        super().__init__();self.scene_dir=scene_dir;self.engine=None;self.cache_key=None;self.cached=None
        self.overview_renderer=None;self.overview_key=None;self.overview_image=None;self.renderer_name=''
    @QtCore.pyqtSlot(object)
    def render(self,request):
        try:
            start=time.perf_counter();settings=RobotSettings().updated(request['settings'])
            if self.engine is None:self.engine=RobotCameraEngine(self.scene_dir,settings)
            self.engine.apply_platform(settings)
            key=tuple((k,v) for k,v in asdict(settings).items() if k!='fps')
            reused=request['reuse'] and self.cache_key==key
            if reused:
                images,base=self.cached;results=deepcopy(base)
                for result in results.values():result['platform_settings']=asdict(settings)
            else:
                images={};results={}
                for name in CAMERAS:images[name],results[name]=self.engine.capture_camera(name)
                self.cache_key=key;self.cached=(images,deepcopy(results))
                if not self.renderer_name:
                    from OpenGL import GL
                    self.engine.renderer._gl_context.make_current();self.renderer_name=GL.glGetString(GL.GL_RENDERER).decode()
            view=request['overview'];view_key=(key,tuple(view.values()))
            if self.overview_key!=view_key:
                if self.overview_renderer is None:
                    visual=self.engine.model.vis.global_;old=(visual.offwidth,visual.offheight)
                    try:
                        visual.offwidth,visual.offheight=900,450
                        self.overview_renderer=mujoco.Renderer(self.engine.model,height=450,width=900)
                    finally:visual.offwidth,visual.offheight=old
                camera=mujoco.MjvCamera();camera.type=mujoco.mjtCamera.mjCAMERA_FREE
                camera.lookat[:]=[0,-.7,1.0]
                for name in ['azimuth','elevation','distance']:setattr(camera,name,view[name])
                self.overview_renderer.update_scene(self.engine.data,camera=camera)
                self.overview_renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=0
                self.overview_renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION]=0
                for name in CAMERAS:
                    self.engine.select_camera(name);self.engine.add_overview_markers(self.overview_renderer.scene)
                self.overview_image=self.overview_renderer.render().copy();self.overview_key=view_key
            self.ready.emit(dict(request=request,images=images,results=results,overview=self.overview_image,overview_key=view_key,
                                 cached=reused,renderer=self.renderer_name,seconds=time.perf_counter()-start,
                                 interference=self.engine.interference_report()))
        except Exception as error:self.failed.emit(f'{type(error).__name__}: {error}')
    @QtCore.pyqtSlot()
    def shutdown(self):
        try:
            if self.overview_renderer:self.overview_renderer.close()
            if self.engine:self.engine.close()
        finally:self.stopped.emit()
