"""A render session is created, used and closed on exactly one thread.

The baseline steps physics on this same worker. A later simulation service can
supply snapshots without changing the immutable published image interface.
"""
from importlib.resources import files
from threading import Lock, get_ident
from time import perf_counter, monotonic_ns
from types import MappingProxyType
from uuid import uuid4
import math
import mujoco
import numpy as np


from ..contracts import CameraConfig, CameraFrame, FrameStamp, ObservationBatch, GroundTruthBatch
from ..coordinates import camera_world_pose, world_pose
from .camera import read_config, apply_intrinsics, clipping


class LatestBatch:
    """One-slot mailbox: a slow UI never queues unlimited full-resolution frames."""
    def __init__(self):
        self._lock = Lock()
        self._value = None
        self.dropped = 0

    def put(self, value):
        with self._lock:
            if self._value is not None:
                self.dropped += 1
            self._value = value

    def take(self):
        with self._lock:
            value, self._value = self._value, None
            return value


class RenderSession:
    def __init__(self, width=1280, height=720, camera_count=3, xml=None, cameras=None):
        if width <= 0 or height <= 0 or camera_count not in (1, 2, 3):
            raise ValueError('Positive dimensions and 1–3 cameras are required')
        self._owner = get_ident()
        self.model = mujoco.MjModel.from_xml_string(
            xml if xml is not None else files('camera_lab').joinpath('demo.xml').read_text())
        self.model.vis.global_.offwidth = max(width, self.model.vis.global_.offwidth)
        self.model.vis.global_.offheight = max(height, self.model.vis.global_.offheight)
        self.cameras = tuple(('front', 'side', 'top')[:camera_count] if cameras is None else cameras)
        if not 1 <= len(self.cameras) <= 3 or len(set(self.cameras)) != len(self.cameras):
            raise ValueError('Select 1–3 distinct cameras')
        for name in self.cameras:
            if mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, name) < 0:
                raise ValueError(f'Unknown camera: {name}')
        self._camera_ids = {name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, name)
                            for name in self.cameras}
        self._configs = {name: read_config(self.model, cid, width, height)
                         for name, cid in self._camera_ids.items()}
        self._revisions = {name: 0 for name in self.cameras}
        for name, cid in self._camera_ids.items():
            apply_intrinsics(self.model, cid, self._configs[name])
        names = {body: mujoco.mj_id2name(self.model,mujoco.mjtObj.mjOBJ_BODY,body)
                 for body in range(1,self.model.nbody)}
        used_names = {name for name in names.values() if name is not None}
        for body, name in names.items():
            if name is None:
                candidate = f'body:{body}'
                while candidate in used_names:
                    candidate += ':'
                names[body] = candidate
                used_names.add(candidate)
        self._body_names = names
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.view = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(self.view)
        self.view.lookat[:] = [0, 0, .8]
        self.view.distance = 4
        self.view.azimuth = 125
        self.view.elevation = -25
        self.renderer = mujoco.Renderer(self.model, height=height, width=width)
        from OpenGL import GL
        self.gl_renderer = (GL.glGetString(GL.GL_RENDERER) or b'unknown').decode()
        self._renderers = {(width,height): self.renderer}
        self._overview_size = (width,height)
        self.run_id = str(uuid4())
        self.model_revision = 0
        self.step_index = 0
        self.sequence = 0
        self.closed = False

    def _check(self):
        if get_ident() != self._owner:
            raise RuntimeError('RenderSession must remain on its creating thread')
        if self.closed:
            raise RuntimeError('RenderSession is closed')

    def advance_to(self, sim_time):
        self._check()
        while self.data.time + 1e-12 < sim_time:
            if self.model.nu:
                self.data.ctrl[0] = .6 * math.sin(2 * math.pi * .6 * self.data.time)
            mujoco.mj_step(self.model, self.data)
            self.step_index += 1
        # mj_step leaves position-derived fields from the previous integration.
        mujoco.mj_forward(self.model, self.data)

    def reset(self):
        self._check()
        self.run_id = str(uuid4())
        self.step_index = 0
        self.sequence = 0
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def orbit(self, dx, dy):
        self._check()
        self.view.azimuth += dx * .3
        self.view.elevation = float(np.clip(self.view.elevation + dy * .3, -89, 89))

    def zoom(self, steps):
        self._check()
        self.view.distance = float(np.clip(self.view.distance * math.exp(-steps * .1), .2, 30))

    def camera_config(self, camera):
        self._check()
        return self._configs[camera]

    def configure_camera(self, camera, config: CameraConfig):
        self._check()
        if camera not in self._camera_ids:
            raise ValueError(f'Unknown camera: {camera}')
        if not isinstance(config, CameraConfig):
            raise TypeError('Expected CameraConfig')
        size = (config.width, config.height)
        if size not in self._renderers:
            old_width, old_height = self.model.vis.global_.offwidth, self.model.vis.global_.offheight
            try:
                self.model.vis.global_.offwidth = max(config.width, old_width)
                self.model.vis.global_.offheight = max(config.height, old_height)
                renderer = mujoco.Renderer(self.model, height=config.height, width=config.width)
            except Exception:
                self.model.vis.global_.offwidth, self.model.vis.global_.offheight = old_width, old_height
                raise
            self._renderers[size] = renderer
        apply_intrinsics(self.model, self._camera_ids[camera], config)
        self._configs[camera] = config
        self._revisions[camera] += 1
        used = {self._overview_size} | {(c.width,c.height) for c in self._configs.values()}
        for key in list(self._renderers):
            if key not in used:
                self._renderers.pop(key).close()

    def capture(self):
        return self.capture_sample()[0]

    def capture_sample(self):
        self._check()
        start = perf_counter()
        stamp = FrameStamp(self.run_id, self.model_revision, self.sequence,
                           self.step_index, float(self.data.time), monotonic_ns())
        self.renderer.disable_depth_rendering()
        self.renderer.update_scene(self.data, camera=self.view)
        overview = self.renderer.render()
        overview.setflags(write=False)
        frames = []
        for camera in self.cameras:
            config = self._configs[camera]
            cid = self._camera_ids[camera]
            renderer = self._renderers[(config.width,config.height)]
            with clipping(self.model, config):
                renderer.disable_depth_rendering()
                renderer.update_scene(self.data, camera=camera)
                rgb = renderer.render()
                renderer.enable_depth_rendering()
                depth = renderer.render()
            valid = np.isfinite(depth) & (depth >= config.near) & (depth < config.far*(1-1e-6))
            depth[~valid] = np.nan
            for value in (rgb,depth,valid):
                value.setflags(write=False)
            frames.append(CameraFrame(stamp, camera, self._revisions[camera], config,
                          rgb, depth, valid, config.K,
                          camera_world_pose(self.data.cam_xpos[cid], self.data.cam_xmat[cid])))
        poses = {}
        for body, name in self._body_names.items():
            poses[name] = world_pose(self.data.xpos[body],self.data.xmat[body])
        truth = GroundTruthBatch(stamp, MappingProxyType(poses))
        result = ObservationBatch(stamp, overview, tuple(frames), (perf_counter()-start)*1000)
        self.sequence += 1
        return result, truth

    def close(self):
        if not self.closed:
            self._check()
            for renderer in self._renderers.values():
                renderer.close()
            self.closed = True

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
