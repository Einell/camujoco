"""A render session is created, used and closed on exactly one thread.

The baseline steps physics on this same worker. A later simulation service can
supply snapshots without changing the immutable published image interface.
"""
from dataclasses import dataclass
from importlib.resources import files
from threading import Lock, get_ident
from time import perf_counter
import math
import mujoco
import numpy as np


@dataclass(frozen=True)
class Frame:
    camera: str
    rgb: np.ndarray
    depth: np.ndarray


@dataclass(frozen=True)
class Batch:
    sequence: int
    sim_time: float
    overview: np.ndarray
    frames: tuple[Frame, ...]
    render_ms: float


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
        self.cameras = tuple(cameras or ('front', 'side', 'top')[:camera_count])
        if not 1 <= len(self.cameras) <= 3:
            raise ValueError('Select 1–3 cameras')
        for name in self.cameras:
            if mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, name) < 0:
                raise ValueError(f'Unknown camera: {name}')
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
        # mj_step leaves position-derived fields from the previous integration.
        mujoco.mj_forward(self.model, self.data)

    def reset(self):
        self._check()
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def orbit(self, dx, dy):
        self._check()
        self.view.azimuth += dx * .3
        self.view.elevation = float(np.clip(self.view.elevation + dy * .3, -89, 89))

    def zoom(self, steps):
        self._check()
        self.view.distance = float(np.clip(self.view.distance * math.exp(-steps * .1), .2, 30))

    def capture(self):
        self._check()
        start = perf_counter()
        self.renderer.disable_depth_rendering()
        self.renderer.update_scene(self.data, camera=self.view)
        overview = self.renderer.render()
        overview.setflags(write=False)
        frames = []
        far = self.model.vis.map.zfar * self.model.stat.extent
        for camera in self.cameras:
            self.renderer.disable_depth_rendering()
            self.renderer.update_scene(self.data, camera=camera)
            rgb = self.renderer.render()
            self.renderer.enable_depth_rendering()
            depth = self.renderer.render()
            depth[depth >= far * .999] = np.nan
            rgb.setflags(write=False)
            depth.setflags(write=False)
            frames.append(Frame(camera, rgb, depth))
        result = Batch(self.sequence, float(self.data.time), overview, tuple(frames),
                       (perf_counter() - start) * 1000)
        self.sequence += 1
        return result

    def close(self):
        if not self.closed:
            self._check()
            self.renderer.close()
            self.closed = True

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
