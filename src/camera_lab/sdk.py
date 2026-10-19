"""Public in-process SDK. All model access is marshalled to one owner thread."""
import math
from numbers import Integral
from .runtime import Worker
from .modules import ModuleManager


class Scene:
    """Model import and initial-state editing through the shared runtime."""
    def __init__(self, lab):
        self._lab = lab

    def load(self, path, *, packages=None):
        """Replace the scene; enter edit mode. Failure preserves the old scene."""
        return self._model(path, False, (0,0,0), (1,0,0,0), packages)

    def append(self, path, *, position=(0,0,0), rotation=(1,0,0,0), packages=None):
        """Append at a world pose (metres, wxyz); reset to the edited initial state."""
        return self._model(path, True, position, rotation, packages)

    def _model(self, path, append, position, rotation, packages):
        return self._lab._call('model', dict(path=str(path), append=append,
            position=tuple(position), quaternion=tuple(rotation), packages=dict(packages or {})))

    def set_pose(self, name, *, position, rotation=(1,0,0,0)):
        """Edit body initial pose, pausing and resetting the entire simulation."""
        return self._lab._call('edit_body', dict(name=name, position=tuple(position), rotation=tuple(rotation)))

    def describe(self):
        """Return copied body descriptors; no mutable MuJoCo handles escape."""
        return self._lab._call('describe')['bodies']


class Cameras:
    def __init__(self, lab):
        self._lab = lab

    def configure(self, name, **fields):
        """Update calibration and/or parent, position, rotation (optical wxyz).

        Unspecified values are retained. Camera edits preserve simulation mode.
        """
        return self._lab._call('configure_camera', dict(name=name, **fields))

    def aim(self, name, target):
        """Aim at a named body's current world origin."""
        return self._lab._call('aim_camera', dict(name=name, target=target))

    def move(self, name, *, dx=0, dy=0, steps=0):
        """Translate along optical axes: .005 m per dx/dy, .1 m per step."""
        return self._lab._call('move_camera', dict(name=name, dx=dx, dy=dy, steps=steps))

    def describe(self):
        return self._lab._call('describe')['cameras']


class CameraLab:
    """Own a Qt-free simulation runtime; use as a context manager.

    Default mode is paused editing, without continuous rendering. Commands block
    until applied; the bounded command inbox raises queue.Full under overload.
    Rendering still requires a working MuJoCo OpenGL backend.
    """
    def __init__(self, *, width=1280, height=720, fps=30, model=None, packages=None):
        if any(isinstance(v,bool) or not isinstance(v,Integral) or v<=0 for v in (width,height)):
            raise ValueError('width and height must be positive integers')
        if isinstance(fps,bool) or not math.isfinite(fps) or fps<=0:
            raise ValueError('fps must be finite and positive')
        self._worker = Worker(width,height,fps,model,packages,running=False,
                              continuous=False,fallback=False,events=False)
        self.modules = ModuleManager(self._worker.api)
        self.scene = Scene(self)
        self.cameras = Cameras(self)
        self._worker.start()
        self._worker.ready.wait()
        if self._worker.failure is not None:
            self._worker.join()
            raise self._worker.failure

    @property
    def data(self):
        """DataAPI: bounded subscriptions, pose evaluation and control commands."""
        return self._worker.api

    @property
    def mode(self):
        return self._call('mode')

    def _call(self, command, payload=None):
        return self._worker.request(command,payload)

    def run(self):
        """Start realtime physics and observation publication at target fps."""
        return self._call('running',True)

    def pause(self):
        return self._call('running',False)

    def step(self, steps=1):
        """Advance exactly this many timesteps, then stay paused."""
        if isinstance(steps,bool) or not isinstance(steps,Integral) or steps<1:
            raise ValueError('steps must be a positive integer')
        return self._call('step',steps)

    def reset(self):
        return self._call('reset')

    def save_initial(self):
        return self._call('save_initial')

    def capture(self):
        """Capture observations without advancing physics in manual mode."""
        return self._call('capture')[0]

    def capture_with_truth(self):
        """Explicit evaluation access: observations and truth share one stamp."""
        return self._call('capture')

    def save_project(self, path):
        return self._call('save_project',dict(path=str(path),layout={}))

    def load_project(self, path):
        return self._call('open_project',dict(path=str(path)))

    def close(self):
        """Close rendering; return whether all algorithm threads stopped within two seconds."""
        modules_stopped = self.modules.close()
        self._worker.stop.set()
        self._worker.join()
        return modules_stopped

    def __enter__(self):
        return self

    def __exit__(self, *_):
        if not self.close():
            import warnings
            warnings.warn("Algorithm threads still stopping after CameraLab close", RuntimeWarning)
