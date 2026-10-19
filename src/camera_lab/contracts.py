"""Versioned, read-only camera observations. All distances are meters."""
from dataclasses import dataclass
from numbers import Integral
from typing import Mapping
import math
import numpy as np


def readonly(array):
    """Publish an owned array without exposing a mutable simulation buffer."""
    value = np.array(array, copy=True)
    value.setflags(write=False)
    return value


@dataclass(frozen=True)
class CameraConfig:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    near: float
    far: float

    def __post_init__(self):
        if any(isinstance(v, bool) or not isinstance(v, Integral) or v <= 0
               for v in (self.width, self.height)):
            raise ValueError('Camera dimensions must be positive integers')
        if not all(math.isfinite(v) for v in (self.fx,self.fy,self.cx,self.cy,self.near,self.far)):
            raise ValueError('Camera calibration must be finite')
        if self.fx <= 0 or self.fy <= 0 or not 0 < self.near < self.far:
            raise ValueError('Require positive focal lengths and 0 < near < far')

    @property
    def K(self):
        return readonly([[self.fx,0,self.cx],[0,self.fy,self.cy],[0,0,1]])


@dataclass(frozen=True)
class FrameStamp:
    run_id: str
    model_revision: int
    batch_id: int
    step_index: int
    sim_time_s: float
    host_monotonic_ns: int


@dataclass(frozen=True)
class CameraFrame:
    stamp: FrameStamp
    camera_id: str
    camera_revision: int
    config: CameraConfig
    rgb: np.ndarray
    depth: np.ndarray
    valid_mask: np.ndarray
    K: np.ndarray
    T_world_camera: np.ndarray

    @property
    def camera(self):
        return self.camera_id


@dataclass(frozen=True)
class ObservationBatch:
    stamp: FrameStamp
    overview: np.ndarray
    frames: tuple[CameraFrame, ...]
    render_ms: float

    @property
    def sequence(self):
        return self.stamp.batch_id

    @property
    def sim_time(self):
        return self.stamp.sim_time_s


@dataclass(frozen=True)
class GroundTruthBatch:
    stamp: FrameStamp
    object_poses: Mapping[str, np.ndarray]


@dataclass(frozen=True)
class PoseEstimate:
    stamp: FrameStamp
    camera_id: str
    object_id: str
    T_camera_object: np.ndarray
    confidence: float=1.0
    latency_ms: float=0.0

    def __post_init__(self):
        t=np.asarray(self.T_camera_object,dtype=float)
        if (t.shape!=(4,4) or not np.isfinite(t).all() or
            not np.allclose(t[3],[0,0,0,1],atol=1e-8) or
            not np.allclose(t[:3,:3].T@t[:3,:3],np.eye(3),atol=1e-6) or
            not np.isclose(np.linalg.det(t[:3,:3]),1,atol=1e-6)):
            raise ValueError('Pose estimate must be a finite rigid transform')
        if not math.isfinite(self.confidence) or not 0<=self.confidence<=1 or not math.isfinite(self.latency_ms) or self.latency_ms<0:
            raise ValueError('Invalid confidence or latency')
        object.__setattr__(self,'T_camera_object',readonly(t))


@dataclass(frozen=True)
class PoseError:
    stamp: FrameStamp
    camera_id: str
    object_id: str
    status: str
    translation_m: float | None=None
    rotation_rad: float | None=None


@dataclass(frozen=True)
class ControlCommand:
    run_id: str
    model_revision: int
    target_step: int
    valid_until_step: int
    expires_at_ns: int
    values: Mapping[str,float]
    semantics: str='native_ctrl'

    def __post_init__(self):
        from types import MappingProxyType
        if any(isinstance(v,bool) or not isinstance(v,Integral) or v<0
               for v in (self.model_revision,self.target_step,self.valid_until_step,self.expires_at_ns)):
            raise ValueError('Command versions, steps and deadline must be nonnegative integers')
        if self.valid_until_step<self.target_step or not self.values:
            raise ValueError('Command needs values and a valid step interval')
        if self.semantics!='native_ctrl':raise ValueError('Only native_ctrl semantics are supported')
        if any(not isinstance(k,str) or not math.isfinite(v) for k,v in self.values.items()):
            raise ValueError('Actuator values must be named and finite')
        object.__setattr__(self,'values',MappingProxyType(dict(self.values)))


@dataclass(frozen=True)
class ControlResult:
    command: ControlCommand
    accepted: bool
    message: str


@dataclass(frozen=True)
class ActuatorInfo:
    name: str | None
    ctrlrange: tuple[float,float] | None
    default: float
    semantics: str='native_ctrl'
