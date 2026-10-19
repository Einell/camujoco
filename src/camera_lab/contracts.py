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
