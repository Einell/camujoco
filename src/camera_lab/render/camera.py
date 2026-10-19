"""Adapt pixel-center calibration to MuJoCo 3.6 camera parameters."""
from contextlib import contextmanager
import math
import numpy as np
from ..contracts import CameraConfig


def read_config(model, camera_id, width, height):
    if model.cam_projection[camera_id] != 0:
        raise ValueError('The observation API currently requires perspective cameras')
    sensor = model.cam_sensorsize[camera_id]
    if sensor[0] > 0 and sensor[1] > 0:
        fx,fy,px,py = model.cam_intrinsic[camera_id]
        fx,fy = fx * width / sensor[0], fy * height / sensor[1]
        cx,cy = (width-1)/2 - px*width/sensor[0], (height-1)/2 - py*height/sensor[1]
    else:
        fx = fy = height / (2 * math.tan(math.radians(model.cam_fovy[camera_id])/2))
        cx,cy = (width-1)/2, (height-1)/2
    return CameraConfig(width,height,float(fx),float(fy),float(cx),float(cy),
                        float(model.vis.map.znear*model.stat.extent),
                        float(model.vis.map.zfar*model.stat.extent))


def apply_intrinsics(model, camera_id, config):
    # A 36mm nominal sensor preserves the requested focal/pixel ratios.
    pitch = .036 / config.width
    model.cam_sensorsize[camera_id] = [.036, config.height*pitch]
    model.cam_resolution[camera_id] = [config.width, config.height]
    model.cam_intrinsic[camera_id] = np.array([
        config.fx, config.fy, (config.width-1)/2-config.cx,
        (config.height-1)/2-config.cy]) * pitch


@contextmanager
def clipping(model, config):
    # The classic depth decoder uses the model's global clipping values.
    # Session ownership makes this temporary per-camera change safe.
    near, far = model.vis.map.znear, model.vis.map.zfar
    try:
        model.vis.map.znear = config.near/model.stat.extent
        model.vis.map.zfar = config.far/model.stat.extent
        yield
    finally:
        model.vis.map.znear, model.vis.map.zfar = near, far
