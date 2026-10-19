"""T_A_B maps coordinates in B into A; camera axes: right, down, forward."""
import numpy as np
from .contracts import readonly


def world_pose(position, rotation):
    transform = np.eye(4)
    transform[:3,:3] = np.asarray(rotation).reshape(3,3)
    transform[:3,3] = position
    return readonly(transform)


def camera_world_pose(position, mujoco_rotation):
    rotation = np.asarray(mujoco_rotation).reshape(3,3) @ np.diag([1,-1,-1])
    return world_pose(position, rotation)
