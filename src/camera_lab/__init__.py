"""MuJoCo camera laboratory: Qt-free Python SDK with an optional desktop."""
from typing import TYPE_CHECKING
from .contracts import CameraConfig, ControlCommand, PoseEstimate

if TYPE_CHECKING:
    from .sdk import CameraLab

__all__ = ['CameraLab', 'CameraConfig', 'ControlCommand', 'PoseEstimate']


def __getattr__(name):
    # Contract-only consumers should not select/initialize a rendering backend.
    if name == 'CameraLab':
        from .sdk import CameraLab
        globals()[name] = CameraLab
        return CameraLab
    raise AttributeError(f'module {__name__!r} has no attribute {name!r}')
