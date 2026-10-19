"""Measured throughput, separate from requested frame rate."""
import importlib.metadata
import os
import platform
import resource
import subprocess
import numpy as np


def summarize(render_ms, elapsed, simulated):
    return {
        'batches': len(render_ms),
        'wall_seconds': elapsed,
        'per_camera_fps': len(render_ms) / elapsed if elapsed else 0,
        'render_p95_ms': float(np.percentile(render_ms, 95)) if render_ms else None,
        'render_mean_ms': float(np.mean(render_ms)) if render_ms else None,
        'simulated_seconds': simulated,
        'real_time_factor': simulated / elapsed if elapsed else 0,
        'peak_rss_mib': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024,
    }


def environment():
    versions = {}
    for name in ('mujoco', 'numpy', 'PySide6-Essentials'):
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = None
    return {'platform': platform.platform(), 'python': platform.python_version(),
            'versions': versions, 'mujoco_gl': os.getenv('MUJOCO_GL', 'glfw'),
            'qt_platform': os.getenv('QT_QPA_PLATFORM', 'desktop')}


def gpu_sample():
    try:
        result = subprocess.run(
            ['nvidia-smi', '--query-gpu=name,driver_version,utilization.gpu,memory.used',
             '--format=csv,noheader,nounits'], capture_output=True, text=True, timeout=3)
        if result.returncode:
            return {'error': result.stderr.strip() or result.stdout.strip()}
        return {'columns': ['name', 'driver', 'utilization_percent', 'memory_mib'],
                'rows': [line.split(', ') for line in result.stdout.strip().splitlines()]}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {'error': str(exc)}


class Measurement:
    """Collect every produced batch, independently of UI frame dropping."""
    def __init__(self):
        from threading import Lock
        self._lock = Lock()
        self._active = False
        self._render_ms = []
        self._simulated = 0.0

    def start(self):
        with self._lock:
            self._render_ms = []
            self._simulated = 0.0
            self._active = True

    def add(self, render_ms, simulated_delta):
        with self._lock:
            if self._active:
                self._render_ms.append(render_ms)
                self._simulated += simulated_delta

    def stop(self):
        with self._lock:
            self._active = False

    def report(self, elapsed, presented):
        with self._lock:
            result = summarize(self._render_ms, elapsed, self._simulated)
        result['produced_per_camera_fps'] = result['per_camera_fps']
        result['presented_batches'] = presented
        result['per_camera_fps'] = presented / elapsed if elapsed else 0
        return result
