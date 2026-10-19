"""Desktop rendering baseline; UI owns no MuJoCo/OpenGL state."""
import argparse
import json
from pathlib import Path
from queue import Queue, Empty
from threading import Thread, Event
from time import perf_counter, process_time
import numpy as np
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                               QGridLayout, QLabel, QPushButton, QCheckBox, QHBoxLayout)
from .render.worker import RenderSession, LatestBatch
from .metrics import Measurement, environment


class Worker(Thread):
    def __init__(self, width, height, fps):
        super().__init__(name='mujoco-render', daemon=False)
        self.width, self.height, self.fps = width, height, fps
        self.stop = Event()
        self.commands = Queue(maxsize=64)
        self.mailbox = LatestBatch()
        self.error = None
        self.gl_renderer = 'initializing'
        self.rendered = 0
        self.measurement = Measurement()

    def run(self):
        try:
            with RenderSession(self.width, self.height) as session:
                self.gl_renderer = session.gl_renderer
                running = True
                target_time = 0.0
                previous = perf_counter()
                while not self.stop.is_set():
                    start = perf_counter()
                    delta, previous = min(start - previous, .25), start
                    while True:
                        try:
                            command, payload = self.commands.get_nowait()
                        except Empty:
                            break
                        if command == 'running':
                            running = payload
                            delta = 0
                        elif command == 'reset':
                            session.reset()
                            target_time = 0.0
                            delta = 0
                        elif command == 'orbit':
                            session.orbit(*payload)
                        elif command == 'zoom':
                            session.zoom(payload)
                    before_step = session.data.time
                    if running:
                        target_time += delta
                        session.advance_to(target_time)
                    batch = session.capture()
                    self.measurement.add(batch.render_ms, session.data.time - before_step)
                    self.mailbox.put(batch)
                    self.rendered += 1
                    self.stop.wait(max(0, 1 / self.fps - (perf_counter() - start)))
        except Exception as exc:
            self.error = f'{type(exc).__name__}: {exc}'


class View(QLabel):
    orbit = Signal(float, float)
    zoom = Signal(float)

    def __init__(self, interactive=False):
        super().__init__('Waiting for renderer…')
        self.interactive = interactive
        self.previous = None
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(240, 135)
        self.setStyleSheet('background:#101720; color:#8a9aad; border-radius:6px;')

    def display(self, array):
        pixels = np.ascontiguousarray(array)
        image = QImage(pixels.data, pixels.shape[1], pixels.shape[0], pixels.strides[0],
                       QImage.Format.Format_RGB888).copy()
        self.setPixmap(QPixmap.fromImage(image).scaled(
            self.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.FastTransformation))

    def mousePressEvent(self, event):
        if self.interactive and event.button() == Qt.MouseButton.LeftButton:
            self.previous = event.position()

    def mouseMoveEvent(self, event):
        if self.previous is not None:
            delta = event.position() - self.previous
            self.previous = event.position()
            self.orbit.emit(delta.x(), delta.y())

    def mouseReleaseEvent(self, event):
        self.previous = None

    def wheelEvent(self, event):
        if self.interactive:
            self.zoom.emit(event.angleDelta().y() / 120)


def depth_preview(depth):
    valid = np.isfinite(depth)
    value = np.clip(np.nan_to_num(depth, nan=5) / 5, 0, 1)
    rgb = np.stack((1 - value, 1 - np.abs(value * 2 - 1), value), axis=-1)
    rgb[~valid] = 0
    return (rgb * 255).astype(np.uint8)


class Window(QMainWindow):
    def __init__(self, width=1280, height=720, fps=30, seconds=0, warmup=10,
                 report=None, screenshot=None):
        super().__init__()
        self.setWindowTitle('MuJoCo Camera Lab — Rendering Baseline')
        self.resize(1280, 850)
        self.width_px, self.height_px = width, height
        self.seconds, self.warmup, self.report, self.screenshot = seconds, warmup, report, screenshot
        self.worker = Worker(width, height, fps)
        self.last_batch = None
        self.first_frame_at = None
        self.measure_start = None
        self.cpu_start = None
        self.sim_start = 0
        self.presented = 0
        self.measure_end = None
        self.saved_report = False
        self._closing = False
        root = QWidget()
        layout = QVBoxLayout(root)
        title = QLabel('CAMERA LAB   /   MuJoCo render baseline')
        title.setStyleSheet('font-size:22px; font-weight:600; padding:10px; color:#dce7f4;')
        layout.addWidget(title)
        controls = QHBoxLayout()
        self.running = QPushButton('Running / Pause')
        self.running.setCheckable(True)
        self.running.setChecked(True)
        self.running.toggled.connect(lambda value: self.send('running', value))
        reset = QPushButton('Reset scene')
        reset.clicked.connect(lambda: self.send('reset', None))
        self.depth = QCheckBox('Show depth · 0–5 m')
        for widget in (self.running, reset, self.depth):
            controls.addWidget(widget)
        controls.addStretch()
        controls.addWidget(QLabel(f'3 cameras · {width} × {height} · target {fps:g} FPS'))
        layout.addLayout(controls)
        grid = QGridLayout()
        self.overview = View(interactive=True)
        self.overview.orbit.connect(lambda x, y: self.send('orbit', (x, y)))
        self.overview.zoom.connect(lambda value: self.send('zoom', value))
        self.camera_views = [View() for _ in range(3)]
        for index, (name, view) in enumerate(zip(
                ('GLOBAL · drag to orbit / scroll to zoom', 'CAM 01 · FRONT', 'CAM 02 · SIDE', 'CAM 03 · TOP'),
                (self.overview, *self.camera_views))):
            panel = QWidget()
            panel_layout = QVBoxLayout(panel)
            panel_layout.addWidget(QLabel(name))
            panel_layout.addWidget(view, 1)
            grid.addWidget(panel, index // 2, index % 2)
        layout.addLayout(grid, 1)
        self.status = QLabel('Initializing OpenGL on render worker…')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.setCentralWidget(root)
        self.setStyleSheet('QMainWindow,QWidget {background:#1a2330; color:#dce7f4;} '
                          'QPushButton {padding:8px; background:#34465c; border-radius:4px;} '
                          'QPushButton:checked {background:#216e58;}')
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(5)
        self.worker.start()

    def send(self, name, payload):
        from queue import Full
        try:
            self.worker.commands.put_nowait((name, payload))
        except Full:
            self.status.setText('Input queue full; retry the command.')

    def poll(self):
        if self._closing:
            if not self.worker.is_alive():
                self.close()
            return
        if self.worker.error:
            self.status.setText('Renderer failed: ' + self.worker.error)
            if self.seconds:
                self.close()
            return
        batch = self.worker.mailbox.take()
        if batch is None:
            return
        self.last_batch = batch
        now = perf_counter()
        if self.first_frame_at is None:
            self.first_frame_at = now
        if now - self.first_frame_at >= self.warmup:
            if self.measure_start is None:
                self.measure_start, self.sim_start, self.cpu_start = now, batch.sim_time, process_time()
                self.worker.measurement.start()
            else:
                self.presented += 1
        self.overview.display(batch.overview)
        for view, frame in zip(self.camera_views, batch.frames):
            view.display(depth_preview(frame.depth) if self.depth.isChecked() else frame.rgb)
        elapsed = now - self.measure_start if self.measure_start is not None else 0
        fps = self.presented / elapsed if elapsed else 0
        self.status.setText(f'{self.worker.gl_renderer}  |  sim {batch.sim_time:.3f}s  |  '
                            f'presented {fps:.1f} FPS (after warmup)  |  render {batch.render_ms:.1f}ms  |  '
                            f'UI dropped {self.worker.mailbox.dropped}')
        if self.seconds and elapsed >= self.seconds:
            if self.screenshot:
                Path(self.screenshot).parent.mkdir(parents=True, exist_ok=True)
                self.grab().save(str(self.screenshot))
            self.close()

    def closeEvent(self, event):
        if self.measure_end is None:
            self.measure_end = perf_counter()
            self.worker.measurement.stop()
        self.worker.stop.set()
        if self.worker.is_alive():
            self._closing = True
            event.ignore()
            return
        self.timer.stop()
        if self.report and not self.saved_report:
            elapsed = self.measure_end - self.measure_start if self.measure_start is not None else 0
            result = {**environment(), **self.worker.measurement.report(elapsed, self.presented),
                      'mode': 'qt_desktop', 'desktop_visible': QApplication.platformName() != 'offscreen',
                      'gl_renderer': self.worker.gl_renderer, 'width': self.width_px, 'height': self.height_px,
                      'cameras': 3, 'rgb_and_depth': True, 'target_fps': self.worker.fps,
                      'warmup_seconds': self.warmup, 'ui_dropped_total': self.worker.mailbox.dropped,
                      'rendered_batches_total': self.worker.rendered, 'error': self.worker.error,
                      'process_cpu_percent': 100 * (process_time() - self.cpu_start) / elapsed if elapsed else 0}
            Path(self.report).parent.mkdir(parents=True, exist_ok=True)
            Path(self.report).write_text(json.dumps(result, indent=2) + '\n')
            self.saved_report = True
        event.accept()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--fps', type=float, default=30)
    parser.add_argument('--seconds', type=float, default=0, help='Exit after measured seconds; 0 keeps window open')
    parser.add_argument('--warmup', type=float, default=10)
    parser.add_argument('--report', type=Path)
    parser.add_argument('--screenshot', type=Path)
    args = parser.parse_args()
    if min(args.width, args.height, args.fps) <= 0 or min(args.seconds, args.warmup) < 0:
        parser.error('dimensions/fps must be positive; seconds/warmup must be nonnegative')
    app = QApplication([])
    window = Window(**vars(args))
    window.show()
    app.exec()
    if window.worker.error:
        parser.exit(1, window.worker.error + '\n')


if __name__ == '__main__':
    main()
