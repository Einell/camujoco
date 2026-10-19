"""Desktop rendering baseline; UI owns no MuJoCo/OpenGL state."""
import argparse
import json
from pathlib import Path
from queue import Empty
from time import perf_counter, process_time
import numpy as np
from PySide6.QtCore import Qt, QTimer, Signal, QByteArray
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                               QGridLayout, QLabel, QPushButton, QCheckBox, QHBoxLayout,
                               QFileDialog, QInputDialog, QDialog, QDockWidget)
from .metrics import environment
from .ui.import_dialog import PlacementDialog
from .ui.editor import Editor
from .ui.preview import PreviewPanel
from .api.streams import DataAPI


from .runtime import Worker


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
                 report=None, screenshot=None, model=None, packages=None, api=None):
        super().__init__()
        self.setWindowTitle('MuJoCo Camera Lab')
        self.resize(1280, 850)
        self.width_px, self.height_px = width, height
        self.seconds, self.warmup, self.report, self.screenshot = seconds, warmup, report, screenshot
        self.packages = dict(packages or {})
        self.worker = Worker(width,height,fps,model,self.packages,api)
        self.api=self.worker.api
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
        title = QLabel('CAMERA LAB   /   MuJoCo scenes and cameras')
        title.setStyleSheet('font-size:22px; font-weight:600; padding:10px; color:#dce7f4;')
        layout.addWidget(title)
        file_controls = QHBoxLayout()
        self.open_model = QPushButton('Open MJCF / URDF')
        self.append_model = QPushButton('Append model')
        self.map_package = QPushButton('Map package')
        self.open_model.clicked.connect(lambda: self.choose_model(False))
        self.append_model.clicked.connect(lambda: self.choose_model(True))
        self.map_package.clicked.connect(self.choose_package)
        for button in (self.open_model,self.append_model,self.map_package):
            file_controls.addWidget(button)
            button.setEnabled(not bool(seconds))
        self.save_project_button=QPushButton('Save project')
        self.open_project_button=QPushButton('Open project')
        self.save_project_button.clicked.connect(self.choose_save_project)
        self.open_project_button.clicked.connect(self.choose_open_project)
        file_controls.addWidget(self.save_project_button)
        file_controls.addWidget(self.open_project_button)
        self.save_project_button.setEnabled(not bool(seconds))
        self.open_project_button.setEnabled(not bool(seconds))
        file_controls.addStretch()
        layout.addLayout(file_controls)
        self.import_status = QLabel('Open replaces the scene; append adds a model at a specified pose. Imports pause simulation.')
        self.import_status.setWordWrap(True)
        layout.addWidget(self.import_status)
        controls = QHBoxLayout()
        self.running = QPushButton('Running / Pause')
        self.running.setCheckable(True)
        self.running.setChecked(model is None)
        self.running.toggled.connect(lambda value: self.send('running', value))
        reset = QPushButton('Reset initial')
        reset.clicked.connect(lambda: self.send('reset', None))
        self.step=QPushButton('Single step')
        self.return_edit=QPushButton('Return to edit')
        self.save_initial=QPushButton('Use pose as initial')
        self.mode=QLabel('Mode: initializing')
        for button,command in ((self.step,'step'),(self.return_edit,'edit'),(self.save_initial,'save_initial')):
            button.clicked.connect(lambda checked=False,c=command:self.send(c,None))
        self.depth = QCheckBox('Show depth · 0–5 m')
        frustums=QCheckBox('Camera frustums')
        self.frustums=frustums
        frustums.toggled.connect(lambda value:self.send('frustums',value))
        for widget in (self.running, reset, self.step, self.return_edit, self.save_initial):
            controls.addWidget(widget)
        controls.addStretch()
        controls.addWidget(self.mode)
        layout.addLayout(controls)
        display_controls=QHBoxLayout()
        display_controls.addWidget(self.depth)
        display_controls.addWidget(frustums)
        display_controls.addStretch()
        display_controls.addWidget(QLabel(f'3 cameras · {width} × {height} · target {fps:g} FPS'))
        layout.addLayout(display_controls)
        grid = QGridLayout()
        self.overview = View(interactive=True)
        self.overview.orbit.connect(lambda x, y: self.send('orbit', (x, y)))
        self.overview.zoom.connect(lambda value: self.send('zoom', value))
        self.camera_views = [View(interactive=True) for _ in range(3)]
        for index,view in enumerate(self.camera_views):
            view.orbit.connect(lambda x,y,i=index:self.drag_camera(i,x,y))
            view.zoom.connect(lambda value,i=index:self.drag_camera(i,0,0,value))
        self.camera_labels = []
        self.camera_panels = []
        self.preview_panels = []
        for index, (name, view) in enumerate(zip(
                ('GLOBAL · drag to orbit / scroll to zoom', 'CAM 01', 'CAM 02', 'CAM 03'),
                (self.overview, *self.camera_views))):
            label=QLabel(name)
            panel=PreviewPanel(label,view)
            self.preview_panels.append(panel)
            if index:
                self.camera_labels.append(label)
                self.camera_panels.append(panel)
            grid.addWidget(panel,index//2,index%2)
        layout.addLayout(grid, 1)
        self.status = QLabel('Initializing OpenGL on render worker…')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.setCentralWidget(root)
        self.editor=Editor()
        self.editor.command.connect(self.edit)
        dock=QDockWidget('Scene properties',self)
        dock.setObjectName('scene-properties')
        dock.setWidget(self.editor)
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea,dock)
        dock.setMinimumWidth(270)
        self.resize(1550,950)
        self.setStyleSheet('QMainWindow,QWidget {background:#1a2330; color:#dce7f4;} '
                          'QPushButton {padding:8px; background:#34465c; border-radius:4px;} '
                          'QPushButton:checked {background:#216e58;}')
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(5)
        self.worker.start()

    def project_layout(self):
        return dict(depth=self.depth.isChecked(),geometry=bytes(self.saveGeometry().toBase64()).decode(),
                    docks=bytes(self.saveState().toBase64()).decode(),
                    floating=[bytes(p.dialog.saveGeometry().toBase64()).decode() if p.dialog else None
                              for p in self.preview_panels])

    def apply_layout(self,layout):
        self.depth.setChecked(bool(layout.get('depth',False)))
        if layout.get('geometry'):self.restoreGeometry(QByteArray.fromBase64(layout['geometry'].encode()))
        if layout.get('docks'):self.restoreState(QByteArray.fromBase64(layout['docks'].encode()))
        for panel,geometry in zip(self.preview_panels,layout.get('floating',[None]*4)):
            if panel.dialog:panel.dialog.close()
            if geometry:
                panel.detach()
                panel.dialog.restoreGeometry(QByteArray.fromBase64(geometry.encode()))

    def choose_save_project(self):
        path,_=QFileDialog.getSaveFileName(self,'Save project','','Camera Lab project (*.camproj)')
        if path:self.request_save_project(path if path.endswith('.camproj') else path+'.camproj')

    def choose_open_project(self):
        path,_=QFileDialog.getOpenFileName(self,'Open project','','Camera Lab project (*.camproj)')
        if path:self.request_open_project(path)

    def request_save_project(self,path):
        self.send('save_project',dict(path=str(path),layout=self.project_layout()))

    def request_open_project(self,path):
        if self.send('open_project',dict(path=str(path))):
            self.import_status.setText('Opening project '+str(path))

    def edit(self,command,payload):
        if command == 'edit_body':
            self.running.setChecked(False)
        self.send(command,payload)

    def drag_camera(self,index,dx,dy,steps=0):
        if self.last_batch:
            self.edit('move_camera',dict(name=self.last_batch.frames[index].camera_id,dx=dx,dy=dy,steps=steps))

    def choose_model(self, append):
        path,_ = QFileDialog.getOpenFileName(self,'Select MJCF or URDF','','Models (*.xml *.urdf);;All files (*)')
        if not path:
            return
        position,quaternion = (0,0,0),(1,0,0,0)
        if append:
            dialog = PlacementDialog(self)
            if dialog.exec() != QDialog.DialogCode.Accepted:
                return
            position,quaternion = dialog.placement()
        self.request_model(path,append,position,quaternion)

    def choose_package(self):
        folder = QFileDialog.getExistingDirectory(self,'Select package root directory')
        if not folder:
            return
        name,ok = QInputDialog.getText(self,'Package name','Name in package://NAME/...',text=Path(folder).name)
        if ok and name.strip():
            self.packages[name.strip()] = folder
            self.import_status.setText(f'Package {name.strip()} → {folder}')

    def request_model(self,path,append=False,position=(0,0,0),quaternion=(1,0,0,0)):
        self.running.setChecked(False)
        payload = dict(path=str(path),append=append,position=position,quaternion=quaternion,
                       packages=dict(self.packages))
        if self.send('model',payload):
            self.import_status.setText(f'Loading {path}…')
            self.open_model.setEnabled(False)
            self.append_model.setEnabled(False)

    def send(self, name, payload):
        from queue import Full
        try:
            self.worker.commands.put_nowait((name, payload))
            return True
        except Full:
            self.status.setText('Input queue full; retry the command.')
            return False

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
        while True:
            try:
                notice = self.worker.events.get_nowait()
            except Empty:
                break
            if 'storage_message' in notice:
                self.import_status.setText(notice['storage_message'])
                continue
            if 'layout' in notice:
                self.apply_layout(notice['layout'])
                self.frustums.blockSignals(True)
                self.frustums.setChecked(notice['frustums'])
                self.frustums.blockSignals(False)
                continue
            if 'mode' in notice:
                self.mode.setText('Mode: '+notice['mode'])
                self.running.blockSignals(True)
                self.running.setChecked(notice['mode']=='running')
                self.running.blockSignals(False)
                continue
            if 'scene' in notice:
                self.editor.update_scene(notice['scene'])
                if 'edit_message' in notice:self.import_status.setText(notice['edit_message'])
                continue
            self.import_status.setText(notice['message'])
            self.open_model.setEnabled(not bool(self.seconds))
            self.append_model.setEnabled(not bool(self.seconds))
            if notice['ok']:
                self.worker.measurement.stop()
                self.measure_start = None
                self.first_frame_at = None
                self.presented = 0
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
        for label, frame in zip(self.camera_labels,batch.frames):
            label.setText(frame.camera_id)
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
        for panel in self.preview_panels:
            if panel.dialog is not None:panel.dialog.close()
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
    parser.add_argument('--model', type=Path)
    parser.add_argument('--package', action='append',default=[],metavar='NAME=PATH')
    args = parser.parse_args()
    if min(args.width, args.height, args.fps) <= 0 or min(args.seconds, args.warmup) < 0:
        parser.error('dimensions/fps must be positive; seconds/warmup must be nonnegative')
    packages = {}
    for mapping in args.package:
        name,separator,path = mapping.partition('=')
        if not separator or not name or not path:
            parser.error('--package must use NAME=PATH')
        packages[name] = path
    del args.package
    args.packages = packages
    app = QApplication([])
    window = Window(**vars(args))
    window.show()
    app.exec()
    if window.worker.error:
        parser.exit(1, window.worker.error + '\n')


if __name__ == '__main__':
    main()
