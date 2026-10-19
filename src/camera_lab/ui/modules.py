"""Optional Qt view of the same module manager used by the Python SDK."""
from queue import Empty
from math import degrees
from PySide6.QtWidgets import QWidget,QVBoxLayout,QComboBox,QPushButton,QLabel


class ModulePanel(QWidget):
    def __init__(self, manager):
        super().__init__();self.manager=manager;self.results=manager.subscribe_results()
        layout=QVBoxLayout(self);self.select=QComboBox();layout.addWidget(self.select)
        self.start_button=QPushButton('Start module');layout.addWidget(self.start_button)
        self.stop_button=QPushButton('Stop module');layout.addWidget(self.stop_button)
        self.status=QLabel();self.result=QLabel('Results refer to the stamped input frame.\nFixed pose demo is not a vision algorithm.')
        self.status.setWordWrap(True);self.result.setWordWrap(True)
        layout.addWidget(self.status);layout.addWidget(self.result)
        self.start_button.clicked.connect(self.start_selected)
        self.stop_button.clicked.connect(self.stop_selected)
        self.poll()

    def start_selected(self):
        try:self.manager.start(self.select.currentText())
        except (RuntimeError,ValueError,KeyError) as exc:self.result.setText(str(exc))

    def stop_selected(self):
        if self.select.currentText():self.manager.stop(self.select.currentText())

    def poll(self):
        names=self.manager.names
        if tuple(self.select.itemText(i) for i in range(self.select.count()))!=names:
            current=self.select.currentText();self.select.clear();self.select.addItems(names)
            if current in names:self.select.setCurrentText(current)
        name=self.select.currentText()
        if name:
            s=self.manager.status(name)
            self.status.setText(f'{name}: {s.state}\nProcessed {s.processed}, dropped {s.dropped}, {s.latency_ms:.1f} ms\n{s.error}')
            self.start_button.setEnabled(s.state in ('stopped','failed'))
            self.stop_button.setEnabled(s.state in ('starting','running'))
        while True:
            try:r=self.results.get_nowait()
            except Empty:break
            lines=[f'{r.module}: input batch {r.stamp.batch_id}',f'run {r.stamp.run_id}, model {r.stamp.model_revision}']
            for error in r.errors:
                detail=(f'translation {error.translation_m*1000:.2f} mm, rotation {degrees(error.rotation_rad):.2f}°'
                        if error.status=='matched' else error.status)
                lines.append(f'{error.camera_id} / {error.object_id}: {detail}')
            self.result.setText('\n'.join(lines) if r.errors else '\n'.join(lines)+'\nNo detections')
