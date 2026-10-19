import time
from PySide6.QtWidgets import QApplication
from camera_lab.app import Window


def test_window_receives_three_views_and_closes_worker():
    app = QApplication.instance() or QApplication([])
    window = Window(width=160, height=90)
    window.show()
    until = time.monotonic() + 15
    while window.last_batch is None and time.monotonic() < until:
        app.processEvents()
        time.sleep(.02)
    try:
        assert window.worker.error is None
        assert window.last_batch is not None
        assert len(window.camera_views) == 3
        assert all(not view.pixmap().isNull() for view in window.camera_views)
        window.running.setChecked(False)
        window.depth.setChecked(True)
        for _ in range(20):
            app.processEvents(); time.sleep(.01)
    finally:
        window.close()
        for _ in range(300):
            app.processEvents()
            if not window.worker.is_alive():
                break
            time.sleep(.01)
        assert not window.worker.is_alive()
