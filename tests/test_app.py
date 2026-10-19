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


def test_window_load_append_and_failed_import_keep_live_scene(tmp_path):
    from camera_lab.project.importer import SceneProject
    source=tmp_path/'box.xml'
    source.write_text('<mujoco><worldbody><body name="box"><geom type="box" size=".1 .1 .1"/></body></worldbody></mujoco>')
    app=QApplication.instance() or QApplication([])
    window=Window(width=160,height=90)
    window.show()
    def wait_until(predicate):
        until=time.monotonic()+15
        while not predicate() and time.monotonic()<until:
            app.processEvents();time.sleep(.01)
        assert predicate(),window.worker.error
    try:
        wait_until(lambda:window.last_batch is not None)
        old_run=window.last_batch.stamp.run_id
        window.request_model(source)
        wait_until(lambda:window.last_batch.stamp.model_revision==1)
        assert window.last_batch.stamp.run_id!=old_run
        assert not window.running.isChecked()
        window.request_model(source,append=True,position=(1,0,0))
        wait_until(lambda:window.last_batch.stamp.model_revision==2)
        stable_run=window.last_batch.stamp.run_id
        window.request_model(tmp_path/'missing.xml')
        wait_until(lambda:window.open_model.isEnabled() and 'missing.xml' in window.import_status.text())
        assert window.worker.error is None
        assert window.last_batch.stamp.run_id==stable_run
        window.running.setChecked(True)
        wait_until(lambda:window.last_batch.sim_time>.05)
    finally:
        window.close()
        wait_until(lambda:not window.worker.is_alive())
        app.processEvents()


def test_initial_import_failure_can_retry(tmp_path):
    app=QApplication.instance() or QApplication([])
    window=Window(width=160,height=90,model=tmp_path/'missing.xml')
    def wait_until(predicate):
        until=time.monotonic()+15
        while not predicate() and time.monotonic()<until:
            app.processEvents();time.sleep(.01)
        assert predicate(),window.worker.error
    try:
        wait_until(lambda:window.last_batch is not None or window.worker.error is not None)
        assert window.worker.error is None
        assert window.worker.is_alive()
        assert 'missing.xml' in window.import_status.text()
        source=tmp_path/'valid.xml'
        source.write_text('<mujoco><worldbody><geom type="sphere" size=".1"/></worldbody></mujoco>')
        window.request_model(source)
        wait_until(lambda:window.last_batch.stamp.model_revision==1)
        assert window.open_model.isEnabled()
    finally:
        window.close()
        wait_until(lambda:not window.worker.is_alive())
        app.processEvents()


def test_editor_changes_camera_and_detaches_preview():
    app=QApplication.instance() or QApplication([])
    window=Window(width=160,height=90)
    window.show()
    def wait_until(predicate):
        until=time.monotonic()+15
        while not predicate() and time.monotonic()<until:
            app.processEvents();time.sleep(.01)
        assert predicate(),window.worker.error
    try:
        wait_until(lambda:window.last_batch is not None and window.editor.description is not None)
        item=window.editor.tree.topLevelItem(1).child(0)
        window.editor.tree.setCurrentItem(item)
        window.editor.values['width'].setValue(200)
        window.editor.values['height'].setValue(100)
        window.editor.apply.click()
        wait_until(lambda:window.last_batch.frames[0].rgb.shape==(100,200,3))
        assert not window.running.isChecked()
        panel=window.camera_panels[0]
        panel.detach()
        assert panel.dialog.isVisible()
        sequence=window.last_batch.sequence
        wait_until(lambda:window.last_batch.sequence>sequence)
        panel.dialog.close()
        assert window.camera_views[0].parent() is panel
        panel.detach()
        floating=panel.dialog
    finally:
        window.close()
        wait_until(lambda:not window.worker.is_alive())
        app.processEvents()
    from shiboken6 import isValid
    assert not isValid(floating) or not floating.isVisible()


def test_simulation_buttons_step_save_and_return_to_edit():
    app=QApplication.instance() or QApplication([])
    window=Window(width=160,height=90)
    window.show()
    def wait_until(predicate):
        until=time.monotonic()+15
        while not predicate() and time.monotonic()<until:
            app.processEvents();time.sleep(.01)
        assert predicate(),window.worker.error
    try:
        wait_until(lambda:window.last_batch is not None)
        window.return_edit.click()
        wait_until(lambda:window.last_batch.sim_time==0 and window.mode.text()=='Mode: edit')
        run=window.last_batch.stamp.run_id
        window.step.click()
        wait_until(lambda:window.last_batch.stamp.step_index==1)
        assert not window.running.isChecked()
        window.save_initial.click()
        wait_until(lambda:window.last_batch.stamp.run_id!=run and window.last_batch.sim_time==0)
        window.running.setChecked(True)
        wait_until(lambda:window.last_batch.sim_time>.05)
        window.return_edit.click()
        wait_until(lambda:window.last_batch.sim_time==0 and window.mode.text()=='Mode: edit')
    finally:
        window.close()
        wait_until(lambda:not window.worker.is_alive())
        app.processEvents()


def test_save_and_open_project_restores_ui_and_paused_scene(tmp_path):
    app=QApplication.instance() or QApplication([])
    window=Window(width=160,height=90)
    window.show()
    def wait_until(predicate):
        until=time.monotonic()+15
        while not predicate() and time.monotonic()<until:
            app.processEvents();time.sleep(.01)
        assert predicate(),window.worker.error
    path=tmp_path/'desktop.camproj'
    try:
        wait_until(lambda:window.last_batch is not None)
        window.depth.setChecked(True)
        window.request_save_project(path)
        wait_until(lambda:path.exists() and 'Saved project' in window.import_status.text())
        window.depth.setChecked(False)
        run=window.last_batch.stamp.run_id
        window.request_open_project(path)
        wait_until(lambda:window.last_batch.stamp.run_id!=run)
        assert window.depth.isChecked()
        assert window.last_batch.sim_time==0
        assert window.mode.text()=='Mode: edit'
        stable=window.last_batch.stamp.run_id
        window.request_open_project(tmp_path/'missing.camproj')
        wait_until(lambda:'missing.camproj' in window.import_status.text() and 'Opening' not in window.import_status.text())
        assert window.last_batch.stamp.run_id==stable
        assert window.worker.error is None
    finally:
        window.close()
        wait_until(lambda:not window.worker.is_alive())
        app.processEvents()


def test_window_publishes_streams_and_accepts_control():
    from camera_lab.api.streams import DataAPI
    from camera_lab.contracts import ControlCommand
    from queue import Empty
    api=DataAPI();images=api.subscribe_observations();truth=api.subscribe_truth();results=api.subscribe_control_results()
    app=QApplication.instance() or QApplication([])
    window=Window(width=160,height=90,api=api)
    window.show()
    def wait_until(predicate):
        until=time.monotonic()+15
        while not predicate() and time.monotonic()<until:
            app.processEvents();time.sleep(.01)
        assert predicate(),window.worker.error
    try:
        window.return_edit.click()
        wait_until(lambda:window.last_batch is not None and window.mode.text()=='Mode: edit')
        stamp=window.last_batch.stamp
        api.submit_control(ControlCommand(stamp.run_id,stamp.model_revision,0,100,time.monotonic_ns()+2_000_000_000,{'swing_control':.2}))
        received=[]
        def result_ready():
            try:received.append(results.get_nowait())
            except Empty:pass
            return bool(received)
        wait_until(result_ready)
        assert received[0].accepted
        window.step.click()
        wait_until(lambda:window.last_batch.stamp.step_index==1)
        assert images.get_nowait().stamp.run_id==stamp.run_id
        assert truth.get_nowait().stamp.run_id==stamp.run_id
    finally:
        window.close();wait_until(lambda:not window.worker.is_alive());app.processEvents()
        assert api.closed and images.closed and truth.closed


def test_control_submitted_before_single_step_is_not_skipped():
    from pathlib import Path
    from queue import Empty
    from camera_lab.app import Worker
    from camera_lab.api import DataAPI
    from camera_lab.contracts import ControlCommand
    api=DataAPI();images=api.subscribe_observations();results=api.subscribe_control_results()
    source=Path(__file__).resolve().parents[1]/'src/camera_lab/demo.xml'
    worker=Worker(80,60,2,model=source,api=api);worker.start()
    def take(sub):
        end=time.monotonic()+15
        while time.monotonic()<end:
            try:return sub.get_nowait()
            except Empty:time.sleep(.005)
        raise AssertionError(worker.error)
    try:
        stamp=take(images).stamp
        api.submit_control(ControlCommand(stamp.run_id,stamp.model_revision,0,3,time.monotonic_ns()+5_000_000_000,{'swing_control':.2}))
        worker.commands.put(('step',None))
        assert take(results).accepted
        for _ in range(10):
            frame=take(images)
            assert frame.stamp.run_id==stamp.run_id
            assert frame.stamp.step_index<=1
            if frame.stamp.step_index==1:break
        else:raise AssertionError('Single-step frame was not published')
    finally:
        worker.stop.set();worker.join(15)
        assert not worker.is_alive()
