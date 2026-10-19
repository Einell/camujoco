"""User workflow across importer, editor, storage, simulation and subscriptions."""
from pathlib import Path
from queue import Empty
import shutil
import time
import numpy as np
from PySide6.QtWidgets import QApplication
from camera_lab.app import Window
from camera_lab.api import DataAPI
from camera_lab.contracts import CameraConfig


def test_import_edit_save_move_reopen_run_subscribe_reset(tmp_path):
    app=QApplication.instance() or QApplication([])
    api=DataAPI();images=api.subscribe_observations();truth=api.subscribe_truth()
    window=Window(width=160,height=90,api=api)
    window.show()
    root=Path(__file__).resolve().parents[1]/'examples/models'
    def wait(predicate):
        end=time.monotonic()+20
        while not predicate() and time.monotonic()<end:
            app.processEvents();time.sleep(.01)
        assert predicate(),window.worker.error
    try:
        wait(lambda:window.last_batch is not None)
        window.request_model(root/'table.xml')
        wait(lambda:window.last_batch.stamp.model_revision==1)
        window.request_model(root/'simple_robot.urdf',append=True,position=(0,0,.75))
        wait(lambda:window.last_batch.stamp.model_revision==2)
        window.edit('edit_body',dict(name='model_0001/base',position=(.1,0,.75),rotation=(1,0,0,0)))
        wait(lambda:window.last_batch.stamp.model_revision==3)
        configs=[CameraConfig(200,120,150,150,99.5,59.5,.01,20),
                 CameraConfig(180,100,140,145,85,49,.02,25),
                 CameraConfig(160,90,120,125,79,44,.03,30)]
        parents=['model_0001/base','world','world']
        names=[f.camera_id for f in window.last_batch.frames]
        for index,(camera,config,parent) in enumerate(zip(names,configs,parents)):
            window.edit('edit_camera',dict(name=camera,parent=parent,position=(index,-2,1),rotation=(1,0,0,0),config=config))
            wait(lambda:window.last_batch.frames[index].config==config)
            revision=window.last_batch.frames[index].camera_revision
            window.edit('aim_camera',dict(name=camera,target='model_0001/base'))
            wait(lambda:window.last_batch.frames[index].camera_revision>revision)
        initial_poses=[f.T_world_camera.copy() for f in window.last_batch.frames]
        save=tmp_path/'first.camproj';window.request_save_project(save)
        wait(lambda:save.exists() and 'Saved project' in window.import_status.text())
        moved=tmp_path/'relocated';moved.mkdir();shutil.move(save,moved/'test.camproj')
        old=window.last_batch.stamp.run_id
        window.request_open_project(moved/'test.camproj')
        wait(lambda:window.last_batch.stamp.run_id!=old)
        assert window.last_batch.sim_time==0
        for frame,config,expected_pose in zip(window.last_batch.frames,configs,initial_poses):
            assert frame.config==config
            assert frame.rgb.shape==(config.height,config.width,3)
            np.testing.assert_allclose(frame.K,config.K)
            np.testing.assert_allclose(frame.T_world_camera,expected_pose,atol=1e-6)
        rows=window.editor.description['cameras']
        assert [row['parent'] for row in rows]==parents
        body=next(row for row in window.editor.description['bodies'] if row['name']=='model_0001/base')
        np.testing.assert_allclose(body['position'],(.1,0,.75),atol=1e-6)
        window.running.setChecked(True)
        wait(lambda:window.last_batch.sim_time>.1)
        window.running.setChecked(False)
        wait(lambda:window.mode.text()=='Mode: paused')
        # Different consumers may retain different batches: match by full stamp.
        image_batches={};truth_batches={}
        end=time.monotonic()+10
        while not set(image_batches)&set(truth_batches) and time.monotonic()<end:
            for sub,records in ((images,image_batches),(truth,truth_batches)):
                try:
                    batch=sub.get_nowait();records[batch.stamp]=batch
                except Empty:pass
            app.processEvents();time.sleep(.01)
        matched=set(image_batches)&set(truth_batches)
        assert matched
        stamp=next(iter(matched))
        assert len(image_batches[stamp].frames)==3
        assert 'model_0001/base' in truth_batches[stamp].object_poses
        old=window.last_batch.stamp.run_id
        window.return_edit.click()
        wait(lambda:window.last_batch.stamp.run_id!=old and window.last_batch.sim_time==0)
        assert window.mode.text()=='Mode: edit'
    finally:
        window.close();wait(lambda:not window.worker.is_alive());app.processEvents()
        assert api.closed
