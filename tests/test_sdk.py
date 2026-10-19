import os
import subprocess
import sys
import threading
import time
from queue import Empty
from pathlib import Path
import numpy as np
import pytest
from camera_lab import CameraLab


def test_core_import_without_qt():
    script='''
import sys
class NoQt:
    def find_spec(self, fullname, *args):
        if fullname.startswith('PySide6'):
            raise AssertionError('SDK imports Qt')
sys.meta_path.insert(0, NoQt())
from camera_lab import CameraLab
with CameraLab(width=80,height=60) as lab:
    assert lab.capture().frames[0].rgb.shape == (60,80,3)
'''
    subprocess.run([sys.executable,'-c',script],check=True,env=os.environ.copy(),timeout=30)


def test_manual_steps_capture_and_close():
    baseline={t.ident for t in threading.enumerate()}
    with CameraLab(width=80,height=60) as lab:
        first=lab.capture()
        assert first.stamp.step_index==0
        time.sleep(.05)
        assert lab.capture().sim_time==0
        lab.step(steps=10)
        observation,truth=lab.capture_with_truth()
        assert observation.stamp==truth.stamp
        assert observation.stamp.step_index==10
        assert lab.mode=='paused'
        assert 'red_box' in truth.object_poses
        lab.reset()
        assert lab.capture().stamp.run_id!=first.stamp.run_id
        assert lab.capture().sim_time==0
        for count in (0,-1,1.5,True):
            with pytest.raises(ValueError):lab.step(count)
    lab.close()
    with pytest.raises(RuntimeError,match='closed'):lab.capture()
    assert not [t for t in threading.enumerate() if t.ident not in baseline and t.name=='mujoco-render']


def test_scene_camera_project_roundtrip_and_failed_load(tmp_path):
    models=Path(__file__).resolve().parents[1]/'examples/models'
    with CameraLab(width=80,height=60) as lab:
        lab.scene.load(models/'table.xml')
        lab.scene.append(models/'simple_robot.urdf',position=(0,0,.75))
        lab.scene.set_pose('model_0001/base',position=(.1,0,.75),rotation=(1,0,0,0))
        name=lab.cameras.describe()[0]['name']
        lab.cameras.configure(name,width=100,height=70,fx=90,fy=95,parent='model_0001/base',position=(0,-2,1))
        lab.cameras.aim(name,'model_0001/base')
        frame=lab.capture().frames[0]
        path=tmp_path/'saved.camproj'
        lab.save_project(path)
        lab.cameras.move(name,steps=2)
        lab.load_project(path)
        restored=lab.capture().frames[0]
        assert restored.config==frame.config
        np.testing.assert_allclose(restored.T_world_camera,frame.T_world_camera,atol=1e-6)
        assert lab.cameras.describe()[0]['parent']=='model_0001/base'
        with pytest.raises((ValueError,OSError)):lab.scene.load(tmp_path/'missing.xml')
        assert lab.capture().frames[0].config==frame.config
        with pytest.raises(ValueError):lab.cameras.configure(name,width=0)
        assert lab.capture().frames[0].config==frame.config


def test_realtime_subscription_pause_and_camera_edit():
    with CameraLab(width=80,height=60) as lab:
        images=lab.data.subscribe_observations()
        lab.run()
        end=time.monotonic()+10
        while True:
            try:batch=images.get_nowait()
            except Empty:
                if time.monotonic()>end:raise
                time.sleep(.01);continue
            if batch.sim_time>.05:break
        lab.cameras.move(batch.frames[0].camera_id,steps=1)
        assert lab.mode=='running'
        lab.pause()
        first=lab.capture()
        time.sleep(.05)
        assert lab.capture().sim_time==first.sim_time
        lab.step(3)
        assert lab.capture().stamp.step_index==first.stamp.step_index+3
    assert images.closed


def test_invalid_startup_does_not_leak_worker(tmp_path):
    before={t.ident for t in threading.enumerate()}
    with pytest.raises((ValueError,OSError)):
        CameraLab(model=tmp_path/'missing.xml',width=80,height=60)
    assert not [t for t in threading.enumerate() if t.ident not in before and t.name=='mujoco-render']


def test_control_and_pose_evaluation_use_public_sdk():
    from camera_lab import ControlCommand,PoseEstimate
    with CameraLab(width=80,height=60) as lab:
        observations,truth=lab.capture_with_truth()
        frame=observations.frames[0]
        transform=np.linalg.inv(frame.T_world_camera) @ truth.object_poses['red_box']
        estimate=PoseEstimate(observations.stamp,frame.camera_id,'red_box',transform)
        error=lab.data.evaluate(estimate)
        assert error.status=='matched'
        assert error.translation_m==pytest.approx(0,abs=1e-9)
        assert error.rotation_rad==pytest.approx(0,abs=1e-6)
        assert any(a.name=='swing_control' for a in lab.data.actuators)
        replies=lab.data.subscribe_control_results()
        stamp=observations.stamp
        lab.data.submit_control(ControlCommand(stamp.run_id,stamp.model_revision,0,10,
                                time.monotonic_ns()+10_000_000_000,{'swing_control':.2}))
        lab.step(5)
        result=replies.get_nowait()
        assert result.accepted
        assert lab.capture().stamp.step_index==5


def test_concurrent_calls_and_shutdown_finish_requests():
    from concurrent.futures import ThreadPoolExecutor
    lab=CameraLab(width=80,height=60)
    try:
        with ThreadPoolExecutor(max_workers=4) as pool:
            futures=[pool.submit(lab.step,5) for _ in range(4)]
            for future in futures:future.result(timeout=10)
            assert lab.capture().stamp.step_index==20
            futures=[pool.submit(lab.capture) for _ in range(4)]
            lab.close()
            for future in futures:
                try:future.result(timeout=10)
                except RuntimeError as exc:assert 'closed' in str(exc)
    finally:lab.close()


def test_calibration_only_change_preserves_tracking_camera(tmp_path):
    path=tmp_path/'tracking.xml'
    path.write_text('<mujoco><worldbody><body name="target" pos="1 0 1"><freejoint/><geom size=".1"/></body><camera name="track" pos="0 -3 1" mode="targetbody" target="target"/></worldbody></mujoco>')
    with CameraLab(model=path,width=80,height=60) as lab:
        before=lab.capture().frames[0]
        lab.cameras.configure('track',fx=95,width=100)
        after=lab.capture().frames[0]
        assert after.config.fx==95
        np.testing.assert_allclose(after.T_world_camera,before.T_world_camera,atol=1e-8)
        lab.step(50)
        images,truth=lab.capture_with_truth()
        frame=images.frames[0]
        direction=truth.object_poses['target'][:3,3]-frame.T_world_camera[:3,3]
        np.testing.assert_allclose(frame.T_world_camera[:3,2],direction/np.linalg.norm(direction),atol=1e-7)


def test_close_rejects_queued_mutation_after_current_capture(tmp_path,monkeypatch):
    from concurrent.futures import ThreadPoolExecutor
    from camera_lab.render.worker import RenderSession
    entered=threading.Event();release=threading.Event()
    original=RenderSession.capture_sample
    def blocked_capture(session):
        entered.set()
        assert release.wait(10)
        return original(session)
    lab=CameraLab(width=80,height=60)
    monkeypatch.setattr(RenderSession,'capture_sample',blocked_capture)
    path=tmp_path/'must-not-be-written.camproj'
    def wait(predicate):
        end=time.monotonic()+5
        while not predicate() and time.monotonic()<end:time.sleep(.005)
        assert predicate()
    try:
        with ThreadPoolExecutor(max_workers=3) as pool:
            current=pool.submit(lab.capture)
            assert entered.wait(5)
            pending=pool.submit(lab.save_project,path)
            wait(lambda:lab._worker.commands.qsize()==1)
            closing=pool.submit(lab.close)
            wait(lab._worker.stop.is_set)
            release.set()
            current.result(timeout=10)
            closing.result(timeout=10)
            with pytest.raises(RuntimeError,match='closed'):pending.result(timeout=10)
        assert not path.exists()
    finally:
        release.set();lab.close()


def test_fatal_realtime_render_error_reaches_caller(monkeypatch):
    from camera_lab.render.worker import RenderSession
    with CameraLab(width=80,height=60) as lab:
        def fail(session):raise RuntimeError('synthetic render fault')
        monkeypatch.setattr(RenderSession,'capture_sample',fail)
        lab.run()
        end=time.monotonic()+5
        while not lab.data.closed and time.monotonic()<end:time.sleep(.01)
        assert lab.data.closed
        with pytest.raises(RuntimeError,match='synthetic render fault'):lab.capture()


def test_data_contract_import_does_not_initialize_mujoco():
    script='''
import sys
from camera_lab.contracts import FrameStamp
assert 'mujoco' not in sys.modules, 'Contract-only users must not initialize a render backend'
assert 'PySide6' not in sys.modules
'''
    subprocess.run([sys.executable,'-c',script],check=True,env=os.environ.copy(),timeout=10)
