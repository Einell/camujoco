import numpy as np
import pytest
from camera_lab import CameraLab


def test_seek_is_atomic_pauses_and_moves_wrist_camera(tmp_path):
    path=tmp_path/'robot.xml'
    path.write_text('''<mujoco><worldbody><body name="wrist"><joint name="turn" axis="0 0 1" range="-90 90"/><geom size=".1"/><camera name="wrist_cam" pos=".2 0 0"/></body></worldbody></mujoco>''')
    with CameraLab(model=path,width=80,height=60) as lab:
        state=lab.state()
        assert state['joints'][0]['name']=='turn'
        before=lab.capture().frames[0].T_world_camera
        lab.run();lab.seek(qpos=[np.pi/2],time_s=1)
        assert lab.mode=='paused'
        after=lab.capture().frames[0].T_world_camera
        np.testing.assert_allclose(after[:3,3],[0,.2,0],atol=1e-7)
        assert not np.allclose(before,after)
        with pytest.raises(ValueError):lab.seek(qpos=[float('nan')],time_s=2)
        assert lab.state()['time_s']==1
        np.testing.assert_allclose(lab.state()['qpos'],[np.pi/2])


def test_attach_to_parent_preserves_hierarchy(tmp_path):
    arm=tmp_path/'arm.xml';hand=tmp_path/'hand.xml'
    arm.write_text('<mujoco><worldbody><body name="wrist"><joint name="turn"/><geom size=".1"/></body></worldbody></mujoco>')
    hand.write_text('<mujoco><worldbody><body name="palm"><geom size=".05"/></body></worldbody></mujoco>')
    with CameraLab(model=arm,width=80,height=60) as lab:
        lab.scene.attach(hand,parent='wrist',position=(.2,0,0))
        lab.seek(qpos=[np.pi/2])
        _,truth=lab.capture_with_truth()
        np.testing.assert_allclose(truth.object_poses['model_0001/palm'][:3,3],[0,.2,0],atol=1e-7)
        with pytest.raises(ValueError):lab.scene.attach(hand,parent='missing')
        assert len(lab.state()['qpos'])==1


def test_ball_limit_rejection_preserves_state(tmp_path):
    path=tmp_path/'ball.xml'
    path.write_text('<mujoco><worldbody><body><joint type="ball" limited="true" range="0 30"/><geom size=".1"/></body></worldbody></mujoco>')
    with CameraLab(model=path,width=80,height=60) as lab:
        with pytest.raises(ValueError):lab.seek(qpos=[0,1,0,0],time_s=2)
        np.testing.assert_allclose(lab.state()['qpos'],[1,0,0,0])
        assert lab.state()['time_s']==0
