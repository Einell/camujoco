import numpy as np
import pytest
from camera_lab.render.worker import RenderSession


def test_single_step_advances_exactly_once_and_updates_truth():
    with RenderSession(width=80,height=60) as s:
        before=s.capture()
        s.single_step()
        batch,truth=s.capture_sample()
        assert batch.sim_time==pytest.approx(s.model.opt.timestep)
        assert batch.stamp.step_index==1
        assert batch.stamp.run_id==before.stamp.run_id
        assert all(f.stamp==truth.stamp for f in batch.frames)
        assert s.capture().sim_time==batch.sim_time


def test_save_initial_clears_motion_and_reset_restores_pose():
    with RenderSession(width=80,height=60) as s:
        s.advance_to(.2)
        q=s.data.qpos.copy(); old=s.run_id
        s.save_initial()
        np.testing.assert_allclose(s.data.qpos,q)
        assert s.data.time==0 and s.run_id!=old
        assert not np.any(s.data.qvel) and not np.any(s.data.ctrl)
        s.advance_to(.1)
        s.reset()
        np.testing.assert_allclose(s.data.qpos,q)
        assert s.step_index==0


def test_saved_initial_survives_append_copy():
    from camera_lab.project.importer import SceneProject
    with RenderSession(width=80,height=60,project=SceneProject.demo()) as s:
        s.advance_to(.15);s.save_initial();q=s.data.qpos.copy()
        with RenderSession(width=80,height=60,project=SceneProject.demo()) as other:
            s.copy_edits_to(other)
            np.testing.assert_allclose(other.data.qpos,q)
            other.advance_to(.05);other.reset()
            np.testing.assert_allclose(other.data.qpos,q)


def test_modes_freeze_pause_and_reject_commands_from_old_run():
    from camera_lab.sim.engine import Simulation,Mode
    with RenderSession(width=80,height=60) as s:
        sim=Simulation(s)
        sim.advance(.1);assert s.data.time==0
        sim.transition('run');sim.advance(.1)
        sim.transition('pause');time=s.data.time
        s.move_camera('front',1,0)
        sim.advance(.5);assert s.data.time==time
        old=s.run_id
        sim.transition('edit');assert sim.mode==Mode.EDIT and s.data.time==0
        with pytest.raises(ValueError,match='stale'):
            sim.transition('run',expected_run_id=old)
        assert sim.mode==Mode.EDIT


def test_mocap_edit_and_saved_initial_match_truth():
    xml='<mujoco><worldbody><body name="target" mocap="true" pos="0 0 1"><geom size=".1"/></body><camera name="front" pos="0 -3 1"/></worldbody></mujoco>'
    with RenderSession(width=80,height=60,xml=xml,cameras=('front',)) as s:
        s.edit_body('target',(1,2,3),(1,0,0,0))
        np.testing.assert_allclose(s.capture_sample()[1].object_poses['target'][:3,3],[1,2,3])
        s.data.mocap_pos[0]=[2,3,4]
        assert s.scene_description()['bodies'][0]['position']==[2,3,4]
        s.save_initial();s.reset()
        np.testing.assert_allclose(s.capture_sample()[1].object_poses['target'][:3,3],[2,3,4])
        with RenderSession(width=80,height=60,xml=xml,cameras=('front',)) as other:
            s.copy_edits_to(other)
            np.testing.assert_allclose(other.capture_sample()[1].object_poses['target'][:3,3],[2,3,4])
