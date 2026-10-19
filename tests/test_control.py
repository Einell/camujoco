from time import monotonic_ns
import numpy as np
import pytest
from camera_lab.render.worker import RenderSession


def command(session,values,**kw):
    from camera_lab.contracts import ControlCommand
    args=dict(run_id=session.run_id,model_revision=session.model_revision,target_step=session.step_index,
              valid_until_step=session.step_index+3,expires_at_ns=monotonic_ns()+1_000_000_000,
              values=values)
    args.update(kw);return ControlCommand(**args)


def test_control_drives_actuator_and_expires_to_defaults():
    with RenderSession(width=80,height=60) as s:
        s.submit_control(command(s,{'swing_control':.3}))
        s.single_step();assert s.data.ctrl[0]==.3
        s.advance_to(.02);assert s.data.ctrl[0]==0
        assert s.data.qpos[-1]!=0


def test_invalid_and_stale_control_do_not_mutate_state():
    with RenderSession(width=80,height=60) as s:
        for values in ({'missing':0},{'swing_control':99},{'swing_control':float('nan')}):
            with pytest.raises(ValueError):s.submit_control(command(s,values))
        old=command(s,{'swing_control':.2})
        s.reset()
        with pytest.raises(ValueError,match='version'):s.submit_control(old)
        with pytest.raises(ValueError,match='expired'):
            s.submit_control(command(s,{'swing_control':.2},expires_at_ns=0))


def test_scheduled_control_applies_only_at_target_step_and_reset_clears():
    with RenderSession(width=80,height=60) as s:
        s.submit_control(command(s,{'swing_control':.4},target_step=2,valid_until_step=5))
        s.single_step();assert s.data.ctrl[0]==0
        s.single_step();assert s.data.ctrl[0]==0
        s.single_step();assert s.data.ctrl[0]==.4
        s.reset();s.single_step();assert s.data.ctrl[0]==0


def test_command_deadline_expires_even_while_paused(monkeypatch):
    from camera_lab.api import control
    with RenderSession(width=80,height=60) as s:
        cmd=command(s,{'swing_control':.2})
        s.submit_control(cmd);s._controls.apply()
        assert s.data.ctrl[0]==.2
        monkeypatch.setattr(control,'monotonic_ns',lambda:cmd.expires_at_ns+1)
        s._controls.apply()
        assert s.data.ctrl[0]==0 and s.data.time==0


def test_unactuated_model_rejects_commands():
    with RenderSession(width=80,height=60,xml='<mujoco><worldbody><geom size=".1"/><camera name="c" pos="0 -1 1"/></worldbody></mujoco>',cameras=('c',)) as s:
        with pytest.raises(ValueError,match='unsupported'):s.submit_control(command(s,{'motor':.1}))


def test_newer_command_supersedes_same_actuator_at_same_step():
    with RenderSession(width=80,height=60) as s:
        s.submit_control(command(s,{'swing_control':.2}))
        s.submit_control(command(s,{'swing_control':-.2}))
        s.single_step();assert s.data.ctrl[0]==-.2


def test_restore_clears_full_old_schedule_before_new_command():
    with RenderSession(width=80,height=60) as s:
        for _ in range(64):
            s.submit_control(command(s,{'swing_control':.1},target_step=10,valid_until_step=20))
        snap=s.snapshot();s.restore_snapshot(snap)
        s.submit_control(command(s,{'swing_control':.25}))
        s.single_step();assert s.data.ctrl[0]==.25
