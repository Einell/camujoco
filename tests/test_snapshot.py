import numpy as np
import pytest
from camera_lab.render.worker import RenderSession


def test_snapshot_restores_full_integration_state_and_rejects_old_run():
    with RenderSession(width=80,height=60) as s:
        s.advance_to(.1)
        s.data.ctrl[:]=.2
        s.data.qfrc_applied[:]=.3
        snapshot=s.snapshot()
        assert not snapshot.state.flags.writeable
        s.advance_to(.2)
        s.restore_snapshot(snapshot)
        restored=s.snapshot()
        np.testing.assert_allclose(restored.state,snapshot.state,atol=1e-12)
        assert s.step_index==snapshot.step_index
        assert s.run_id!=snapshot.run_id
        with pytest.raises(ValueError,match='stale'):
            s.restore_snapshot(snapshot)


def test_snapshot_rejects_different_session():
    with RenderSession(width=80,height=60) as first:
        snap=first.snapshot()
        with RenderSession(width=80,height=60) as second:
            with pytest.raises(ValueError,match='stale'):
                second.restore_snapshot(snap)
