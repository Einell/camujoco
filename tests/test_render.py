import numpy as np
import pytest
from camera_lab.render.worker import RenderSession, LatestBatch


def test_three_cameras_have_independent_rgb_and_metric_depth():
    with RenderSession(width=160, height=90) as session:
        batch = session.capture()
        assert len(batch.frames) == 3
        assert batch.overview.shape == (90, 160, 3)
        for frame in batch.frames:
            assert frame.rgb.shape == (90, 160, 3)
            assert frame.rgb.dtype == np.uint8
            assert frame.rgb.std() > 5
            assert frame.depth.shape == (90, 160)
            assert frame.depth.dtype == np.float32
            assert np.isfinite(frame.depth).any()
            assert not frame.rgb.flags.writeable
        assert not np.array_equal(batch.frames[0].rgb, batch.frames[1].rgb)


def test_motion_reset_and_previous_frames_are_stable():
    with RenderSession(width=160, height=90) as session:
        first = session.capture()
        saved = first.frames[0].rgb.copy()
        initial_qpos = session.data.qpos.copy()
        initial_xpos = session.data.xpos.copy()
        session.advance_to(0.5)
        moving = session.capture()
        assert moving.sim_time >= 0.5
        assert not np.array_equal(saved, moving.frames[0].rgb)
        assert np.array_equal(saved, first.frames[0].rgb)
        session.reset()
        reset = session.capture()
        assert reset.sim_time == 0
        np.testing.assert_array_equal(session.data.qpos, initial_qpos)
        np.testing.assert_array_equal(session.data.xpos, initial_xpos)
        # NVIDIA rasterization can differ by one 8-bit level even with no step.
        np.testing.assert_allclose(saved, reset.frames[0].rgb, atol=1, rtol=0)


def test_depth_measures_two_meter_plane():
    xml = '''<mujoco><visual><global offwidth="80" offheight="60"/></visual>
      <worldbody><geom type="plane" size="5 5 .1"/>
      <camera name="test" pos="0 0 2"/></worldbody></mujoco>'''
    with RenderSession(width=80, height=60, xml=xml, cameras=['test']) as session:
        depth = session.capture().frames[0].depth
        np.testing.assert_allclose(depth[20:40, 30:50], 2.0, atol=.001)


def test_invalid_dimensions_rejected_before_gl_initialization():
    with pytest.raises(ValueError):
        RenderSession(width=0)
    with pytest.raises(ValueError):
        RenderSession(camera_count=4)


def test_latest_batch_drops_stale_frames_without_growing():
    box = LatestBatch()
    box.put('first'); box.put('second'); box.put('third')
    assert box.take() == 'third'
    assert box.dropped == 2
    assert box.take() is None


def test_context_can_be_recreated():
    for _ in range(3):
        with RenderSession(width=80, height=60) as session:
            assert session.capture().frames[0].rgb.shape == (60, 80, 3)
