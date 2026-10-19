from dataclasses import replace
import numpy as np
import pytest
from camera_lab.contracts import CameraConfig
from camera_lab.render.worker import RenderSession

MARKER = '''<mujoco><visual><quality offsamples="0"/></visual><worldbody>
<camera name="test" pos="0 0 2"/>
<body name="marker" pos=".2 -.1 0"><geom type="sphere" size=".035" rgba="1 0 0 1"/></body>
</worldbody></mujoco>'''


def test_off_center_non_square_intrinsics_match_rendered_marker():
    config = CameraConfig(width=200, height=140, fx=160, fy=110, cx=83, cy=57, near=.1, far=5)
    with RenderSession(width=200, height=140, xml=MARKER, cameras=['test']) as session:
        session.configure_camera('test', config)
        frame = session.capture().frames[0]
        rgb = frame.rgb.astype(int)
        y, x = np.nonzero((rgb[:,:,0] > 30) & (rgb[:,:,0] > rgb[:,:,1] * 2))
        assert len(x) > 3
        # Optical coordinates (.2, .1, 2), independently calculated projection.
        assert abs(x.mean() - 99) < 1
        assert abs(y.mean() - 62.5) < 1
        np.testing.assert_array_equal(frame.K, [[160,0,83],[0,110,57],[0,0,1]])
        np.testing.assert_array_equal(frame.T_world_camera,
            [[1,0,0,0],[0,-1,0,0],[0,0,-1,2],[0,0,0,1]])


def test_reconfigure_changes_resolution_and_metadata_together():
    with RenderSession(width=160, height=90) as session:
        before = session.capture()
        config = replace(session.camera_config('front'), width=100, height=80, fx=90, fy=100, cx=41, cy=33)
        session.configure_camera('front', config)
        after = session.capture()
        assert after.frames[0].rgb.shape == (80,100,3)
        assert after.frames[0].depth.shape == (80,100)
        assert after.frames[1].rgb.shape == (90,160,3)
        assert after.frames[0].camera_revision == before.frames[0].camera_revision + 1
        assert after.frames[1].camera_revision == before.frames[1].camera_revision
        assert after.frames[0].K[0,2] == 41
        assert before.frames[0].rgb.shape == (90,160,3)
        assert not before.frames[0].K.flags.writeable


def test_per_camera_clipping_masks_background_and_restores_global_settings():
    xml = '''<mujoco><worldbody><geom type="plane" size="5 5 .1"/>
    <camera name="near" pos="0 0 2"/><camera name="far" pos="0 0 2"/></worldbody></mujoco>'''
    with RenderSession(width=80,height=60,xml=xml,cameras=['near','far']) as session:
        original = (session.model.vis.map.znear, session.model.vis.map.zfar)
        session.configure_camera('near', replace(session.camera_config('near'), near=.1, far=1))
        session.configure_camera('far', replace(session.camera_config('far'), near=.1, far=4))
        batch = session.capture()
        assert not batch.frames[0].valid_mask.any()
        assert np.isnan(batch.frames[0].depth).all()
        np.testing.assert_allclose(batch.frames[1].depth[20:40,30:50],2,atol=.001)
        assert batch.frames[1].valid_mask[30,40]
        assert (session.model.vis.map.znear, session.model.vis.map.zfar) == original


@pytest.mark.parametrize('changes', [dict(width=0),dict(width=1.5),dict(height=True),
    dict(fx=0),dict(fy=float('nan')),dict(cx=float('inf')),dict(near=-1),dict(far=.001)])
def test_invalid_calibration_is_rejected(changes):
    with pytest.raises(ValueError):
        replace(CameraConfig(160,90,100,100,79.5,44.5,.1,5), **changes)


@pytest.mark.parametrize('names', [[], ['front','front']])
def test_camera_ids_must_be_nonempty_and_unique(names):
    with pytest.raises(ValueError):
        RenderSession(cameras=names)
