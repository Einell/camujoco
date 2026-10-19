import numpy as np
from camera_lab.render.worker import RenderSession


def test_batch_truth_and_cameras_share_one_sample_identity():
    with RenderSession(width=80,height=60) as session:
        session.advance_to(.1)
        batch, truth = session.capture_sample()
        assert truth.stamp == batch.stamp
        assert all(frame.stamp == batch.stamp for frame in batch.frames)
        assert batch.stamp.step_index == 50
        assert abs(batch.stamp.sim_time_s - .1) < 1e-10
        assert batch.stamp.host_monotonic_ns > 0
        pose = truth.object_poses['red_box']
        np.testing.assert_allclose(pose[:3,3], session.data.body('red_box').xpos)
        assert not pose.flags.writeable
        old = pose.copy()
        session.advance_to(.3)
        newer = session.capture()
        assert newer.stamp.batch_id == batch.stamp.batch_id + 1
        np.testing.assert_array_equal(pose,old)
        assert not hasattr(batch,'object_poses')
        session.reset()
        reset = session.capture()
        assert reset.stamp.run_id != batch.stamp.run_id
        assert reset.stamp.step_index == 0
        assert reset.stamp.model_revision == batch.stamp.model_revision


def test_mounted_camera_extrinsics_follow_body_motion():
    xml='''<mujoco><worldbody><body name="mount" pos="0 0 2"><freejoint/>
    <geom type="sphere" size=".1"/><camera name="mounted" pos=".1 0 0"/></body>
    <geom type="plane" size="5 5 .1"/></worldbody></mujoco>'''
    with RenderSession(width=80,height=60,xml=xml,cameras=['mounted']) as session:
        first=session.capture().frames[0]
        session.advance_to(.2)
        batch,truth=session.capture_sample()
        last=batch.frames[0]
        assert last.T_world_camera[2,3] < first.T_world_camera[2,3]
        np.testing.assert_allclose(last.T_world_camera[:3,3], truth.object_poses['mount'][:3,3]+[.1,0,0])


def test_anonymous_body_ids_do_not_overwrite_explicit_names():
    xml='''<mujoco><worldbody><camera name="c" pos="0 0 3"/>
    <body pos="1 0 0"><geom type="sphere" size=".1"/></body>
    <body name="body:1" pos="2 0 0"><geom type="sphere" size=".1"/></body>
    </worldbody></mujoco>'''
    with RenderSession(width=80,height=60,xml=xml,cameras=['c']) as session:
        _,truth=session.capture_sample()
        assert len(truth.object_poses) == 2
        assert sorted(pose[0,3] for pose in truth.object_poses.values()) == [1,2]
        assert truth.object_poses['body:1'][0,3] == 2
        _,again=session.capture_sample()
        assert truth.object_poses.keys() == again.object_poses.keys()
