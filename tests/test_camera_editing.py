import numpy as np
import pytest
from camera_lab.render.worker import RenderSession
from camera_lab.contracts import CameraConfig


def test_static_body_edit_updates_truth_and_reset():
    with RenderSession(width=80,height=60) as s:
        body=next(name for bid,name in s._body_names.items() if s.model.body_jntnum[bid] and s.model.jnt_type[s.model.body_jntadr[bid]]==0)
        s.edit_body(body,(.4,.2,1.5),(1,0,0,0))
        batch,truth=s.capture_sample()
        np.testing.assert_allclose(truth.object_poses[body][:3,3],[.4,.2,1.5])
        s.reset()
        np.testing.assert_allclose(s.capture_sample()[1].object_poses[body][:3,3],[.4,.2,1.5])
        assert batch.sim_time==0


def test_camera_mount_tracks_parent_and_overview_is_independent():
    with RenderSession(width=80,height=60) as s:
        name=next(iter(s._body_names.values()))
        before=s.view.azimuth
        s.edit_camera('front',name,(0,0,1),(1,0,0,0),CameraConfig(90,70,80,80,44.5,34.5,.01,10))
        batch,truth=s.capture_sample()
        expected=truth.object_poses[name] @ np.array([0,0,1,1])
        np.testing.assert_allclose(batch.frames[0].T_world_camera[:3,3],expected[:3])
        assert batch.frames[0].rgb.shape==(70,90,3)
        assert s.view.azimuth==before
        s.orbit(30,10)
        np.testing.assert_allclose(s.capture().frames[0].T_world_camera,batch.frames[0].T_world_camera)


def test_look_at_and_invalid_edit_are_atomic():
    with RenderSession(width=80,height=60) as s:
        s.aim_camera('front',(0,0,.8))
        frame=s.capture().frames[0]
        direction=np.array([0,0,.8])-frame.T_world_camera[:3,3]
        np.testing.assert_allclose(frame.T_world_camera[:3,2],direction/np.linalg.norm(direction),atol=1e-7)
        before=frame.T_world_camera.copy()
        with pytest.raises(ValueError):
            s.edit_camera('front','world',(0,0,0),(0,0,0,0),frame.config)
        np.testing.assert_allclose(s.capture().frames[0].T_world_camera,before)


def test_mount_follows_moving_body_and_append_preserves_edits():
    from camera_lab.project.importer import SceneProject
    with RenderSession(width=80,height=60,project=SceneProject.demo()) as s:
        body=next(name for bid,name in s._body_names.items() if s.model.body_jntnum[bid] and s.model.jnt_type[s.model.body_jntadr[bid]]==0)
        s.edit_body(body,(.3,0,2),(1,0,0,0))
        s.edit_camera('front',body,(0,0,.5),(1,0,0,0),s.camera_config('front'))
        s.advance_to(.1)
        batch,truth=s.capture_sample()
        expected=truth.object_poses[body] @ np.array([0,0,.5,1])
        np.testing.assert_allclose(batch.frames[0].T_world_camera[:3,3],expected[:3])
        with RenderSession(width=80,height=60,project=SceneProject.demo()) as other:
            s.copy_edits_to(other)
            np.testing.assert_allclose(other.capture_sample()[1].object_poses[body][:3,3],[.3,0,2])
            assert other.scene_description()['cameras'][0]['parent']==body


def test_frustum_overlay_only_changes_overview():
    with RenderSession(width=120,height=90) as s:
        s.show_frustums=False
        before=s.capture()
        s.show_frustums=True
        after=s.capture()
        assert np.any(before.overview!=after.overview)
        np.testing.assert_array_equal(before.frames[0].rgb,after.frames[0].rgb)


def test_append_preserves_unedited_tracking_camera(tmp_path):
    from camera_lab.project.importer import SceneProject
    path=tmp_path/'tracking.xml'
    path.write_text('<mujoco><worldbody><body name="target" pos="1 0 0"><geom size=".1"/></body><camera name="track" pos="0 -3 1" mode="targetbody" target="target"/></worldbody></mujoco>')
    project=SceneProject.open(path)
    with RenderSession(width=80,height=60,project=project) as old:
        before=old.capture().frames[0].T_world_camera
        with RenderSession(width=80,height=60,project=project.append(path)) as new:
            old.copy_edits_to(new)
            np.testing.assert_allclose(new.capture().frames[0].T_world_camera,before)
