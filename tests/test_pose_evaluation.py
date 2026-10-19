import numpy as np
import pytest
from camera_lab.render.worker import RenderSession


def test_pose_error_uses_original_frame_and_expires():
    from camera_lab.api.streams import DataAPI
    from camera_lab.contracts import PoseEstimate
    api=DataAPI(history_size=2)
    with RenderSession(width=80,height=60) as s:
        image,truth=s.capture_sample();api.publish(image,truth)
        frame=image.frames[0];name=next(iter(truth.object_poses))
        target=np.linalg.inv(frame.T_world_camera) @ truth.object_poses[name]
        prediction=target.copy();prediction[0,3]+=.01
        estimate=PoseEstimate(image.stamp,'front',name,prediction)
        s.advance_to(.1);api.publish(*s.capture_sample())
        error=api.evaluate(estimate)
        assert error.status=='matched'
        assert error.translation_m==pytest.approx(.01)
        assert error.rotation_rad==pytest.approx(0,abs=1e-7)
        api.publish(*s.capture_sample())
        assert api.evaluate(estimate).status=='unavailable'


def test_rotation_error_and_invalid_pose():
    from camera_lab.api.streams import DataAPI
    from camera_lab.contracts import PoseEstimate
    api=DataAPI()
    with RenderSession(width=80,height=60) as s:
        image,truth=s.capture_sample();api.publish(image,truth)
        frame=image.frames[0];name=next(iter(truth.object_poses))
        target=np.linalg.inv(frame.T_world_camera) @ truth.object_poses[name]
        target[:3,:3]=target[:3,:3] @ np.array([[0,-1,0],[1,0,0],[0,0,1]])
        assert api.evaluate(PoseEstimate(image.stamp,'front',name,target)).rotation_rad==pytest.approx(np.pi/2)
        with pytest.raises(ValueError):PoseEstimate(image.stamp,'front',name,np.zeros((4,4)))
        estimate=PoseEstimate(image.stamp,'front',name,target)
        s.reset();api.publish(*s.capture_sample())
        assert api.evaluate(estimate).status=='unavailable'
