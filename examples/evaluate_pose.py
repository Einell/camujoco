"""Synthetic 1cm error against the same batch's ground truth."""
from dataclasses import asdict
import numpy as np
from camera_lab.api import DataAPI
from camera_lab.contracts import PoseEstimate
from camera_lab.render.worker import RenderSession

api=DataAPI()
with RenderSession(width=320,height=180) as renderer:
    observation,truth=renderer.capture_sample()
    api.publish(observation,truth)
    frame=observation.frames[0]
    name=next(iter(truth.object_poses))
    prediction=np.linalg.inv(frame.T_world_camera) @ truth.object_poses[name]
    prediction[0,3]+=.01
    result=api.evaluate(PoseEstimate(observation.stamp,frame.camera_id,name,prediction))
    print(asdict(result))
api.close()
