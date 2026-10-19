from types import SimpleNamespace
import numpy as np
import pytest


def frame(depth=1.):
    return SimpleNamespace(K=np.array([[100,0,50],[0,100,50],[0,0,1.]]),
        T_world_camera=np.eye(4),depth=np.full((100,100),depth),valid_mask=np.ones((100,100),bool))


def test_surface_point_visibility_distinguishes_occlusion_and_range():
    from camera_lab.evaluation import project_feature
    assert project_feature(frame(),[0,0,1],diameter_m=.02)['pixels']==pytest.approx(2)
    assert project_feature(frame(),[0,0,1])['status']=='visible'
    assert project_feature(frame(.5),[0,0,1])['status']=='occluded'
    assert project_feature(frame(),[1,0,1])['status']=='out_of_frame'
    assert project_feature(frame(),[0,0,-1])['status']=='behind_camera'
    assert project_feature(frame(),[0,0,1],min_range_m=1.1)['status']=='out_of_range'
    assert project_feature(frame(),[0,0,1],diameter_m=.001,min_pixels=2)['status']=='too_small'
    assert project_feature(frame(float('nan')),[0,0,1])['status']=='invalid_depth'


def test_duration_weighted_joint_visibility_and_phase_gaps():
    from camera_lab.evaluation import summarize
    rows=[dict(time_s=t,duration_s=d,phase=p,visible=v) for t,d,p,v in
          [(0,.1,'close',{'a':True,'b':True}),(.1,.2,'close',{'a':False,'b':True}),
           (.3,.1,'insert',{'a':False,'b':False}),(.4,.6,'insert',{'a':True,'b':True})]]
    r=summarize(rows)
    assert r['all']['joint_visible_fraction']==pytest.approx(.7)
    assert r['all']['longest_joint_loss_s']==pytest.approx(.3)
    assert r['phases']['close']['joint_visible_fraction']==pytest.approx(1/3)
    assert r['all']['features']['a']['longest_loss_s']==pytest.approx(.3)
    with pytest.raises(ValueError):summarize([rows[1],rows[0]])


def test_rendered_occlusion_and_batch_runner(tmp_path):
    from camera_lab.experiments.wrist import run_experiment
    model=tmp_path/'scene.xml'
    model.write_text('''<mujoco><worldbody>
    <body name="target" pos="0 0 1"><geom type="box" size=".1 .1 .1"/></body>
    <body name="blocker" pos=".4 0 .5"><joint name="slide" type="slide" axis="1 0 0"/><geom type="box" size=".15 .15 .05"/></body>
    <camera name="camera" pos="0 0 0" quat="0 1 0 0"/>
    </worldbody></mujoco>''')
    config=dict(model=str(model),width=100,height=100,
        candidates=[dict(name='test',camera='camera',parent='world',position=[0,0,0],rotation=[1,0,0,0])],
        features=[dict(name='face',body='target',point=[0,0,-.1])],
        samples=[dict(time_s=0,duration_s=.1,phase='open',qpos=[0]),
                 dict(time_s=.1,duration_s=.1,phase='closed',qpos=[-.4])])
    result=run_experiment(config,tmp_path/'result')
    r=result['candidates']['test']
    assert r['phases']['open']['joint_visible_fraction']==1
    assert r['phases']['closed']['joint_visible_fraction']==0
    assert (tmp_path/'result'/'frames.csv').exists()
    assert (tmp_path/'result'/'summary.json').exists()
    assert (tmp_path/'result'/'input.camproj').exists()
    assert len(result['project_sha256'])==64


def test_generate_24_mounts_aims_optical_axis_at_target():
    from camera_lab.experiments.layouts import generate_candidates
    from camera_lab.render.editing import matrix
    mounts={name:dict(origin=[0,0,0],outward=v) for name,v in
            [('palm',[0,-1,0]),('thumb',[1,0,0]),('little',[-1,0,0]),('back',[0,1,0])]}
    candidates=generate_candidates('cam','wrist',mounts,{'near':[0,0,.2],'mid':[0,0,.3],'far':[0,0,.4]})
    assert len(candidates)==24
    c=candidates[0]
    np.testing.assert_allclose(c['position'],[0,-.02,0])
    optical=matrix(np.array(c['rotation']))[:,2]
    direction=np.array([0,0,.2])-c['position'];direction/=np.linalg.norm(direction)
    np.testing.assert_allclose(optical,direction,atol=1e-8)
