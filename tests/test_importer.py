from pathlib import Path
import numpy as np
import pytest
from camera_lab.project.importer import SceneProject, ImportFailure
from camera_lab.render.worker import RenderSession

ROBOT = '''<robot name="robot"><link name="base"><visual><geometry><box size=".2 .2 .2"/></geometry></visual>
<collision><geometry><box size=".1 .1 .1"/></geometry></collision></link>
<link name="tip"><visual><geometry><sphere radius=".05"/></geometry></visual></link>
<joint name="fixed" type="fixed"><parent link="base"/><child link="tip"/><origin xyz="0 0 .3"/></joint></robot>'''
OBJECT = '''<mujoco><option gravity="0 0 -1"/><worldbody><body name="object" pos="0 0 .2">
<freejoint name="root"/><geom type="box" size=".1 .1 .1"/></body></worldbody></mujoco>'''


def write(tmp_path,name,text):
    path=tmp_path/name; path.write_text(text); return path


def test_open_urdf_preserves_visual_and_static_links(tmp_path):
    project=SceneProject.open(write(tmp_path,'robot.urdf',ROBOT))
    assert project.model.body('base').id > 0
    assert project.model.body('tip').id > 0
    assert project.model.ngeom == 3
    assert len(project.cameras) == 3
    assert project.model.nu == 0


def test_append_twice_namespaces_and_positions_without_changing_original(tmp_path):
    source=write(tmp_path,'object.xml',OBJECT)
    original=SceneProject.demo()
    first=original.append(source,position=(2,0,0))
    second=first.append(source,position=(3,0,0))
    assert original.model.nbody < first.model.nbody < second.model.nbody
    np.testing.assert_allclose(second.model.body('model_0001/object').pos,[2,0,.2])
    np.testing.assert_allclose(second.model.body('model_0002/object').pos,[3,0,.2])
    np.testing.assert_allclose(second.model.opt.gravity,[0,0,-9.81])
    assert any('gravity' in warning for warning in second.warnings)
    assert second.revision == original.revision+2
    assert first.cameras == second.cameras


def test_missing_asset_failure_does_not_mutate_project(tmp_path):
    original=SceneProject.demo(); xml=original.spec.to_xml()
    broken=write(tmp_path,'broken.xml','''<mujoco><asset><mesh name="bad" file="missing.obj"/></asset>
    <worldbody><geom type="mesh" mesh="bad"/></worldbody></mujoco>''')
    with pytest.raises(ImportFailure,match='missing.obj'):
        original.append(broken)
    assert original.spec.to_xml()==xml
    assert original.revision == 0


def test_package_uri_and_mesh_resource_survive_append(tmp_path):
    package=tmp_path/'robot_description'; package.mkdir()
    (package/'tetra.obj').write_text('v 0 0 0\nv .1 0 0\nv 0 .1 0\nv 0 0 .1\nf 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n')
    robot=write(tmp_path,'mesh.urdf','''<robot name="r"><link name="base"><visual><geometry>
    <mesh filename="package://robot_description/tetra.obj"/></geometry></visual></link></robot>''')
    with pytest.raises(ImportFailure,match='robot_description'):
        SceneProject.open(robot)
    project=SceneProject.demo().append(robot,packages={'robot_description':package})
    assert project.model.nmesh == 1
    assert project.model.body('model_0001/base').id > 0


def test_nested_include_relative_mesh_and_repeated_assets(tmp_path):
    (tmp_path/'parts').mkdir(); (tmp_path/'assets').mkdir()
    (tmp_path/'assets/t.obj').write_text('v 0 0 0\nv .1 0 0\nv 0 .1 0\nv 0 0 .1\nf 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n')
    (tmp_path/'parts/body.xml').write_text('<mujoco><worldbody><body name="meshbody"><geom type="mesh" mesh="m"/></body></worldbody></mujoco>')
    main=write(tmp_path,'main.xml','''<mujoco><compiler meshdir="assets"/>
    <asset><mesh name="m" file="t.obj"/></asset><include file="parts/body.xml"/></mujoco>''')
    project=SceneProject.open(main)
    again=project.append(main)
    assert again.model.nmesh==2
    assert again.model.mesh('model_0001/m').id >= 0


def test_loaded_scene_renders_without_driving_first_actuator(tmp_path):
    source=write(tmp_path,'actuated.xml','''<mujoco><worldbody><body name="b" pos="0 0 1">
    <joint name="j"/><geom type="capsule" size=".05 .2"/></body></worldbody>
    <actuator><motor joint="j"/></actuator></mujoco>''')
    project=SceneProject.open(source)
    with RenderSession(width=80,height=60,project=project) as session:
        session.advance_to(.1)
        assert session.data.ctrl[0]==0
        batch=session.capture()
        assert len(batch.frames)==3
        assert batch.stamp.model_revision==project.revision
        assert all(frame.rgb.std()>1 for frame in batch.frames)


def test_bad_xml_is_reported_with_source_path(tmp_path):
    path=write(tmp_path,'bad.xml','<mujoco>')
    with pytest.raises(ImportFailure,match='bad.xml'):
        SceneProject.open(path)


def test_append_rotation_moves_freejoint_initial_pose(tmp_path):
    import mujoco
    source=write(tmp_path,'offset.xml',OBJECT.replace('pos="0 0 .2"','pos=".2 0 .2"'))
    scene=SceneProject.demo().append(source,position=(2,0,0),quaternion=(2**-.5,0,0,2**-.5))
    data=mujoco.MjData(scene.model);mujoco.mj_forward(scene.model,data)
    np.testing.assert_allclose(data.body('model_0001/object').xpos,[2,.2,.2],atol=1e-12)


def test_append_copies_actuator_joint_references(tmp_path):
    source=write(tmp_path,'hinge.xml','''<mujoco><worldbody><body name="arm">
    <joint name="hinge"/><geom type="capsule" size=".05 .2"/></body></worldbody>
    <actuator><motor name="drive" joint="hinge"/></actuator></mujoco>''')
    scene=SceneProject.open(source).append(source)
    drive=scene.model.actuator('model_0001/drive').id
    hinge=scene.model.joint('model_0001/hinge').id
    assert scene.model.actuator_trnid[drive,0] == hinge


def test_example_robot_can_be_placed_on_table():
    root=Path(__file__).resolve().parents[1]/'examples/models'
    scene=SceneProject.open(root/'table.xml').append(root/'simple_robot.urdf',position=(0,0,.75))
    assert scene.model.body('model_0001/base').pos[2] == .75
    assert scene.model.njnt == 1


def test_append_reports_physics_flags_difference(tmp_path):
    source=write(tmp_path,'flags.xml','<mujoco><option><flag contact="disable"/></option><worldbody><geom type="sphere" size=".1"/></worldbody></mujoco>')
    scene=SceneProject.demo().append(source)
    assert any('disableflags' in warning for warning in scene.warnings)
    assert scene.model.opt.disableflags == 0
