import json
import shutil
import zipfile
import numpy as np
import pytest
from camera_lab.project.importer import SceneProject
from camera_lab.render.worker import RenderSession
from camera_lab.contracts import CameraConfig


def test_roundtrip_restores_initial_edits_cameras_and_layout(tmp_path):
    from camera_lab.project.store import save_project,load_project
    path=tmp_path/'saved.camproj'
    with RenderSession(width=80,height=60,project=SceneProject.demo()) as s:
        s.edit_body('red_box',(.2,.3,1.2),(1,0,0,0))
        s.edit_camera('front','red_box',(0,-1,1),(1,0,0,0),CameraConfig(90,70,80,80,44,34,.01,10))
        s.advance_to(.1);s.save_initial()
        q=s.data.qpos.copy();truth=s.capture_sample()[1]
        s.advance_to(.2)
        save_project(s,path,{'depth':True})
    project,layout=load_project(path)
    with RenderSession(width=80,height=60,project=project) as restored:
        assert restored.data.time==0
        np.testing.assert_allclose(restored.data.qpos,q)
        assert restored.scene_description()['cameras'][0]['parent']=='red_box'
        assert restored.capture().frames[0].rgb.shape==(70,90,3)
        np.testing.assert_allclose(restored.capture_sample()[1].object_poses['red_box'],truth.object_poses['red_box'],atol=1e-5)
        assert layout=={'depth':True}


def test_assets_survive_move_and_deleted_sources(tmp_path):
    from camera_lab.project.store import save_project,load_project
    src=tmp_path/'source';src.mkdir()
    (src/'mesh.obj').write_text('v 0 0 0\nv .1 0 0\nv 0 .1 0\nv 0 0 .1\nf 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n')
    (src/'scene.xml').write_text('<mujoco><asset><mesh name="m" file="mesh.obj"/></asset><worldbody><body name="b"><geom type="mesh" mesh="m"/></body></worldbody></mujoco>')
    project=SceneProject.demo().append(src/'scene.xml').append(src/'scene.xml')
    path=tmp_path/'first.camproj'
    with RenderSession(width=80,height=60,project=project) as s:save_project(s,path)
    shutil.rmtree(src)
    moved=tmp_path/'moved';moved.mkdir();path.rename(moved/'p.camproj')
    project,_=load_project(moved/'p.camproj')
    assert project.model.nmesh==2
    with RenderSession(width=80,height=60,project=project) as s:
        save_project(s,moved/'again.camproj')
        assert s.capture().frames[0].rgb.std()>0
    assert load_project(moved/'again.camproj')[0].model.nmesh==2


def test_failed_replace_keeps_previous_project(tmp_path,monkeypatch):
    from camera_lab.project import store
    path=tmp_path/'saved.camproj'
    with RenderSession(width=80,height=60,project=SceneProject.demo()) as s:
        store.save_project(s,path)
        original=path.read_bytes()
        def fail(*args):raise OSError('disk failure')
        monkeypatch.setattr(store.os,'replace',fail)
        with pytest.raises(OSError,match='disk failure'):store.save_project(s,path)
        assert path.read_bytes()==original
        assert store.load_project(path)[0].model.nbody>0
        assert not list(tmp_path.glob('*.tmp'))


def test_unknown_schema_is_rejected(tmp_path):
    from camera_lab.project.store import load_project
    path=tmp_path/'bad.camproj'
    with zipfile.ZipFile(path,'w') as z:z.writestr('project.json',json.dumps({'schema_version':999}))
    with pytest.raises(ValueError,match='schema'):load_project(path)


def test_texture_assets_and_same_named_meshes_are_portable(tmp_path):
    import struct,zlib
    from camera_lab.project.store import save_project,load_project
    def chunk(kind,data):return struct.pack('!I',len(data))+kind+data+struct.pack('!I',zlib.crc32(kind+data)&0xffffffff)
    png=b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!2I5B',1,1,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(b'\x00\xff\x00\x00'))+chunk(b'IEND',b'')
    project=SceneProject.demo()
    folders=[]
    for i in range(2):
        folder=tmp_path/str(i);folder.mkdir();folders.append(folder)
        (folder/'tex.png').write_bytes(png)
        (folder/'mesh.obj').write_text(f'v 0 0 0\nv {i+1} 0 0\nv 0 1 0\nv 0 0 1\nf 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n')
        (folder/'model.xml').write_text('<mujoco><asset><mesh name="m" file="mesh.obj"/><texture name="t" type="2d" file="tex.png"/><material name="mat" texture="t"/></asset><worldbody><body name="b"><geom type="mesh" mesh="m" material="mat"/></body></worldbody></mujoco>')
        project=project.append(folder/'model.xml')
    path=tmp_path/'textures.camproj'
    with RenderSession(width=80,height=60,project=project) as session:save_project(session,path)
    for folder in folders:shutil.rmtree(folder)
    restored,_=load_project(path)
    assert restored.model.nmesh==2
    assert restored.model.ntex==3
    np.testing.assert_allclose(restored.model.mesh_vert,project.model.mesh_vert,atol=1e-5)


def test_invalid_camera_parent_rejected_before_session(tmp_path):
    from camera_lab.project.store import save_project,load_project
    path=tmp_path/'valid.camproj'
    with RenderSession(width=80,height=60,project=SceneProject.demo()) as s:save_project(s,path)
    with zipfile.ZipFile(path) as z:entries={n:z.read(n) for n in z.namelist()}
    metadata=json.loads(entries['project.json'])
    metadata['state']['cameras'][0]['parent']='missing_body'
    entries['project.json']=json.dumps(metadata)
    bad=tmp_path/'bad.camproj'
    with zipfile.ZipFile(bad,'w') as z:
        for n,data in entries.items():z.writestr(n,data)
    with pytest.raises(ValueError,match='parent'):load_project(bad)
