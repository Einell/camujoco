"""Portable project archives with atomic replacement and explicit schema."""
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import zipfile
import mujoco
import numpy as np
from .importer import SceneProject
from ..contracts import CameraConfig
from ..render.editing import pose

SCHEMA=1
FILE_KEYS=('file','fileup','filedown','fileleft','fileright','filefront','fileback')


def _scene_assets(spec):
    root=ET.fromstring(spec.to_xml())
    compiler=root.find('compiler')
    directories={} if compiler is None else dict(compiler.attrib)
    assets={}
    for section in root.findall('asset'):
        for node in section:
            for key in FILE_KEYS:
                value=node.get(key)
                if not value:continue
                if value in spec.assets:
                    data=spec.assets[value]
                else:
                    folder=directories.get('texturedir' if node.tag=='texture' else 'meshdir','')
                    source=Path(value)
                    if not source.is_absolute():source=Path(spec.modelfiledir)/folder/source
                    data=source.read_bytes()
                suffix=Path(value).suffix.lower()
                name='assets/'+hashlib.sha256(data).hexdigest()+suffix
                assets[name]=data;node.set(key,name)
    if compiler is not None:
        compiler.attrib.pop('meshdir',None);compiler.attrib.pop('texturedir',None)
        compiler.set('strippath','false')
    return ET.tostring(root,encoding='unicode'),assets


def _state(session):
    cameras=session.scene_description()['cameras']
    for row in cameras:
        row['config']=asdict(row['config'])
        row['edited']=row['name'] in session._camera_edits
        row['revision']=session._revisions[row['name']]
    return dict(body_edits={n:[p.tolist(),q.tolist()] for n,(p,q) in session._body_edits.items()},
                body_names=list(session._body_names.values()),
                initial_qpos=session._initial_qpos.tolist(),
                mocap_pos=session._initial_mocap_pos.tolist(),mocap_quat=session._initial_mocap_quat.tolist(),
                cameras=cameras,model_revision=session.model_revision,
                overview=dict(lookat=session.view.lookat.tolist(),distance=session.view.distance,
                              azimuth=session.view.azimuth,elevation=session.view.elevation),
                frustums=session.show_frustums)


def save_project(session,path,layout=None):
    session._check()
    if session.project is None:raise ValueError('Saving requires a SceneProject session')
    path=Path(path).expanduser().resolve()
    path.parent.mkdir(parents=True,exist_ok=True)
    xml,assets=_scene_assets(session.project.spec)
    metadata=dict(schema_version=SCHEMA,state=_state(session),layout=layout or {},
                  cameras=list(session.cameras),next_instance=session.project.next_instance,
                  demo_motion=session.project.demo_motion)
    fd,temp=tempfile.mkstemp(prefix=path.name+'.',suffix='.tmp',dir=path.parent)
    try:
        with os.fdopen(fd,'wb') as stream:
            with zipfile.ZipFile(stream,'w',compression=zipfile.ZIP_DEFLATED) as archive:
                archive.writestr('project.json',json.dumps(metadata,ensure_ascii=False,allow_nan=False))
                archive.writestr('scene.xml',xml)
                for name,data in assets.items():archive.writestr(name,data)
            stream.flush();os.fsync(stream.fileno())
        load_project(temp)
        os.replace(temp,path)
    finally:
        if os.path.exists(temp):os.unlink(temp)


def load_project(path,revision=None):
    with zipfile.ZipFile(path) as archive:
        metadata=json.loads(archive.read('project.json'))
        if metadata.get('schema_version')!=SCHEMA:raise ValueError('Unsupported project schema_version')
        xml=archive.read('scene.xml').decode('utf-8')
        assets={n:archive.read(n) for n in archive.namelist() if n.startswith('assets/')}
    # Archive resources are resolved only from the in-memory bundle.
    root=ET.fromstring(xml)
    if root.findall('.//include'):raise ValueError('Project scene must be normalized without includes')
    for node in root.iter():
        for key in FILE_KEYS:
            if node.get(key) and node.get(key) not in assets:raise ValueError('Project references missing bundled resource')
    spec=mujoco.MjSpec.from_string(xml,assets=assets)
    model=spec.compile()
    state=metadata['state']
    names=[mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,i) for i in range(1,model.nbody)]
    if names!=state['body_names']:raise ValueError('Project body identities do not match scene')
    for key,shape in (('initial_qpos',(model.nq,)),('mocap_pos',(model.nmocap,3)),('mocap_quat',(model.nmocap,4))):
        array=np.asarray(state[key],dtype=float)
        if array.size==0:array=array.reshape(shape)
        if array.shape!=shape or not np.isfinite(array).all():raise ValueError('Invalid project '+key)
    cameras=tuple(metadata['cameras'])
    if not 1<=len(cameras)<=3 or len(set(cameras))!=len(cameras):raise ValueError('Invalid project cameras')
    for camera in cameras:
        if mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_CAMERA,camera)<0:raise ValueError('Missing project camera')
    if [row['name'] for row in state['cameras']]!=list(cameras):raise ValueError('Camera state does not match selection')
    for name,(position,rotation) in state['body_edits'].items():
        if name not in names:raise ValueError('Unknown edited body')
        pose(position,rotation)
    for row in state['cameras']:
        CameraConfig(**row['config'])
        if row['parent'] not in ['world',*names]:raise ValueError('Unknown camera parent')
        pose(row['position'],row['quaternion'])
        if type(row['revision']) is not int or row['revision']<0:raise ValueError('Invalid camera revision')
        if type(row['edited']) is not bool:raise ValueError('Invalid camera edit flag')
    overview=state['overview']
    lookat=np.asarray(overview['lookat'],dtype=float)
    if lookat.shape!=(3,) or not np.isfinite(lookat).all():raise ValueError('Invalid overview target')
    if not np.isfinite([overview['distance'],overview['azimuth'],overview['elevation']]).all() or overview['distance']<=0:
        raise ValueError('Invalid overview parameters')
    if type(state['model_revision']) is not int or state['model_revision']<0:raise ValueError('Invalid model revision')
    if type(metadata['next_instance']) is not int or metadata['next_instance']<1:raise ValueError('Invalid instance counter')
    if type(metadata['demo_motion']) is not bool or type(state['frustums']) is not bool:raise ValueError('Invalid project flags')
    layout=metadata.get('layout',{})
    if not isinstance(layout,dict):raise ValueError('Invalid project layout')
    for key in ('geometry','docks'):
        if key in layout and not isinstance(layout[key],str):raise ValueError('Invalid layout '+key)
    if 'floating' in layout and (not isinstance(layout['floating'],list) or len(layout['floating'])!=4 or
                                any(v is not None and not isinstance(v,str) for v in layout['floating'])):
        raise ValueError('Invalid floating window layout')
    version=state['model_revision'] if revision is None else revision
    project=SceneProject(spec,model,cameras,version,source=str(path),demo_motion=metadata['demo_motion'],
                         next_instance=metadata['next_instance'],saved_state=state)
    return project,metadata.get('layout',{})


def restore_state(session,state):
    for name,(p,q) in state['body_edits'].items():session.edit_body(name,p,q)
    session._initial_qpos=np.array(state['initial_qpos'],dtype=float)
    session._initial_mocap_pos=np.array(state['mocap_pos'],dtype=float).reshape((-1,3))
    session._initial_mocap_quat=np.array(state['mocap_quat'],dtype=float).reshape((-1,4))
    session.reset()
    for row in state['cameras']:
        config=CameraConfig(**row['config'])
        if row['edited']:session.edit_camera(row['name'],row['parent'],row['position'],row['quaternion'],config)
        else:session.configure_camera(row['name'],config)
        session._revisions[row['name']]=row['revision']
    for name,value in state['overview'].items():
        if name=='lookat':session.view.lookat[:]=value
        else:setattr(session.view,name,value)
    session.show_frustums=state['frustums']
    session.model_revision=session.project.revision
