"""Compile candidate scenes before changing a running application's scene."""
from dataclasses import dataclass
from importlib.resources import files
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from .assets import resolve_resource


class ImportFailure(ValueError):
    pass


def load_spec(path, packages):
    path=Path(path).expanduser().resolve()
    warnings=[]
    root=ET.parse(path).getroot()
    if root.tag=='robot':
        if any('xacro' in node.tag or '${' in str(node.attrib) for node in root.iter()):
            raise ValueError('Expand Xacro to URDF before importing')
        extension=root.find('mujoco')
        if extension is None: extension=ET.SubElement(root,'mujoco')
        compiler=extension.find('compiler')
        if compiler is None: compiler=ET.SubElement(extension,'compiler')
        compiler.set('discardvisual','false')
        compiler.set('fusestatic','false')
        compiler.set('strippath','false')
        meshbase=path.parent/Path(compiler.get('meshdir',''))
        for node in root.iter():
            if node.tag in ('mesh','texture') and node.get('filename'):
                node.set('filename',resolve_resource(node.get('filename'),meshbase,packages))
        if root.findall('transmission') or root.findall('gazebo'):
            warnings.append('URDF transmission/Gazebo extensions do not create MuJoCo controllers.')
        spec=mujoco.MjSpec.from_string(ET.tostring(root,encoding='unicode'))
        spec.modelfiledir=str(path.parent)+'/'
        warnings.append('URDF visuals and fixed links retained; no actuators generated automatically.')
    elif root.tag=='mujoco':
        spec=mujoco.MjSpec.from_file(str(path))
        for collection in (spec.meshes,spec.textures,spec.hfields,spec.skins):
            for asset in collection:
                value=getattr(asset,'file','')
                if '://' in value:
                    asset.file=resolve_resource(value,path.parent,packages)
    else:
        raise ValueError(f'Expected <mujoco> or <robot>, got <{root.tag}>')
    # Preserve stable body identities while combining multiple models.
    spec.compiler.fusestatic=False
    names={body.name for body in spec.bodies if body.name}
    for index,body in enumerate(spec.bodies):
        if body is spec.worldbody or body.name: continue
        name=f'body_{index}'
        while name in names: name+='_'
        body.name=name; names.add(name)
    spec.compile()
    return spec,warnings


def prepare_views(spec):
    model=spec.compile()
    selected=[cam.name for cam in spec.cameras if cam.proj==mujoco.mjtProjection.mjPROJ_PERSPECTIVE and cam.name][:3]
    center=np.array(model.stat.center)
    radius=max(float(model.stat.extent),.4)
    directions=((0,-1,.5),(1,-.2,.4),(0,0,1))
    for index in range(3-len(selected)):
        name=f'__camera_lab_{index}'
        while spec.camera(name) is not None: name+='_'
        direction=np.array(directions[index],dtype=float)
        direction/=np.linalg.norm(direction)
        pos=center+direction*radius*1.8
        up=np.array([0.,0.,1.]) if index!=2 else np.array([0.,1.,0.])
        right=np.cross(up,direction);right/=np.linalg.norm(right)
        rotation=np.column_stack([right,np.cross(direction,right),direction])
        quat=np.empty(4);mujoco.mju_mat2Quat(quat,rotation.ravel())
        spec.worldbody.add_camera(name=name,pos=pos,quat=quat,fovy=45)
        selected.append(name)
    if not spec.lights:
        spec.worldbody.add_light(pos=center+[0,-radius,2*radius],dir=[0,.3,-1],diffuse=[.8,.8,.8])
    return tuple(selected)


@dataclass
class SceneProject:
    spec: object
    model: object
    cameras: tuple[str,...]
    revision: int=0
    warnings: tuple[str,...]=()
    source: str=''
    demo_motion: bool=False
    next_instance: int=1

    @classmethod
    def demo(cls):
        spec=mujoco.MjSpec.from_string(files('camera_lab').joinpath('demo.xml').read_text())
        return cls(spec,spec.compile(),('front','side','top'),demo_motion=True)

    @classmethod
    def open(cls,path,packages=None,revision=0):
        try:
            spec,warnings=load_spec(path,packages or {})
            cameras=prepare_views(spec)
            if len(spec.cameras)>3:
                warnings.append('Only the first three perspective cameras are previewed; all cameras remain in the model.')
            return cls(spec,spec.compile(),cameras,revision,tuple(warnings),str(Path(path).resolve()))
        except Exception as exc:
            raise ImportFailure(f'{path}: {exc}') from exc

    def append(self,path,position=(0,0,0),quaternion=(1,0,0,0),packages=None):
        try:
            position=np.asarray(position,dtype=float);quaternion=np.asarray(quaternion,dtype=float)
            if position.shape!=(3,) or quaternion.shape!=(4,) or not np.isfinite(position).all() or not np.isfinite(quaternion).all() or np.linalg.norm(quaternion)<1e-12:
                raise ValueError('Position must be 3 finite numbers and quaternion must be nonzero wxyz')
            quaternion=quaternion/np.linalg.norm(quaternion)
            child,warnings=load_spec(path,packages or {})
            candidate=self.spec.copy()
            number=self.next_instance
            # Names are namespaced across bodies, joints, assets and references by MjSpec.
            collections=('bodies','joints','geoms','cameras','meshes','textures','materials','actuators','sensors','sites','frames','tendons','keys','equalities','lights')
            names=[element.name for name in collections for element in getattr(candidate,name)]
            prefix=f'model_{number:04d}/'
            while any(name.startswith(prefix) for name in names):
                number+=1;prefix=f'model_{number:04d}/'
            warnings.append('Append preserves current global physics and rendering settings.')
            for key in dir(child.option):
                if key.startswith('_') or callable(getattr(child.option,key)):
                    continue
                if not np.array_equal(getattr(candidate.option,key),getattr(child.option,key)):
                    warnings.append(f'Append keeps current global {key}; imported setting is ignored.')
            frame=candidate.worldbody.add_frame(pos=position,quat=quaternion)
            candidate.copy_during_attach=True
            candidate.attach(child,prefix=prefix,frame=frame)
            compiled=candidate.compile()
            return SceneProject(candidate,compiled,self.cameras,self.revision+1,
                                tuple(warnings),self.source,self.demo_motion,number+1)
        except Exception as exc:
            raise ImportFailure(f'{path}: {exc}') from exc
