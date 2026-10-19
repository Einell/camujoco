"""Thread-owned editing operations; camera poses use optical axes."""
import mujoco
import numpy as np
from ..coordinates import camera_world_pose


def pose(position, quaternion):
    p=np.asarray(position,dtype=float); q=np.asarray(quaternion,dtype=float)
    if p.shape!=(3,) or q.shape!=(4,) or not np.isfinite(p).all() or not np.isfinite(q).all() or np.linalg.norm(q)<1e-12:
        raise ValueError('Require finite XYZ and nonzero wxyz quaternion')
    return p,q/np.linalg.norm(q)


def matrix(q):
    r=np.empty(9);mujoco.mju_quat2Mat(r,q);return r.reshape(3,3)


def quaternion(r):
    q=np.empty(4);mujoco.mju_mat2Quat(q,np.ascontiguousarray(r).ravel());return q


class Editing:
    def scene_description(self):
        self._check()
        bodies=[]
        for bid,name in self._body_names.items():
            joint=int(self.model.body_jntadr[bid])
            free=self.model.body_jntnum[bid]>0 and self.model.jnt_type[joint]==mujoco.mjtJoint.mjJNT_FREE
            adr=int(self.model.jnt_qposadr[joint]) if free else 0
            p=self.data.qpos[adr:adr+3] if free else self.model.body_pos[bid]
            q=self.data.qpos[adr+3:adr+7] if free else self.model.body_quat[bid]
            mocap=int(self.model.body_mocapid[bid])
            if mocap>=0:
                p=self.data.mocap_pos[mocap];q=self.data.mocap_quat[mocap]
            bodies.append(dict(name=name,parent=self._body_names.get(int(self.model.body_parentid[bid]),'world'),position=p.tolist(),quaternion=q.tolist(),free=bool(free)))
        cameras=[]
        for name,cid in self._camera_ids.items():
            optical=matrix(self.model.cam_quat[cid]) @ np.diag([1,-1,-1])
            cameras.append(dict(name=name,parent=self._body_names.get(int(self.model.cam_bodyid[cid]),'world'),position=self.model.cam_pos[cid].tolist(),quaternion=quaternion(optical).tolist(),config=self.camera_config(name)))
        return dict(bodies=bodies,cameras=cameras)

    def edit_body(self,name,position,rotation):
        self._check()
        p,q=pose(position,rotation)
        bid=next((bid for bid,n in self._body_names.items() if n==name),None)
        if bid is None: raise ValueError(f'Unknown body: {name}')
        joint=int(self.model.body_jntadr[bid])
        if self.model.body_jntnum[bid] and self.model.jnt_type[joint]==mujoco.mjtJoint.mjJNT_FREE:
            adr=int(self.model.jnt_qposadr[joint]);self.model.qpos0[adr:adr+7]=np.r_[p,q]
            self._initial_qpos[adr:adr+7]=np.r_[p,q]
        else:
            self.model.body_pos[bid]=p;self.model.body_quat[bid]=q
        mocap=int(self.model.body_mocapid[bid])
        if mocap>=0:
            self._initial_mocap_pos[mocap]=p
            self._initial_mocap_quat[mocap]=q
        mujoco.mj_setConst(self.model,self.data)
        self.model_revision+=1
        self.reset()
        self._body_edits[name]=(p.copy(),q.copy())

    def edit_camera(self,name,parent,position,rotation,config):
        self._check()
        p,q=pose(position,rotation)
        if name not in self._camera_ids: raise ValueError(f'Unknown camera: {name}')
        bid=0 if parent=='world' else next((bid for bid,n in self._body_names.items() if n==parent),None)
        if bid is None: raise ValueError(f'Unknown parent: {parent}')
        self.configure_camera(name,config)
        cid=self._camera_ids[name]
        self.model.cam_bodyid[cid]=bid
        self.model.cam_mode[cid]=mujoco.mjtCamLight.mjCAMLIGHT_FIXED
        self.model.cam_pos[cid]=p
        self.model.cam_quat[cid]=quaternion(matrix(q) @ np.diag([1,-1,-1]))
        mujoco.mj_forward(self.model,self.data)
        self._camera_edits.add(name)

    def aim_camera(self,name,target):
        self._check()
        target=np.asarray(target,dtype=float)
        cid=self._camera_ids[name]
        p=self.data.cam_xpos[cid].copy()
        if target.shape!=(3,) or not np.isfinite(target).all() or np.linalg.norm(target-p)<1e-9:
            raise ValueError('Target must be finite and distinct from camera position')
        z=(target-p)/np.linalg.norm(target-p)
        up=np.array([0.,0.,1.]) if abs(z[2])<.99 else np.array([0.,1.,0.])
        x=np.cross(z,up);x/=np.linalg.norm(x)
        r=np.column_stack((x,np.cross(z,x),z))
        bid=int(self.model.cam_bodyid[cid]);parent=self._body_names.get(bid,'world')
        parent_r=self.data.xmat[bid].reshape(3,3)
        self.edit_camera(name,parent,self.model.cam_pos[cid],quaternion(parent_r.T @ r),self.camera_config(name))

    def move_camera(self,name,dx,dy,steps=0):
        cid=self._camera_ids[name]
        optical=matrix(self.model.cam_quat[cid]) @ np.diag([1,-1,-1])
        p=self.model.cam_pos[cid]+optical @ np.array([dx*.005,dy*.005,steps*.1])
        self.edit_camera(name,self._body_names.get(int(self.model.cam_bodyid[cid]),'world'),p,quaternion(optical),self.camera_config(name))

    def copy_edits_to(self,other):
        for name,(p,q) in self._body_edits.items(): other.edit_body(name,p,q)
        other_names={name:bid for bid,name in other._body_names.items()}
        for bid,name in self._body_names.items():
            dest=other_names[name]
            for offset in range(int(self.model.body_jntnum[bid])):
                j=int(self.model.body_jntadr[bid])+offset
                k=int(other.model.body_jntadr[dest])+offset
                size={0:7,1:4,2:1,3:1}[int(self.model.jnt_type[j])]
                a=int(self.model.jnt_qposadr[j]);b=int(other.model.jnt_qposadr[k])
                other._initial_qpos[b:b+size]=self._initial_qpos[a:a+size]
            a=int(self.model.body_mocapid[bid]);b=int(other.model.body_mocapid[dest])
            if a>=0 and b>=0:
                other._initial_mocap_pos[b]=self._initial_mocap_pos[a]
                other._initial_mocap_quat[b]=self._initial_mocap_quat[a]
        other.reset()
        for c in self.scene_description()['cameras']:
            if c['name'] in self._camera_edits:
                other.edit_camera(c['name'],c['parent'],c['position'],c['quaternion'],c['config'])
            else:
                other.configure_camera(c['name'],c['config'])

    def draw_frustums(self):
        scene=self.renderer.scene
        colors=((1,.4,.2,1),(.2,1,.5,1),(.3,.6,1,1))
        for (name,cid),color in zip(self._camera_ids.items(),colors):
            c=self.camera_config(name)
            length=min(max(c.near*2,self.model.stat.extent*.5),c.far)
            t=camera_world_pose(self.data.cam_xpos[cid],self.data.cam_xmat[cid])
            corners=[t[:3,:3] @ np.array([(u-c.cx)/c.fx*length,(v-c.cy)/c.fy*length,length])+t[:3,3]
                     for u,v in ((0,0),(c.width-1,0),(c.width-1,c.height-1),(0,c.height-1))]
            lines=[(t[:3,3],p) for p in corners]+[(corners[i],corners[(i+1)%4]) for i in range(4)]
            for a,b in lines:
                if scene.ngeom>=scene.maxgeom:return
                geom=scene.geoms[scene.ngeom]
                mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_LINE,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.array(color,dtype=np.float32))
                mujoco.mjv_connector(geom,mujoco.mjtGeom.mjGEOM_LINE,2,a,b)
                scene.ngeom+=1
