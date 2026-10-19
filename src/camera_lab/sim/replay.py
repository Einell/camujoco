"""Validated kinematic replay; intentionally not a dynamics controller."""
from uuid import uuid4
import mujoco
import numpy as np


def state(session):
    session._check();model=session.model
    sizes={0:7,1:4,2:1,3:1}
    joints=[]
    for j in range(model.njnt):
        kind=int(model.jnt_type[j])
        joints.append(dict(name=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_JOINT,j),
            type=kind,qpos_address=int(model.jnt_qposadr[j]),size=sizes[kind],
            limited=bool(model.jnt_limited[j]),range=model.jnt_range[j].tolist()))
    return dict(qpos=session.data.qpos.copy(),qvel=session.data.qvel.copy(),
                time_s=float(session.data.time),joints=joints,
                run_id=session.run_id,model_revision=session.model_revision)


def seek(session, qpos, time_s=0., qvel=None):
    session._check();model=session.model
    q=np.array(qpos,dtype=float,copy=True)
    v=np.zeros(model.nv) if qvel is None else np.array(qvel,dtype=float,copy=True)
    if q.shape!=(model.nq,) or v.shape!=(model.nv,) or not np.isfinite(q).all() or not np.isfinite(v).all():
        raise ValueError('Replay requires finite full qpos and qvel arrays matching the model')
    if not np.isfinite(time_s) or time_s<0:raise ValueError('time_s must be finite and nonnegative')
    for j in range(model.njnt):
        kind=int(model.jnt_type[j]);adr=int(model.jnt_qposadr[j])
        if kind in (0,1):
            quat=q[adr+3:adr+7] if kind==0 else q[adr:adr+4]
            if not np.isclose(np.linalg.norm(quat),1,atol=1e-6):
                raise ValueError('Replay requires unit quaternions (wxyz)')
            if kind==1 and model.jnt_limited[j]:
                angle=2*np.arccos(np.clip(abs(quat[0]),0,1))
                if angle>model.jnt_range[j,1]+1e-9:raise ValueError(f'Ball joint {j} outside model limits')
        elif model.jnt_limited[j] and not model.jnt_range[j,0]-1e-9<=q[adr]<=model.jnt_range[j,1]+1e-9:
            raise ValueError(f'Joint {j} outside model limits')
    # Validate everything before mutating the live state. A new run invalidates controls.
    if session._controls is not None:session._controls.clear()
    mujoco.mj_resetData(model,session.data)
    session.data.qpos[:]=q;session.data.qvel[:]=v;session.data.time=float(time_s)
    session.data.mocap_pos[:]=session._initial_mocap_pos
    session.data.mocap_quat[:]=session._initial_mocap_quat
    session.run_id=str(uuid4());session.sequence=0;session.step_index=0
    mujoco.mj_forward(model,session.data)
