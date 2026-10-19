"""Owner-thread command scheduling in native MuJoCo actuator ctrl units."""
from time import monotonic_ns
import mujoco
import numpy as np
from ..contracts import ControlCommand


class ControlScheduler:
    def __init__(self,session):
        self.session=session
        model=session.model
        self.names={mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_ACTUATOR,i):i
                    for i in range(model.nu) if mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_ACTUATOR,i)}
        self.defaults=np.zeros(model.nu)
        for i in range(model.nu):
            if model.actuator_ctrllimited[i]:self.defaults[i]=np.clip(0,*model.actuator_ctrlrange[i])
        self.enabled=False
        self.pending=[]
        self.active={}
        self.version=(session.run_id,session.model_revision)
        self.sequence=0

    def clear(self):
        self.pending.clear();self.active.clear()
        self.version=(self.session.run_id,self.session.model_revision)
        if self.enabled:self.session.data.ctrl[:]=self.defaults

    def submit(self,command):
        s=self.session;s._check()
        if not isinstance(command,ControlCommand):raise ValueError('Expected ControlCommand')
        if (command.run_id,command.model_revision)!=(s.run_id,s.model_revision):raise ValueError('Control version mismatch')
        if command.target_step<s.step_index or command.valid_until_step<s.step_index or command.expires_at_ns<=monotonic_ns():
            raise ValueError('Control expired or target step already passed')
        for name,value in command.values.items():
            if name not in self.names:raise ValueError(f'Unknown or unsupported actuator: {name}')
            i=self.names[name]
            if s.model.actuator_ctrllimited[i] and not s.model.actuator_ctrlrange[i,0]<=value<=s.model.actuator_ctrlrange[i,1]:
                raise ValueError(f'Control outside ctrlrange: {name}')
        if self.version!=(s.run_id,s.model_revision):self.clear()
        if len(self.pending)>=64:raise ValueError('Scheduled control queue full')
        self.enabled=True
        self.pending.append((command.target_step,self.sequence,command));self.sequence+=1
        self.pending.sort(key=lambda item:item[:2])

    def apply(self):
        s=self.session;s._check()
        if self.version!=(s.run_id,s.model_revision):self.clear()
        if not self.enabled:return False
        now=monotonic_ns()
        remaining=[]
        for target,sequence,command in self.pending:
            if command.expires_at_ns<=now or command.valid_until_step<s.step_index:continue
            if target<=s.step_index:
                for name,value in command.values.items():self.active[name]=(value,command.valid_until_step,command.expires_at_ns)
            else:remaining.append((target,sequence,command))
        self.pending=remaining
        s.data.ctrl[:]=self.defaults
        for name,(value,last_step,deadline) in list(self.active.items()):
            if s.step_index<=last_step and now<deadline:s.data.ctrl[self.names[name]]=value
            else:del self.active[name]
        return True


def describe_actuators(session):
    from ..contracts import ActuatorInfo
    result=[]
    for i in range(session.model.nu):
        bounds=tuple(float(v) for v in session.model.actuator_ctrlrange[i]) if session.model.actuator_ctrllimited[i] else None
        result.append(ActuatorInfo(mujoco.mj_id2name(session.model,mujoco.mjtObj.mjOBJ_ACTUATOR,i),bounds,
                                   float(np.clip(0,*bounds)) if bounds else 0.0))
    return tuple(result)
