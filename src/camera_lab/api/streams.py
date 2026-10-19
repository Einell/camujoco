"""Bounded fanout without running consumer callbacks on the simulation thread."""
from collections import deque,OrderedDict
from queue import Queue,Empty
from threading import Lock
import numpy as np
from ..contracts import PoseEstimate,PoseError,ControlCommand,ControlResult


class Subscription:
    def __init__(self, cancel=None):
        self._cancel=cancel
        self._queue=deque(maxlen=2);self._lock=Lock();self.dropped=0;self.closed=False

    def put(self,value):
        with self._lock:
            if self.closed:return
            if len(self._queue)==2:self.dropped+=1
            self._queue.append(value)

    def get_nowait(self):
        with self._lock:
            if not self._queue:raise Empty
            return self._queue.popleft()

    def clear(self):
        with self._lock:self._queue.clear()

    def close(self):
        with self._lock:
            self.closed=True;self._queue.clear()
            if self._cancel is not None:self._cancel.set()


class DataAPI:
    def __init__(self,history_size=120):
        if type(history_size) is not int or history_size<1:raise ValueError('Positive history_size required')
        self.history_size=history_size
        self._lock=Lock();self._history=OrderedDict();self._version=None
        self._subscribers={'observations':[],'truth':[],'controls':[]}
        self._commands=Queue(maxsize=64)
        self.closed=False
        self._actuators=()

    @property
    def actuators(self):
        with self._lock:return self._actuators

    def bind(self,session):
        session._check()
        from .control import describe_actuators
        with self._lock:
            version=(session.run_id,session.model_revision)
            if version!=self._version:
                self._activate(version)
                self._actuators=describe_actuators(session)

    def _activate(self,version):
        self._version=version;self._history.clear()
        for kind in ('observations','truth'):
            for sub in self._subscribers[kind]:sub.clear()

    def _subscribe(self,kind,cancel=None):
        with self._lock:
            if self.closed:raise RuntimeError('DataAPI closed')
            self._subscribers[kind]=[s for s in self._subscribers[kind] if not s.closed]
            if len(self._subscribers[kind])>=64:raise ValueError('Too many subscribers')
            sub=Subscription(cancel);self._subscribers[kind].append(sub);return sub

    def subscribe_observations(self, *, cancel=None):return self._subscribe('observations',cancel)
    def subscribe_truth(self):return self._subscribe('truth')
    def subscribe_control_results(self):return self._subscribe('controls')

    def _publish(self,kind,value):
        self._subscribers[kind]=[s for s in self._subscribers[kind] if not s.closed]
        for sub in self._subscribers[kind]:sub.put(value)

    def publish(self,observation,truth):
        if observation.stamp!=truth.stamp:raise ValueError('Observation and truth stamps must match')
        stamp=observation.stamp
        with self._lock:
            if self.closed:return
            version=(stamp.run_id,stamp.model_revision)
            if version!=self._version:
                self._activate(version)
            self._history[stamp]=({f.camera_id:f.T_world_camera for f in observation.frames},truth.object_poses)
            while len(self._history)>self.history_size:self._history.popitem(last=False)
            self._publish('observations',observation);self._publish('truth',truth)

    def evaluate(self,estimate):
        if not isinstance(estimate,PoseEstimate):raise ValueError('Expected PoseEstimate')
        with self._lock:sample=self._history.get(estimate.stamp)
        result=dict(stamp=estimate.stamp,camera_id=estimate.camera_id,object_id=estimate.object_id)
        if sample is None:return PoseError(**result,status='unavailable')
        cameras,bodies=sample
        if estimate.camera_id not in cameras or estimate.object_id not in bodies:return PoseError(**result,status='unknown_target')
        actual=np.linalg.inv(cameras[estimate.camera_id]) @ bodies[estimate.object_id]
        delta=estimate.T_camera_object
        translation=float(np.linalg.norm(delta[:3,3]-actual[:3,3]))
        angle=float(np.arccos(np.clip((np.trace(delta[:3,:3].T @ actual[:3,:3])-1)/2,-1,1)))
        return PoseError(**result,status='matched',translation_m=translation,rotation_rad=angle)

    def submit_control(self,command):
        if not isinstance(command,ControlCommand):raise ValueError('Expected ControlCommand')
        with self._lock:
            if self.closed:raise RuntimeError('DataAPI closed')
            self._commands.put_nowait(command)

    def drain_controls(self,session):
        for _ in range(64):
            try:command=self._commands.get_nowait()
            except Empty:break
            try:
                session.submit_control(command)
                result=ControlResult(command,True,'Scheduled; acceptance does not guarantee execution before expiry')
            except ValueError as exc:result=ControlResult(command,False,str(exc))
            with self._lock:self._publish('controls',result)
        if session._controls is not None:session._controls.apply()

    def close(self):
        with self._lock:
            self.closed=True;self._history.clear()
            for group in self._subscribers.values():
                for sub in group:sub.close()
            while True:
                try:self._commands.get_nowait()
                except Empty:break
