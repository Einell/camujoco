"""Qt-free pose estimator adapters and bounded, cooperative execution.

Factories and lifecycle methods execute on an algorithm thread, never on the
MuJoCo owner thread. Adapters must observe cancellation and release resources.
"""
from dataclasses import dataclass
from queue import Empty
from threading import Event, RLock, Thread
from time import monotonic
from typing import Protocol, Iterable

from .api.streams import Subscription
from .contracts import ObservationBatch, PoseEstimate, PoseError, FrameStamp


class PoseEstimator(Protocol):
    def start(self, cancel: Event) -> None: ...
    def estimate(self, observation: ObservationBatch) -> Iterable[PoseEstimate]: ...
    def close(self) -> None: ...


@dataclass(frozen=True)
class ModuleStatus:
    state: str = 'stopped'
    processed: int = 0
    dropped: int = 0
    latency_ms: float = 0.
    error: str = ''


@dataclass(frozen=True)
class ModuleResult:
    module: str
    stamp: FrameStamp
    estimates: tuple[PoseEstimate, ...]
    errors: tuple[PoseError, ...]
    latency_ms: float


class _Runner(Thread):
    def __init__(self, name, factory, api, publish):
        super().__init__(name=f'estimator-{name}', daemon=True)
        self.module_name=name;self.factory=factory;self.api=api;self.publish=publish
        self.cancel=Event();self.subscription=api.subscribe_observations(cancel=self.cancel)
        self.lock=RLock();self.snapshot=ModuleStatus(state='starting')

    def status(self):
        with self.lock:
            s=self.snapshot
            return ModuleStatus(s.state,s.processed,self.subscription.dropped,s.latency_ms,s.error)

    def run(self):
        adapter=None;error='';processed=0;latency=0.
        try:
            adapter=self.factory();adapter.start(self.cancel)
            with self.lock:self.snapshot=ModuleStatus(state='running')
            while not self.cancel.is_set() and not self.subscription.closed:
                try:observation=self.subscription.get_nowait()
                except Empty:
                    self.cancel.wait(.005);continue
                started=monotonic();estimates=[]
                for estimate in adapter.estimate(observation):
                    if self.cancel.is_set():break
                    if not isinstance(estimate,PoseEstimate) or estimate.stamp!=observation.stamp:
                        raise ValueError('Estimator must return PoseEstimate with the input frame stamp')
                    if estimate.camera_id not in {f.camera_id for f in observation.frames}:
                        raise ValueError('Estimator returned unknown camera')
                    if len(estimates)>=256:raise ValueError('At most 256 estimates per batch')
                    estimates.append(estimate)
                if self.cancel.is_set():break
                latency=(monotonic()-started)*1000
                errors=tuple(self.api.evaluate(e) for e in estimates)
                self.publish(ModuleResult(self.module_name,observation.stamp,tuple(estimates),errors,latency))
                processed+=1
                with self.lock:self.snapshot=ModuleStatus('running',processed,0,latency)
        except Exception as exc:
            error=f'{type(exc).__name__}: {exc}'
        finally:
            self.subscription.close()
            if adapter is not None:
                try:adapter.close()
                except Exception as exc:error += f' close: {type(exc).__name__}: {exc}'
            with self.lock:self.snapshot=ModuleStatus('failed' if error else 'stopped',processed,0,latency,error)


class ModuleManager:
    """Register factories; start/stop independent adapters; subscribe to results.

    Stop is cooperative. A non-responsive adapter remains 'stopping'; Python
    threads cannot be forcibly killed. Use a subprocess adapter for hard limits.
    """
    def __init__(self, api):
        self._api=api;self._lock=RLock();self._factories={};self._runners={}
        self._results=[];self._closed=False

    @property
    def names(self):
        with self._lock:return tuple(self._factories)

    def register(self, name, factory):
        with self._lock:
            if self._closed:raise RuntimeError('ModuleManager closed')
            if not isinstance(name,str) or not name.strip() or not callable(factory):
                raise ValueError('Nonempty name and callable factory required')
            if name in self._factories:raise ValueError('Module already registered')
            self._factories[name]=factory

    def subscribe_results(self):
        with self._lock:
            if self._closed:raise RuntimeError('ModuleManager closed')
            self._results=[s for s in self._results if not s.closed]
            if len(self._results)>=64:raise ValueError('Too many subscribers')
            sub=Subscription();self._results.append(sub);return sub

    def _publish(self, result):
        with self._lock:
            if not self._closed:
                for sub in self._results:sub.put(result)

    def start(self, name):
        with self._lock:
            if self._closed:raise RuntimeError('ModuleManager closed')
            factory=self._factories[name]
            previous=self._runners.get(name)
            if previous is not None and previous.is_alive():raise RuntimeError('Module already active')
            runner=_Runner(name,factory,self._api,self._publish)
            self._runners[name]=runner;runner.start()

    def status(self, name):
        with self._lock:
            if name not in self._factories:raise KeyError(name)
            runner=self._runners.get(name)
            if runner is None:return ModuleStatus()
            s=runner.status()
            if runner.cancel.is_set() and runner.is_alive():
                return ModuleStatus('stopping',s.processed,s.dropped,s.latency_ms,s.error)
            return s

    def stop(self, name, timeout=0.):
        with self._lock:
            if name not in self._factories:raise KeyError(name)
            runner=self._runners.get(name)
            if runner is None:return True
            runner.cancel.set()
        runner.join(timeout)
        return not runner.is_alive()

    def close(self, timeout=2.):
        with self._lock:
            self._closed=True;runners=tuple(self._runners.values())
            for runner in runners:runner.cancel.set()
            for sub in self._results:sub.close()
        deadline=monotonic()+max(0,timeout)
        for runner in runners:runner.join(max(0,deadline-monotonic()))
        return all(not runner.is_alive() for runner in runners)


class FixedPoseEstimator:
    """Interface demo only: constant configured pose, no ground-truth access."""
    def __init__(self, object_id='box', camera_id='front', transform=None):
        import numpy as np
        self.object_id=object_id;self.camera_id=camera_id
        self.transform=np.eye(4) if transform is None else np.array(transform,copy=True)

    def start(self, cancel):self.cancel=cancel

    def estimate(self, observation):
        if self.cancel.is_set():return ()
        if not any(f.camera_id==self.camera_id for f in observation.frames):return ()
        return (PoseEstimate(observation.stamp,self.camera_id,self.object_id,self.transform),)

    def close(self):pass
