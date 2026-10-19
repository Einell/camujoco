from queue import Empty
from threading import Event
from time import monotonic, sleep
from types import SimpleNamespace
import numpy as np
import pytest
from camera_lab.api.streams import DataAPI
from camera_lab.contracts import FrameStamp, PoseEstimate


def wait_for(predicate):
    until=monotonic()+3
    while monotonic()<until:
        if predicate(): return
        sleep(.005)
    raise AssertionError('module did not finish')


def sample(api, batch=1):
    stamp=FrameStamp('test',1,batch,0,0.,0)
    obs=SimpleNamespace(stamp=stamp,frames=(SimpleNamespace(camera_id='front',T_world_camera=np.eye(4)),))
    api.publish(obs,SimpleNamespace(stamp=stamp,object_poses={'box':np.eye(4)}))
    return obs


class Estimator:
    def start(self, cancel): self.cancel=cancel
    def estimate(self, observation):
        return [PoseEstimate(observation.stamp,'front','box',np.eye(4))]
    def close(self): self.closed=True


def test_module_runs_evaluates_and_restarts():
    from camera_lab.modules import ModuleManager
    api=DataAPI();manager=ModuleManager(api);instances=[]
    def factory():
        obj=Estimator();instances.append(obj);return obj
    manager.register('test',factory)
    results=manager.subscribe_results()
    manager.start('test');sample(api)
    wait_for(lambda:manager.status('test').processed==1)
    result=results.get_nowait()
    assert result.errors[0].translation_m==0
    assert result.estimates[0].stamp==result.stamp
    assert manager.stop('test',timeout=1)
    assert instances[0].closed
    manager.start('test');sample(api,2)
    wait_for(lambda:manager.status('test').processed==1)
    manager.close()
    assert len(instances)==2
    with pytest.raises(RuntimeError):manager.start('test')


def test_bad_stamp_fails_only_algorithm():
    from camera_lab.modules import ModuleManager
    class Bad(Estimator):
        def estimate(self, obs):
            raise ValueError('bad input')
    api=DataAPI();manager=ModuleManager(api);manager.register('bad',Bad)
    manager.start('bad');sample(api)
    wait_for(lambda:manager.status('bad').state=='failed')
    assert 'bad input' in manager.status('bad').error
    assert not api.closed
    manager.close()


def test_slow_algorithm_is_bounded_and_cooperatively_stopped():
    from camera_lab.modules import ModuleManager
    entered=Event()
    class Slow(Estimator):
        def estimate(self, obs):
            entered.set();self.cancel.wait(2)
            return super().estimate(obs)
    api=DataAPI();manager=ModuleManager(api);manager.register('slow',Slow)
    results=manager.subscribe_results();manager.start('slow');sample(api)
    assert entered.wait(1)
    for i in range(2,20):sample(api,i)
    assert manager.status('slow').dropped>=15
    assert manager.stop('slow',timeout=1)
    with pytest.raises(Empty):results.get_nowait()
    manager.close()


def test_wrong_frame_is_rejected_and_cleanup_runs():
    from camera_lab.modules import ModuleManager
    obj=Estimator()
    old=FrameStamp('old',1,1,0,0.,0)
    obj.estimate=lambda obs:[PoseEstimate(old,'front','box',np.eye(4))]
    api=DataAPI();manager=ModuleManager(api);manager.register('wrong',lambda:obj)
    results=manager.subscribe_results();manager.start('wrong');sample(api)
    wait_for(lambda:manager.status('wrong').state=='failed')
    assert 'frame stamp' in manager.status('wrong').error
    assert obj.closed
    with pytest.raises(Empty):results.get_nowait()
    manager.close()


def test_inflight_result_after_reset_is_unavailable():
    from camera_lab.modules import ModuleManager
    entered=Event();release=Event()
    class Delayed(Estimator):
        def estimate(self, obs):
            entered.set();release.wait(2)
            return super().estimate(obs)
    api=DataAPI();manager=ModuleManager(api);manager.register('delay',Delayed)
    results=manager.subscribe_results();manager.start('delay');sample(api)
    assert entered.wait(1)
    with api._lock:api._activate(('new',1))
    release.set()
    wait_for(lambda:manager.status('delay').processed==1)
    assert results.get_nowait().errors[0].status=='unavailable'
    manager.close()


def test_close_reports_pending_cleanup_and_can_be_polled():
    from camera_lab.modules import ModuleManager
    cleaning=Event();release=Event()
    class Cleanup(Estimator):
        def close(self):cleaning.set();release.wait(2)
    api=DataAPI();manager=ModuleManager(api);manager.register('cleanup',Cleanup)
    manager.start('cleanup')
    wait_for(lambda:manager.status('cleanup').state=='running')
    assert not manager.close(timeout=0)
    assert cleaning.wait(1)
    assert manager.status('cleanup').state=='stopping'
    release.set()
    assert manager.close(timeout=1)
    assert manager.status('cleanup').state=='stopped'


def test_source_close_cancels_active_algorithm():
    from camera_lab.modules import ModuleManager
    entered=Event()
    class Waiting(Estimator):
        def estimate(self, obs):
            entered.set();self.cancel.wait(2)
            return ()
    api=DataAPI();manager=ModuleManager(api);manager.register('waiting',Waiting)
    manager.start('waiting');sample(api)
    assert entered.wait(1)
    api.close()
    until=monotonic()+.5
    while manager.status('waiting').state!='stopped' and monotonic()<until:sleep(.005)
    try:assert manager.status('waiting').state=='stopped'
    finally:manager.close()
