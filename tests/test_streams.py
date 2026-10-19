import pytest
from camera_lab.render.worker import RenderSession


def test_slow_subscriber_drops_whole_batches_without_blocking():
    from camera_lab.api.streams import DataAPI
    api=DataAPI()
    images=api.subscribe_observations();truth=api.subscribe_truth()
    with RenderSession(width=80,height=60) as s:
        for i in range(5):
            s.single_step();api.publish(*s.capture_sample())
        assert images.dropped==3 and truth.dropped==3
        assert images.get_nowait().stamp.batch_id==3
        observed=images.get_nowait();actual=truth.get_nowait();actual=truth.get_nowait()
        assert observed.stamp==actual.stamp
        assert observed.stamp.step_index==5
        assert not hasattr(observed,'object_poses')
        images.close();api.publish(*s.capture_sample())
        assert images.closed


def test_new_run_discards_queued_old_frames_and_history():
    from camera_lab.api.streams import DataAPI
    api=DataAPI();sub=api.subscribe_observations()
    with RenderSession(width=80,height=60) as s:
        api.publish(*s.capture_sample());old=s.run_id
        s.reset();api.publish(*s.capture_sample())
        assert sub.get_nowait().stamp.run_id!=old
