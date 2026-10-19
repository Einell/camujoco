from camera_lab.metrics import summarize


def test_metrics_use_wall_time_not_mean_render_reciprocal():
    result = summarize([10, 20, 30], elapsed=2, simulated=.6)
    assert result['per_camera_fps'] == 1.5
    assert result['real_time_factor'] == .3
    assert result['render_p95_ms'] == 29
    assert result['batches'] == 3


def test_render_statistics_include_frames_not_presented_and_cross_reset_progress():
    from camera_lab.metrics import Measurement
    measurement = Measurement()
    measurement.start()
    for _ in range(20):
        measurement.add(80, .03)
        measurement.add(5, .03)
    measurement.stop()
    measurement.add(999, 100)
    result = measurement.report(elapsed=2, presented=20)
    assert result['render_p95_ms'] == 80
    assert result['per_camera_fps'] == 10
    assert result['produced_per_camera_fps'] == 20
    assert abs(result['simulated_seconds'] - 1.2) < 1e-9
