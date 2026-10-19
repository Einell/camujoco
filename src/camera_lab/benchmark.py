"""Headless rendering benchmark. This does not measure desktop presentation."""
import argparse
import json
from pathlib import Path
from time import perf_counter, process_time, sleep
from .render.worker import RenderSession
from .metrics import summarize, environment, gpu_sample


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--cameras', type=int, choices=(1, 2, 3), default=3)
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--warmup', type=float, default=10)
    parser.add_argument('--fps', type=float, default=30)
    parser.add_argument('--report', type=Path, default=Path('reports/headless.json'))
    args = parser.parse_args()
    if args.seconds <= 0 or args.warmup < 0 or args.fps <= 0:
        parser.error('seconds and fps must be positive; warmup must be nonnegative')
    with RenderSession(args.width, args.height, args.cameras) as session:
        deadline = perf_counter() + args.warmup
        while perf_counter() < deadline:
            session.advance_to(session.data.time + 1 / args.fps)
            session.capture()
        session.reset()
        latencies, samples = [], []
        cpu_start = process_time()
        start = perf_counter()
        next_sample = start
        while perf_counter() - start < args.seconds:
            session.advance_to(perf_counter() - start)
            batch = session.capture()
            latencies.append(batch.render_ms)
            now = perf_counter()
            if now >= next_sample:
                samples.append({'wall_seconds': now - start, **gpu_sample()})
                next_sample = now + 5
            sleep(max(0, start + len(latencies) / args.fps - perf_counter()))
        elapsed = perf_counter() - start
        report = {**environment(), **summarize(latencies, elapsed, session.data.time),
                  'mode': 'headless_with_overview_render', 'desktop_visible': False,
                  'gl_renderer': session.gl_renderer, 'width': args.width, 'height': args.height,
                  'cameras': args.cameras, 'rgb_and_depth': True, 'target_fps': args.fps,
                  'warmup_seconds': args.warmup,
                  'process_cpu_percent': 100 * (process_time() - cpu_start) / elapsed,
                  'gpu_samples': samples}
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
