"""Real desktop throughput or long-run/reload acceptance; no lowered resolution."""
import argparse
import json
import os
from pathlib import Path
from time import monotonic
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from camera_lab.app import Window
from camera_lab.metrics import environment,gpu_sample


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=1800)
    parser.add_argument('--reload-every',type=float,default=60)
    parser.add_argument('--project',type=Path,default=Path('examples/projects/table-robot.camproj'))
    parser.add_argument('--report',type=Path,default=Path('reports/acceptance-soak.json'))
    parser.add_argument('--benchmark',action='store_true',help='60-second-style throughput for loaded project')
    args=parser.parse_args()
    if args.seconds<=0 or args.reload_every<=0:parser.error('durations must be positive')
    app=QApplication([])
    if args.benchmark:
        window=Window(seconds=args.seconds,warmup=10,report=args.report)
        window.setWindowTitle('Camera Lab — AUTOMATED BENCHMARK (input disabled)')
        window.setEnabled(False)
        state={'loaded':False,'started':False,'error':None}
        deadline=monotonic()+20
        def start_project():
            if not state['started'] and monotonic()>deadline:
                state['error']='Requested project did not load and start within 20 seconds'
                window.close();return
            if not state['loaded']:
                window.request_open_project(args.project);state['loaded']=True
            elif window.last_batch and window.last_batch.stamp.model_revision>=1 and not state['started']:
                window.running.setChecked(True);state['started']=True
            if state['started'] and window.last_batch:
                frames=window.last_batch.frames
                if len(frames)!=3 or any(f.rgb.shape!=(720,1280,3) or f.depth.shape!=(720,1280) for f in frames):
                    state['error']='Loaded project does not produce three 1280x720 RGB/depth cameras'
                    window.close()
        timer=QTimer();timer.timeout.connect(start_project);timer.start(100)
        window.show();app.exec();timer.stop()
        if window.worker.error or state['error']:
            raise RuntimeError(window.worker.error or state['error'])
        if not state['started'] or not args.report.exists():raise RuntimeError('Benchmark incomplete')
        result=json.loads(args.report.read_text())
        result['requested_project']=str(args.project)
        result['acceptance_passed']=bool(result['desktop_visible'] and result['wall_seconds']>=args.seconds and
            result['real_time_factor']>=.95 and result['per_camera_fps']>=29 and
            result['render_p95_ms'] is not None and result['render_p95_ms']<=33.3)
        args.report.write_text(json.dumps(result,indent=2)+'\n')
        if not result['acceptance_passed']:raise RuntimeError('Benchmark target not met; inspect '+str(args.report))
        return
    window=Window(warmup=10)
    window.setWindowTitle('Camera Lab — AUTOMATED 30-MINUTE SOAK (input disabled)')
    window.setEnabled(False)
    start=monotonic();next_sample=0;next_reload=args.reload_every
    samples=[];reopens=0;completed=0;pending_run=None;error=None;last_sequence=None;last_progress=start
    pending_since=None;last_sim=None;last_physics=start
    def tick():
        nonlocal next_sample,next_reload,reopens,completed,pending_run,error,last_sequence,last_progress
        nonlocal pending_since,last_sim,last_physics
        elapsed=monotonic()-start
        if window.worker.error:error=window.worker.error
        batch=window.last_batch
        if batch:
            marker=(batch.stamp.run_id,batch.sequence)
            if marker!=last_sequence:last_progress=monotonic();last_sequence=marker
            sim_marker=(batch.stamp.run_id,batch.sim_time)
            if sim_marker!=last_sim:last_physics=monotonic();last_sim=sim_marker
            if len(batch.frames)!=3 or any(f.rgb.shape!=(720,1280,3) or f.depth.shape!=(720,1280) for f in batch.frames):
                error='Output dimensions changed: '+str([(f.camera_id,f.rgb.shape,f.depth.shape) for f in batch.frames])
        if monotonic()-last_progress>20:error='No rendered batch progress for 20 seconds'
        if pending_run is not None and batch and batch.stamp.run_id!=pending_run:
            completed+=1;pending_run=None;pending_since=None;window.running.setChecked(True)
        if pending_since is not None and monotonic()-pending_since>20:error='Project reopen timed out'
        if pending_run is None and monotonic()-last_physics>20:error='Physics stopped progressing for 20 seconds'
        if elapsed>=next_sample:
            rss=int(Path('/proc/self/statm').read_text().split()[1])*os.sysconf('SC_PAGE_SIZE')/1024**2
            samples.append(dict(seconds=elapsed,rss_mib=rss,rendered=window.worker.rendered,
                                sim_time=batch.sim_time if batch else None,
                                reopens_completed=completed,ui_dropped=window.worker.mailbox.dropped,gpu=gpu_sample()))
            print(json.dumps(samples[-1]),flush=True)
            next_sample=elapsed+60
        if elapsed>=args.seconds or error:
            timer.stop();window.close();return
        if elapsed>=next_reload and batch and pending_run is None:
            pending_run=batch.stamp.run_id;pending_since=monotonic()
            window.request_open_project(args.project);reopens+=1
            next_reload=elapsed+args.reload_every
    timer=QTimer();timer.timeout.connect(tick);timer.start(200)
    window.show();app.exec();timer.stop()
    result={**environment(),'gl_renderer':window.worker.gl_renderer,'requested_seconds':args.seconds,
            'elapsed_seconds':monotonic()-start,'width':1280,'height':720,'cameras':3,'rgb_and_depth':True,
            'desktop_visible':QApplication.platformName()!='offscreen','reopens_requested':reopens,
            'reopens_completed':completed,'rendered_batches':window.worker.rendered,
            'worker_stopped':not window.worker.is_alive(),'memory_review':'Inspect steady-state RSS/GPU trends separately','error':error or window.worker.error,'samples':samples}
    args.report.parent.mkdir(parents=True,exist_ok=True)
    args.report.write_text(json.dumps(result,indent=2)+'\n')
    if result['error'] or not result['desktop_visible'] or not result['worker_stopped'] or completed!=reopens or result['elapsed_seconds']<args.seconds:
        raise RuntimeError('Soak incomplete; inspect '+str(args.report))


if __name__=='__main__':main()
