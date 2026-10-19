"""Generate 24 configurations for software validation, NOT a Revo3 recommendation."""
import json
from pathlib import Path
from camera_lab.experiments.layouts import generate_candidates

root=Path(__file__).resolve().parent
mounts={name:dict(origin=origin,outward=direction) for name,origin,direction in [
    ('palm',[0,-.06,.04],[0,-1,0]),('thumb',[.06,0,.04],[1,0,0]),
    ('little',[-.06,0,.04],[-1,0,0]),('back',[0,.06,.04],[0,1,0])]}
config=dict(model='synthetic.xml',width=320,height=240,
    description='Synthetic software fixture only, not Revo3 or a real task trajectory',
    candidates=generate_candidates('wrist_camera','wrist',mounts,
        {'near':[0,0,.44],'mid':[0,0,.50],'far':[0,0,.56]}),
    features=[dict(name='face_left',body='target',point=[-.015,0,-.04],diameter_m=.005),
              dict(name='face_right',body='target',point=[.015,0,-.04],diameter_m=.005)],
    thresholds=dict(min_range_m=.05,max_range_m=1.,min_pixels=1.,tolerance_m=.002),samples=[])
for i in range(9):
    config['samples'].append(dict(time_s=i/30,duration_s=1/30,
        phase=('open' if i<3 else 'occlude' if i<6 else 'release'),
        qpos=[(i-4)*.04, -.2 if 3<=i<6 else 0]))
(root/'synthetic.json').write_text(json.dumps(config,indent=2)+'\n')
