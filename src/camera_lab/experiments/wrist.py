"""Run: python -m camera_lab.experiments.wrist CONFIG.json --output NEW_DIRECTORY."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import re
import platform
import os
import mujoco
import numpy as np
from ..sdk import CameraLab
from ..evaluation import evaluate_features, summarize


def run_experiment(config, output):
    """Replay identical full qpos samples for each fixed camera mount.

    Outputs geometric screening only; no task success or collision safety claim.
    Paths in a dictionary are relative to cwd; CLI resolves paths by config file.
    """
    if not config.get('model'):raise ValueError('Fill in the model path first')
    candidates=config.get('candidates',[]);samples=config.get('samples',[])
    if not candidates or not samples:raise ValueError('Candidates and samples are required')
    names=[c['name'] for c in candidates]
    if len(set(names))!=len(names) or any(not re.fullmatch(r'[A-Za-z0-9_-]+',n) for n in names):
        raise ValueError('Candidate names must be unique safe filenames')
    # Validate interval semantics before starting expensive rendering.
    summarize([dict(s,visible={'validation':True}) for s in samples])
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    normalized=json.dumps(config,sort_keys=True,allow_nan=False)
    (output/'config.json').write_text(normalized+'\n')
    result=dict(kind='kinematic_surface_landmark_screening',
        config_sha256=hashlib.sha256(normalized.encode()).hexdigest(),
        limitations=['ideal pinhole/depth; no optical blur or sensor dropout model',
                     'surface landmarks only; not whole feature visibility',
                     'camera/bracket collision safety and task success NOT evaluated'],candidates={})
    model=Path(config['model'])
    result['model_file_sha256']=hashlib.sha256(model.read_bytes()).hexdigest()
    with CameraLab(model=model,width=config.get('width',1280),height=config.get('height',720),
                   packages=config.get('packages')) as lab, (output/'frames.csv').open('w',newline='') as stream:
        fields=['candidate','sample','phase','time_s','duration_s','run_id','batch_id','feature',
                'status','visible','u','v','pixels','range_m']
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        lab.save_project(output/'input.camproj')
        result['project_sha256']=hashlib.sha256((output/'input.camproj').read_bytes()).hexdigest()
        result['environment']=dict(mujoco=mujoco.__version__,numpy=np.__version__,
            python=platform.python_version(),system=platform.platform(),gl_backend=os.environ.get('MUJOCO_GL','default'))
        original={c['name']:c for c in lab.cameras.describe()}
        for candidate in candidates:
            camera=candidate['camera']
            if camera not in original:raise ValueError(f'Camera {camera} not selected in model')
            # Restore initial camera calibration so each candidate is independent.
            calibration=vars(original[camera]['config']).copy()
            calibration.update(candidate.get('intrinsics',{}))
            lab.cameras.configure(camera,parent=candidate['parent'],position=candidate['position'],
                                  rotation=candidate['rotation'],**calibration)
            rows=[];worst=None
            for i,sample in enumerate(samples):
                lab.seek(qpos=sample['qpos'],qvel=sample.get('qvel'),time_s=sample['time_s'])
                observation,truth=lab.capture_with_truth()
                metrics=evaluate_features(observation,truth,camera,config['features'],**config.get('thresholds',{}))
                rows.append(dict(time_s=sample['time_s'],duration_s=sample['duration_s'],phase=sample['phase'],
                                 visible={name:m['visible'] for name,m in metrics.items()}))
                for name,m in metrics.items():
                    writer.writerow(dict(candidate=candidate['name'],sample=i,phase=sample['phase'],
                        time_s=sample['time_s'],duration_s=sample['duration_s'],run_id=observation.stamp.run_id,
                        batch_id=observation.stamp.batch_id,feature=name,**m))
                score=sum(m['visible'] for m in metrics.values())
                if worst is None or score<worst:
                    worst=score
                    frame=next(f for f in observation.frames if f.camera_id==camera)
                    # Standard NumPy archive keeps RGB, ideal depth and calibration together.
                    np.savez_compressed(output/(candidate['name']+'-worst.npz'),rgb=frame.rgb,depth=frame.depth,
                        K=frame.K,T_world_camera=frame.T_world_camera,sample_index=i)
            result['candidates'][candidate['name']]=summarize(rows)
            stream.flush()
    (output/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();config=json.loads(args.config.read_text())
    if config.get('model'):
        config['model']=str((args.config.parent/config['model']).resolve())
    if config.get('packages'):
        config['packages']={k:str((args.config.parent/v).resolve()) for k,v in config['packages'].items()}
    run_experiment(config,args.output)
    print(f'Geometric screening saved to {args.output}; not a real task success evaluation.')


if __name__=='__main__':main()
