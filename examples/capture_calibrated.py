"""Capture aligned observations and ground truth without a desktop window."""
import argparse
from dataclasses import asdict, replace
from pathlib import Path
import json
import numpy as np
from camera_lab.render.worker import RenderSession


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('captures/sample.npz'))
    args = parser.parse_args()
    with RenderSession() as session:
        session.configure_camera('front', replace(session.camera_config('front'),
            width=640, height=480, fx=520, fy=510, cx=315, cy=237, near=.05, far=10))
        session.advance_to(.2)
        observations, truth = session.capture_sample()
        arrays = {}
        for frame in observations.frames:
            for name in ('rgb','depth','valid_mask','K','T_world_camera'):
                arrays[f'{frame.camera_id}/{name}'] = getattr(frame,name)
        # JSON metadata avoids numpy object arrays and pickle-based loading.
        metadata = {'stamp':asdict(observations.stamp),
                    'cameras':{f.camera_id:{'revision':f.camera_revision, 'config':asdict(f.config)}
                               for f in observations.frames},
                    'T_world_object':{name:pose.tolist() for name,pose in truth.object_poses.items()}}
        arrays['metadata_json'] = np.array(json.dumps(metadata))
        args.output.parent.mkdir(parents=True,exist_ok=True)
        np.savez_compressed(args.output,**arrays)
        print(f'Saved {len(observations.frames)} synchronized cameras: {args.output}')


if __name__ == '__main__':
    main()
