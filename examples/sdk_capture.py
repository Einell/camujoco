"""Capture synchronized arrays with the Python SDK; no Qt or window required."""
import argparse
import json
from pathlib import Path
import numpy as np
from camera_lab import CameraLab


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',type=Path)
    parser.add_argument('--output',type=Path,default=Path('captures/sdk'))
    parser.add_argument('--width',type=int,default=1280)
    parser.add_argument('--height',type=int,default=720)
    args=parser.parse_args()
    with CameraLab(model=args.model,width=args.width,height=args.height) as lab:
        lab.step(steps=10)
        observation=lab.capture()
        args.output.mkdir(parents=True,exist_ok=True)
        cameras=[]
        for index,frame in enumerate(observation.frames):
            # Stable indices avoid treating model-provided camera names as paths.
            prefix=args.output/f'camera_{index}'
            np.save(str(prefix)+'_rgb.npy',frame.rgb)
            np.save(str(prefix)+'_depth.npy',frame.depth)
            np.save(str(prefix)+'_valid.npy',frame.valid_mask)
            cameras.append(dict(name=frame.camera_id,K=frame.K.tolist(),
                                T_world_camera=frame.T_world_camera.tolist()))
        metadata=dict(step_index=observation.stamp.step_index,
                      sim_time_s=observation.sim_time,cameras=cameras)
        (args.output/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
        print(json.dumps(metadata,indent=2))


if __name__=='__main__':main()
