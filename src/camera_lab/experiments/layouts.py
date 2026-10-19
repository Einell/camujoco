"""Candidate generation from measured mount locations, never inferred anatomy."""
import numpy as np
from ..render.editing import quaternion


def generate_candidates(camera, parent, mounts, targets, extensions=(.02,.04), up=(0,1,0)):
    """All coordinates are in parent frame, metres. Origin is zero-extension
    optical centre, not shell surface. Outward gives translation direction.
    up fixes image roll; it must not be parallel to any optical axis.
    """
    result=[];up=np.asarray(up,dtype=float)
    if up.shape!=(3,) or not np.isfinite(up).all() or np.linalg.norm(up)<1e-9:
        raise ValueError('Require finite nonzero up direction')
    up=up/np.linalg.norm(up)
    for side,mount in mounts.items():
        origin=np.asarray(mount['origin'],dtype=float);outward=np.asarray(mount['outward'],dtype=float)
        if origin.shape!=(3,) or outward.shape!=(3,) or not np.isfinite(origin).all() or not np.isfinite(outward).all() or np.linalg.norm(outward)<1e-9:
            raise ValueError('Require measured XYZ origin and nonzero outward vector')
        outward=outward/np.linalg.norm(outward)
        for index,extension in enumerate(extensions):
            if not np.isfinite(extension) or extension<0:raise ValueError('Invalid extension')
            position=origin+extension*outward
            for region,target in targets.items():
                target=np.asarray(target,dtype=float)
                if target.shape!=(3,) or not np.isfinite(target).all():raise ValueError('Invalid target')
                z=target-position
                if np.linalg.norm(z)<1e-9:raise ValueError('Camera coincides with target')
                z/=np.linalg.norm(z);x=np.cross(z,up)
                if np.linalg.norm(x)<1e-9:raise ValueError('Choose up not parallel to optical axis')
                x/=np.linalg.norm(x)
                rotation=np.column_stack((x,np.cross(z,x),z))
                result.append(dict(name=f'{side}-e{index}-{region}',camera=camera,parent=parent,
                                   position=position.tolist(),rotation=quaternion(rotation).tolist()))
    return result
