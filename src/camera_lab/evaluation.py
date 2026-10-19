"""Geometric surface-landmark screening, not a learned perception metric."""
import math
import numpy as np


def project_feature(frame, point_world, *, diameter_m=.005, min_pixels=0.,
                    min_range_m=0., max_range_m=math.inf, tolerance_m=.002):
    """Compare a labelled *surface* point with ideal rendered axial depth.

    Range limits are Euclidean camera-to-point distances. Pixel size is a
    fronto-parallel approximation, not a measured silhouette or sharpness score.
    """
    point=np.asarray(point_world,dtype=float)
    values=(diameter_m,min_pixels,min_range_m,tolerance_m)
    if point.shape!=(3,) or not np.isfinite(point).all() or any(not math.isfinite(v) or v<0 for v in values):
        raise ValueError('Finite XYZ and nonnegative thresholds required')
    if math.isnan(max_range_m) or max_range_m<=min_range_m:raise ValueError('Invalid range interval')
    camera=frame.T_world_camera[:3,:3].T @ (point-frame.T_world_camera[:3,3])
    x,y,z=camera;distance=float(np.linalg.norm(camera))
    result=dict(status='behind_camera',visible=False,u=None,v=None,pixels=0.,range_m=distance)
    if z<=0:return result
    pixel=frame.K @ camera;u,v=pixel[:2]/pixel[2]
    result.update(u=float(u),v=float(v),pixels=float(min(frame.K[0,0],frame.K[1,1])*diameter_m/z))
    height,width=frame.depth.shape
    if not (0<=u<width and 0<=v<height):result['status']='out_of_frame'
    elif not min_range_m<=distance<=max_range_m:result['status']='out_of_range'
    else:
        col=min(int(np.floor(u+.5)),width-1);row=min(int(np.floor(v+.5)),height-1)
        depth=float(frame.depth[row,col])
        if not frame.valid_mask[row,col] or not math.isfinite(depth):result['status']='invalid_depth'
        elif depth<z-tolerance_m:result['status']='occluded'
        elif abs(depth-z)>tolerance_m:result['status']='surface_mismatch'
        elif result['pixels']<min_pixels:result['status']='too_small'
        else:result.update(status='visible',visible=True)
    return result


def evaluate_features(observation, truth, camera_id, features, **thresholds):
    if observation.stamp!=truth.stamp:raise ValueError('Observation/truth frame mismatch')
    frame=next((f for f in observation.frames if f.camera_id==camera_id),None)
    if frame is None:raise ValueError(f'Unknown camera: {camera_id}')
    result={}
    for feature in features:
        name=feature['name']
        if name in result:raise ValueError('Feature names must be unique')
        transform=truth.object_poses[feature['body']]
        local=np.asarray(feature['point'],dtype=float)
        if local.shape!=(3,):raise ValueError('Feature point must be XYZ in body coordinates')
        world=transform[:3,:3] @ local+transform[:3,3]
        result[name]=project_feature(frame,world,diameter_m=feature.get('diameter_m',.005),**thresholds)
    if not result:raise ValueError('At least one surface feature is required')
    return result


def summarize(rows):
    """Time-weighted metrics over explicit, non-overlapping sample intervals.

    Each boolean is held for duration_s. Gaps break loss streaks; no interpolation
    or visibility claims are made between missing intervals.
    """
    if not rows:raise ValueError('At least one sample required')
    names=tuple(rows[0]['visible'])
    if not names:raise ValueError('At least one feature required')
    end=-math.inf
    for row in rows:
        t,d=row['time_s'],row['duration_s']
        if not math.isfinite(t) or not math.isfinite(d) or t<0 or d<=0 or t<end-1e-9:
            raise ValueError('Require ordered, nonoverlapping, positive-duration intervals')
        if set(row['visible'])!=set(names):raise ValueError('Feature set must stay constant')
        end=t+d
    def metric(subset, predicate):
        duration=sum(r['duration_s'] for r in subset)
        visible=0.;longest=0.;streak=0.;previous=-math.inf
        for row in subset:
            if row['time_s']>previous+1e-9:streak=0.
            if predicate(row):visible+=row['duration_s'];streak=0.
            else:streak+=row['duration_s'];longest=max(longest,streak)
            previous=row['time_s']+row['duration_s']
        return visible/duration,longest
    def group(subset):
        rate,gap=metric(subset,lambda r:all(r['visible'].values()))
        individual={}
        for name in names:
            fraction,loss=metric(subset,lambda r:r['visible'][name])
            individual[name]=dict(visible_fraction=fraction,longest_loss_s=loss)
        return dict(duration_s=sum(r['duration_s'] for r in subset),joint_visible_fraction=rate,
                    longest_joint_loss_s=gap,features=individual)
    phases={phase:group([r for r in rows if r['phase']==phase]) for phase in dict.fromkeys(r['phase'] for r in rows)}
    return dict(all=group(rows),phases=phases)
