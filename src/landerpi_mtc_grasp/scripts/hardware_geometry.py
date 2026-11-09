"""Calibrated bus-servo mapping and bounded trajectory interpolation."""
import math

TICK_RAD=math.radians(240)/1000

def to_angle(pulse,config):
    if not min(config['min'],config['max']) <= pulse <= max(config['min'],config['max']):
        raise ValueError('servo feedback outside calibrated pulse range')
    return (config['init']-pulse if config['min']>config['max'] else pulse-config['init'])*TICK_RAD

def to_pulse(angle,config):
    if not math.isfinite(angle): raise ValueError('nonfinite joint angle')
    pulse=config['init']+(-1 if config['min']>config['max'] else 1)*angle/TICK_RAD
    if not min(config['min'],config['max']) <= pulse <= max(config['min'],config['max']):
        raise ValueError('trajectory exceeds calibrated servo range')
    return round(pulse)

def validate(trajectory,expected,config,initial):
    names=trajectory.joint_names
    if len(names)!=len(expected) or set(names)!=set(expected) or len(set(names))!=len(names):
        raise ValueError('trajectory joint set differs from controller')
    if not trajectory.points: raise ValueError('empty trajectory')
    last=-1.;previous=[initial[n] for n in names]
    for point in trajectory.points:
        stamp=point.time_from_start.sec+point.time_from_start.nanosec*1e-9
        if stamp<0 or stamp<=last or len(point.positions)!=len(names):
            raise ValueError('invalid waypoint dimensions or timestamps')
        for name,angle,before in zip(names,point.positions,previous):
            to_pulse(angle,config[name])
            dt=stamp-max(last,0.)
            if dt==0 and abs(angle-before)>.08: raise ValueError('discontinuous trajectory start')
            if dt>0 and abs(angle-before)/dt>1.01: raise ValueError('joint velocity exceeds 1 rad/s')
        last=stamp;previous=point.positions

def interpolate(points,initial,elapsed):
    before=initial;start=0.
    for point in points:
        end=point.time_from_start.sec+point.time_from_start.nanosec*1e-9
        if elapsed<=end:
            weight=1. if end<=start else max(0.,min(1.,(elapsed-start)/(end-start)))
            return [a+weight*(b-a) for a,b in zip(before,point.positions)]
        before=point.positions;start=end
    return list(points[-1].positions)
