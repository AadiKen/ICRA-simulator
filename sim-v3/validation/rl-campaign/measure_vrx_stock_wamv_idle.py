#!/usr/bin/env python3
"""Measure a stock WAM-V idle interval from gz dynamic-pose JSONL."""
import json,math,sys

def rpy(q):
    x,y,z,w=(float(q.get(k,0.0)) for k in ("x","y","z","w"))
    roll=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
    pitch=math.asin(max(-1.0,min(1.0,2*(w*y-z*x))))
    yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    return roll,pitch,yaw

first=last=None;count=0;max_horizontal=0.0;min_z=math.inf;max_z=-math.inf;max_roll=max_pitch=0.0
for line in sys.stdin:
    try: message=json.loads(line)
    except json.JSONDecodeError: continue
    stamp=message.get("header",{}).get("stamp",{})
    t=float(stamp.get("sec",0))+float(stamp.get("nsec",0))*1e-9
    for pose in message.get("pose",[]):
        if pose.get("name")!="wamv": continue
        p=pose["position"]; roll,pitch,yaw=rpy(pose["orientation"])
        sample=(t,float(p["x"]),float(p["y"]),float(p["z"]),roll,pitch,yaw)
        if first is None: first=sample
        last=sample;count+=1
        max_horizontal=max(max_horizontal,math.hypot(sample[1]-first[1],sample[2]-first[2]))
        min_z=min(min_z,sample[3]);max_z=max(max_z,sample[3])
        max_roll=max(max_roll,abs(roll));max_pitch=max(max_pitch,abs(pitch))
        break
    if first and t-first[0]>=120.0: break
if count<2: raise SystemExit("insufficient WAM-V samples")
a,b=first,last
result={"simulation_interval_s":b[0]-a[0],"samples":count,
 "start":{"simulation_time_s":a[0],"position_enu_m":list(a[1:4]),"roll_rad":a[4],"pitch_rad":a[5]},
 "end":{"simulation_time_s":b[0],"position_enu_m":list(b[1:4]),"roll_rad":b[4],"pitch_rad":b[5]},
 "net_horizontal_displacement_m":math.hypot(b[1]-a[1],b[2]-a[2]),
 "maximum_horizontal_excursion_m":max_horizontal,"vertical_range_m":max_z-min_z,
 "maximum_abs_roll_rad":max_roll,"maximum_abs_pitch_rad":max_pitch}
print(json.dumps(result,indent=2))
