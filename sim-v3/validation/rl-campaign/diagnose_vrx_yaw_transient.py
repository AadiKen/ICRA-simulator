#!/usr/bin/env python3
"""Separate Surveyor yaw transient, mixed-command, and coupling divergence."""
import json,math,statistics,sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/"validation/rl-campaign/ports"),str(ROOT/"packages/python-client")]
from bcod_sim import CommonWaypointEnv
from bcod_sim.node_bridge import PersistentNodeBridge
from vrx_gym_runtime import Runtime as VrxRuntime

DT=.05
SEED=30000

def command(action):
    return {"active_sensors":["imu","gps"],"actuators":{"effectors":{"port":{"command":action[0]},"starboard":{"command":action[1]}}}}

def angle_delta(a,b):
    return (a-b+math.pi)%(2*math.pi)-math.pi

probe=CommonWaypointEnv(ROOT,fixed_reset_seed=SEED,disturbance_mode="zero")
ZERO_CONFIG=probe._config(SEED)
probe.disturbance_mode="seeded"
SEEDED_CONFIG=probe._config(SEED)
probe.close()

def rates_from_headings(headings):
    return [angle_delta(headings[i],headings[i-1])/DT for i in range(1,len(headings))]

def run_node(config,actions):
    bridge=PersistentNodeBridge(ROOT)
    try:
        bridge.reset([config]);headings=[bridge.ground_truth()["attitude_rad"][2]]
        for action in actions:
            bridge.step([command(action)]);headings.append(bridge.ground_truth()["attitude_rad"][2])
        return rates_from_headings(headings)
    finally:bridge.close()

def run_vrx(config,actions):
    runtime=VrxRuntime()
    try:
        initial=runtime.reset(config);headings=[initial["truth"]["attitude_rad"][2]]
        for action in actions:
            result=runtime.step(command(action));headings.append(result["truth"]["attitude_rad"][2])
        return rates_from_headings(headings)
    finally:runtime._stop()

def first_crossing(values,target,fraction):
    threshold=abs(target)*fraction
    for i,value in enumerate(values):
        if abs(value)>=threshold:return (i+1)*DT
    return None

def transient_metrics(values):
    steady=statistics.mean(values[-40:]);peak=max(values,key=abs)
    band=.05*abs(steady)
    settling=None
    for i in range(len(values)):
        if all(abs(x-steady)<=band for x in values[i:]):settling=(i+1)*DT;break
    return {"steady_rate_rad_s":steady,"peak_rate_rad_s":peak,"rise_time_10_s":first_crossing(values,steady,.1),"rise_time_90_s":first_crossing(values,steady,.9),"overshoot_fraction":max(0,(abs(peak)-abs(steady))/max(abs(steady),1e-12)),"settling_time_5pct_s":settling}

step_actions=[(-.5,.5)]*200
step_zero=[(0.,0.)]*200
node_step=run_node(ZERO_CONFIG,step_actions);node_zero=run_node(ZERO_CONFIG,step_zero)
vrx_step=run_vrx(ZERO_CONFIG,step_actions);vrx_zero=run_vrx(ZERO_CONFIG,step_zero)
node_isolated=[a-b for a,b in zip(node_step,node_zero)]
vrx_isolated=[a-b for a,b in zip(vrx_step,vrx_zero)]

control_actions=[(.2+.01*i,.2-.005*i) for i in range(12)]
physics_mixed=[action for action in control_actions for _ in range(2)]
physics_pure=[];physics_common=[]
for port,starboard in control_actions:
    common=(port+starboard)/2;diff=(starboard-port)/2
    physics_pure.extend([(-diff,diff)]*2);physics_common.extend([(common,common)]*2)

series={}
for simulator,runner in (("bcod",run_node),("vrx",run_vrx)):
    mixed=runner(SEEDED_CONFIG,physics_mixed)
    pure=runner(SEEDED_CONFIG,physics_pure)
    common=runner(SEEDED_CONFIG,physics_common)
    zero=runner(SEEDED_CONFIG,[(0.,0.)]*len(physics_mixed))
    coupling=[m-p-c+z for m,p,c,z in zip(mixed,pure,common,zero)]
    series[simulator]={"mixed":mixed,"pure_differential":pure,"common_mode":common,"zero_command":zero,"mixed_minus_zero":[m-z for m,z in zip(mixed,zero)],"pure_differential_minus_zero":[p-z for p,z in zip(pure,zero)],"common_mode_minus_zero":[c-z for c,z in zip(common,zero)],"nonlinear_superposition_residual":coupling}

def rmse(a,b):return math.sqrt(statistics.mean((x-y)**2 for x,y in zip(a,b)))
report={
    "schema_version":1,
    "artifact_kind":"vrx-yaw-transient-and-mixed-command-diagnostic",
    "seed":SEED,
    "differential_step":{"command_normalized":[-.5,.5],"duration_s":10,"paired_zero_subtraction":True,"bcod":transient_metrics(node_isolated),"vrx":transient_metrics(vrx_isolated),"rate_rmse_rad_s":rmse(node_isolated,vrx_isolated),"samples":{"time_s":[(i+1)*DT for i in range(200)],"bcod_rate_rad_s":node_isolated,"vrx_rate_rad_s":vrx_isolated}},
    "gate_c":{"command_kind":"mixed common-mode surge plus differential yaw; exact 12-control-step Gate C sequence, each held for two 50 ms physics steps","control_actions":control_actions,"decomposition":{"pure_differential_actions":[[-(s-p)/2,(s-p)/2] for p,s in control_actions],"common_mode_actions":[[(p+s)/2,(p+s)/2] for p,s in control_actions],"paired_seeded_zero_subtraction":True},"raw_rmse_bcod_vs_vrx_rad_s":{"mixed":rmse(series["bcod"]["mixed"],series["vrx"]["mixed"]),"pure_differential":rmse(series["bcod"]["pure_differential"],series["vrx"]["pure_differential"]),"common_mode":rmse(series["bcod"]["common_mode"],series["vrx"]["common_mode"]),"zero_command":rmse(series["bcod"]["zero_command"],series["vrx"]["zero_command"])},"disturbance_subtracted_rmse_bcod_vs_vrx_rad_s":{"mixed":rmse(series["bcod"]["mixed_minus_zero"],series["vrx"]["mixed_minus_zero"]),"pure_differential":rmse(series["bcod"]["pure_differential_minus_zero"],series["vrx"]["pure_differential_minus_zero"]),"common_mode":rmse(series["bcod"]["common_mode_minus_zero"],series["vrx"]["common_mode_minus_zero"])},"mean_abs_nonlinear_superposition_residual_rad_s":{name:statistics.mean(map(abs,value["nonlinear_superposition_residual"])) for name,value in series.items()},"series":series}
}
out=ROOT/"artifacts/rl-campaign/vrx-yaw-transient-and-mixed-command-diagnostic.json"
out.write_text(json.dumps(report,indent=2)+"\n")
print(json.dumps({"artifact":str(out),"differential_step":{k:v for k,v in report["differential_step"].items() if k!="samples"},"gate_c":{k:v for k,v in report["gate_c"].items() if k not in ("series","control_actions","decomposition")}},indent=2))
