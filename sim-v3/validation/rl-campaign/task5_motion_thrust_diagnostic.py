#!/usr/bin/env python3
"""Evaluation-only motion/thrust diagnostic for the initial Task 5 models."""
from __future__ import annotations

import json
import math
import statistics
import sys
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "packages/python-client"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_task5_conditions import Task5ConditionEnv  # noqa: E402

OUT_DIR = ROOT / "artifacts/rl-campaign/surveyor/task-5-sensor-yaw-250k"
OUT = OUT_DIR / "motion-thrust-los-diagnostic.json"
PLOT = OUT_DIR / "distance-to-goal-policy-vs-los-pid.svg"
SEEDS = range(30000, 30050)
DT = 0.1
MAX_THRUST_N = 70.0


def summary(xs):
    xs = list(map(float, xs))
    return {"median": statistics.median(xs), "mean": statistics.fmean(xs),
            "p95": float(np.quantile(xs, .95)), "min": min(xs), "max": max(xs)} if xs else None


def wrap(x):
    return (x + math.pi) % (2 * math.pi) - math.pi


def actuator_snapshot(env):
    state = env.bridge.checkpoint()["checkpoints"][0]["payload"]["actuatorState"]
    effectors = {x["id"]: x for x in state["effectors"]}
    return state, effectors


def classify(distances, success):
    closest_i = int(np.argmin(distances)); closest = distances[closest_i]
    improvement = distances[0] - closest
    rebound = max(distances[closest_i:]) - closest
    tail = distances[max(0, len(distances) - 100):]
    tail_span = max(tail) - min(tail)
    if success: return "success"
    if improvement < 2.0: return "never_meaningly_decreased"
    if rebound >= 2.0: return "decreased_then_reversed"
    if tail_span < 1.0: return "decreased_then_stalled"
    return "decreased_without_closing"


def row(seed, distances, speeds, commands, thrust_times, thrust_commands, achieved, modes, allocation_failures, success):
    cls = classify(distances, success)
    cmd = np.asarray(commands); thrust_cmd = np.asarray(thrust_commands); ach = np.asarray(achieved)
    target = thrust_cmd * MAX_THRUST_N
    return {"seed": seed, "success": success, "distance_trace_class": cls,
            "start_distance_m": distances[0], "closest_distance_m": min(distances),
            "final_distance_m": distances[-1], "net_closure_m": distances[0]-distances[-1],
            "closure_to_closest_m": distances[0]-min(distances),
            "speed_mps": summary(speeds), "mean_abs_command": float(np.mean(np.abs(cmd))),
            "command_saturation_fraction": float(np.mean(np.abs(cmd) >= .99)),
            "mean_abs_target_thrust_n": float(np.mean(np.abs(target))),
            "mean_abs_achieved_thrust_n": float(np.mean(np.abs(ach))),
            "thrust_tracking_abs_error_n": summary(np.abs(target-ach).ravel()),
            "allocation_failure_count": allocation_failures,
            "effector_failure_modes": dict(Counter(modes)),
            "trace_10hz": {"time_s": [round(i*DT, 3) for i in range(len(distances))],
                            "distance_m": distances, "speed_mps": speeds,
                            "command": commands},
            "thrust_trace_1hz": {"time_s": thrust_times, "command": thrust_commands,
                                 "target_thrust_n": target.tolist(), "achieved_thrust_n": achieved}}


def policy_episode(model, condition, seed):
    env = Task5ConditionEnv(ROOT, fixed_reset_seed=seed, final_leg_curriculum=True, condition=condition)
    try:
        obs, _ = env.reset(); state = None; start = np.ones(1, bool)
        distances = [env._distance()]; speeds = [0.0]; commands = [[0., 0.]]
        thrust_times=[]; thrust_commands=[]; achieved=[]
        modes = []; allocation_failures = 0; success = False
        while True:
            action, state = model.predict(obs, state=state, episode_start=start, deterministic=True)
            obs, _, terminated, truncated, info = env.step(action)
            act = np.asarray(action, float).reshape(-1)
            distances.append(float(info["distance_to_final_waypoint_m"])); speeds.append(float(info["speed_mps"]))
            commands.append(act.tolist())
            if (len(distances)-1) % 10 == 0 or terminated or truncated:
                astate, eff = actuator_snapshot(env); thrust_times.append((len(distances)-1)*DT); thrust_commands.append(act.tolist())
                achieved.append([float(eff["port"]["thrust"]), float(eff["starboard"]["thrust"])])
                modes.extend([eff["port"]["failureMode"], eff["starboard"]["failureMode"]])
                allocation_failures += int(bool(astate.get("lastAllocationDiagnostics", {}).get("failed", False) if astate.get("lastAllocationDiagnostics") else False))
            allocation_failures += int(info["termination_reason"] == "allocation_failure")
            success = bool(info["success"]); start = np.asarray([terminated or truncated], bool)
            if terminated or truncated: break
        return row(seed, distances, speeds, commands, thrust_times, thrust_commands, achieved, modes, allocation_failures, success)
    finally: env.close()


def los_episode(seed):
    env = Task5ConditionEnv(ROOT, fixed_reset_seed=seed, final_leg_curriculum=True, condition="default-noise")
    try:
        env.reset(); a, b = env.route[1], env.route[2]; dx, dy = b[0]-a[0], b[1]-a[1]; den = dx*dx+dy*dy
        distances=[env._distance()]; speeds=[0.]; commands=[[0.,0.]]; thrust_times=[]; thrust_commands=[]; achieved=[]; modes=[]; failures=0; success=False
        for step in range(env.max_control_steps):
            t=env.last_truth; p=t["position_ned_m"]; yaw=t["attitude_rad"][2]; u=float(t["velocity_body_mps"][0]); r=float(t["angular_rate_body_rad_s"][2])
            cross=(-dy*(p[0]-a[0])+dx*(p[1]-a[1]))/math.sqrt(den); heading=math.atan2(dy,dx)-math.atan2(cross,4.0); err=wrap(heading-yaw)
            surge=max(-150.,min(150.,100.*(1.5-u))); yaw_w=max(-100.,min(100.,70.*err-35.*r))
            cmd={"active_sensors":["imu","gps"],"actuators":{"desiredWrench":[surge,0,0,0,0,yaw_w]}}
            for _ in range(env.physics_steps_per_action): env.bridge.step([cmd])
            env.last_truth=env.bridge.ground_truth(); d=env._distance(); v=math.hypot(*env.last_truth["velocity_body_mps"][:2]); current_cmd=commands[-1]
            if step % 10 == 9 or d <= env.final_radius_m:
                astate,eff=actuator_snapshot(env)
                allocated=astate["lastEffectorCommands"]
                current_cmd=[float(allocated["port"]["thrust"])/MAX_THRUST_N,
                             float(allocated["starboard"]["thrust"])/MAX_THRUST_N]
                thrust_times.append((step+1)*DT); thrust_commands.append(current_cmd); achieved.append([float(eff["port"]["thrust"]),float(eff["starboard"]["thrust"])])
                modes.extend([eff["port"]["failureMode"],eff["starboard"]["failureMode"]])
                diagnostics = astate.get("lastAllocationDiagnostics") or {}
                failures += int(bool(diagnostics.get("failed",False)))
            distances.append(d); speeds.append(v); commands.append(current_cmd)
            if d <= env.final_radius_m: success=True; break
        return row(seed,distances,speeds,commands,thrust_times,thrust_commands,achieved,modes,failures,success)
    finally: env.close()


def aggregate(rows):
    return {"episodes": len(rows), "success_rate": sum(r["success"] for r in rows)/len(rows),
            "distance_trace_classes": dict(Counter(r["distance_trace_class"] for r in rows)),
            "start_distance_m": summary(r["start_distance_m"] for r in rows),
            "closest_distance_m": summary(r["closest_distance_m"] for r in rows),
            "final_distance_m": summary(r["final_distance_m"] for r in rows),
            "net_closure_m": summary(r["net_closure_m"] for r in rows),
            "episode_peak_speed_mps": summary(r["speed_mps"]["max"] for r in rows),
            "episode_mean_abs_command": summary(r["mean_abs_command"] for r in rows),
            "episode_mean_abs_achieved_thrust_n": summary(r["mean_abs_achieved_thrust_n"] for r in rows),
            "total_allocation_failures": sum(r["allocation_failure_count"] for r in rows), "rows": rows}


def plot(groups):
    width,height=1500,500; margin=55; panel_w=(width-2*margin)/3; panel_h=360; top=65
    sx=lambda panel,t: margin+panel*panel_w+(t/120)*(panel_w-25)
    sy=lambda d: top+panel_h-min(d,45)/45*panel_h
    lines=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
           '<rect width="100%" height="100%" fill="white"/>',
           '<text x="750" y="28" text-anchor="middle" font-family="sans-serif" font-size="18">Task 5 policies vs LOS-PID-v2 — seeds 30000–30049</text>']
    los=groups["LOS-PID-v2"]["rows"]
    for panel,condition in enumerate(("control","noise-zero","default-noise")):
        x0=sx(panel,0); x1=sx(panel,120); y0=sy(0); y1=sy(45)
        lines += [f'<rect x="{x0}" y="{y1}" width="{x1-x0}" height="{y0-y1}" fill="none" stroke="#aaa"/>',
                  f'<text x="{(x0+x1)/2}" y="52" text-anchor="middle" font-family="sans-serif" font-size="15">{condition}</text>',
                  f'<line x1="{x0}" y1="{sy(2)}" x2="{x1}" y2="{sy(2)}" stroke="#111" stroke-dasharray="5,4"/>']
        for rows,color,label in ((groups[condition]["rows"],"#3b82f6",condition),(los,"#ef4444","LOS-PID-v2")):
            for r in rows:
                ts=r["trace_10hz"]["time_s"][::10]; ds=r["trace_10hz"]["distance_m"][::10]
                pts=" ".join(f'{sx(panel,t):.1f},{sy(d):.1f}' for t,d in zip(ts,ds)); lines.append(f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-opacity=".10" stroke-width=".7"/>')
            grid=np.arange(0,120.1,1.0); stack=[]
            for r in rows:
                y=np.asarray(r["trace_10hz"]["distance_m"]); y=np.pad(y,(0,max(0,1201-len(y))),constant_values=y[-1]); stack.append(y[::10][:len(grid)])
            med=np.median(stack,axis=0); pts=" ".join(f'{sx(panel,t):.1f},{sy(d):.1f}' for t,d in zip(grid,med)); lines.append(f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="3"/>')
        lines += [f'<text x="{(x0+x1)/2}" y="465" text-anchor="middle" font-family="sans-serif" font-size="12">time (s)</text>',
                  f'<text x="{x0+8}" y="{top+16}" font-family="sans-serif" font-size="11" fill="#3b82f6">policy</text>',
                  f'<text x="{x0+55}" y="{top+16}" font-family="sans-serif" font-size="11" fill="#ef4444">LOS-PID-v2</text>']
    lines += ['<text x="16" y="250" transform="rotate(-90 16 250)" text-anchor="middle" font-family="sans-serif" font-size="13">distance to final goal (m)</text>','</svg>']
    PLOT.write_text("\n".join(lines)+"\n")


def main():
    from sb3_contrib import RecurrentPPO
    groups={}
    for condition in ("control","noise-zero","default-noise"):
        model=RecurrentPPO.load(OUT_DIR/condition/"recurrent-ppo-250k.zip",device="cpu")
        groups[condition]=aggregate([policy_episode(model,condition,s) for s in SEEDS])
        print(condition,groups[condition]["success_rate"],groups[condition]["distance_trace_classes"],flush=True)
    groups["LOS-PID-v2"]=aggregate([los_episode(s) for s in SEEDS]); print("LOS-PID-v2",groups["LOS-PID-v2"]["success_rate"],flush=True)
    plot(groups)
    OUT.write_text(json.dumps({"schema_version":1,"training_performed":False,"seeds":"30000-30049 identical for every controller","trace_class_thresholds":{"never_meaningly_decreased":"less than 2m improvement from start to closest approach","decreased_then_reversed":"at least 2m rebound after closest approach","decreased_then_stalled":"less than 1m range over final 10s","decreased_without_closing":"remaining unsuccessful approaches"},"controllers":groups,"plot":str(PLOT.relative_to(ROOT))},indent=2)+"\n")

if __name__ == "__main__": main()
