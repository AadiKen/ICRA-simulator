"""Decode privileged ground-truth heading from frozen recurrent checkpoints.

Heading is used only as a diagnostic target. It is never added to policy input.
The decoder is trained and evaluated on disjoint reset seeds.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np
from sb3_contrib import RecurrentPPO

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "packages/python-client"))
from bcod_sim import CommonWaypointEnv  # noqa: E402

RUN = ROOT / "artifacts/rl-campaign/surveyor/p3-v7-recurrent-local"
OUT = ROOT / "artifacts/rl-campaign/surveyor/p3-v7-hidden-heading-probe.json"
PLOT = ROOT / "artifacts/rl-campaign/surveyor/p3-v7-hidden-heading-decodability.svg"
CHECKPOINTS = range(250_000, 1_500_001, 250_000)
TRAIN_SEEDS = range(30000, 30035)
TEST_SEEDS = range(30035, 30050)
SAMPLE_EVERY = 10
RIDGE_ALPHA = 1.0


def collect(model, seeds):
    hidden, target = [], []
    for seed in seeds:
        env = CommonWaypointEnv(ROOT, fixed_reset_seed=seed, final_leg_curriculum=True)
        obs, _ = env.reset(); state = None; episode_start = np.ones((1,), dtype=bool)
        try:
            for decision in range(env.max_control_steps):
                action, state = model.predict(obs, state=state, episode_start=episode_start,
                                              deterministic=True)
                if decision % SAMPLE_EVERY == 0:
                    yaw = float(env.last_truth["attitude_rad"][2])
                    hidden.append(np.asarray(state[0][-1, 0], dtype=np.float64).copy())
                    target.append((math.sin(yaw), math.cos(yaw)))
                obs, _, terminated, truncated, _ = env.step(action)
                episode_start = np.asarray([terminated or truncated], dtype=bool)
                if terminated or truncated: break
        finally: env.close()
    return np.asarray(hidden), np.asarray(target)


def ridge_fit_predict(x_train, y_train, x_test):
    mean=x_train.mean(0); scale=x_train.std(0); scale[scale < 1e-8] = 1.0
    a=(x_train-mean)/scale; b=(x_test-mean)/scale
    a=np.column_stack((np.ones(len(a)),a)); b=np.column_stack((np.ones(len(b)),b))
    penalty=np.eye(a.shape[1])*RIDGE_ALPHA; penalty[0,0]=0
    weights=np.linalg.solve(a.T@a+penalty,a.T@y_train)
    return b@weights


def score(y, prediction):
    true_angle=np.arctan2(y[:,0],y[:,1]); predicted_angle=np.arctan2(prediction[:,0],prediction[:,1])
    error=np.abs((predicted_angle-true_angle+np.pi)%(2*np.pi)-np.pi)
    residual=np.sum((y-prediction)**2); total=np.sum((y-y.mean(0))**2)
    return {"circular_mae_deg":float(np.degrees(error).mean()),
            "circular_median_absolute_error_deg":float(np.degrees(np.median(error))),
            "circular_p95_absolute_error_deg":float(np.degrees(np.quantile(error,.95))),
            "sin_cos_r2":float(1-residual/total),"test_samples":len(y)}


def write_svg(rows):
    width,height,left,top=760,440,85,45; plot_w,plot_h=630,320
    values=[r["circular_mae_deg"] for r in rows]; ymax=max(90,math.ceil(max(values)/10)*10)
    point=lambda i,v:(left+i*plot_w/(len(rows)-1),top+(ymax-v)*plot_h/ymax)
    points=" ".join(f"{x:.1f},{y:.1f}" for i,v in enumerate(values) for x,y in [point(i,v)])
    circles="".join(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="5" fill="#2563eb"/>' for i,v in enumerate(values) for x,y in [point(i,v)])
    xlabels="".join(f'<text x="{x:.1f}" y="395" text-anchor="middle">{rows[i]["checkpoint_steps"]/1e6:g}</text>' for i,v in enumerate(values) for x,_ in [point(i,v)])
    ylines="".join(f'<line x1="{left}" y1="{top+(ymax-y)*plot_h/ymax:.1f}" x2="{left+plot_w}" y2="{top+(ymax-y)*plot_h/ymax:.1f}" stroke="#ddd"/><text x="70" y="{top+(ymax-y)*plot_h/ymax+4:.1f}" text-anchor="end">{y}</text>' for y in range(0,ymax+1,15))
    PLOT.write_text(f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}"><rect width="100%" height="100%" fill="white"/><g font-family="sans-serif" font-size="13" fill="#222">{ylines}<line x1="{left}" y1="{top}" x2="{left}" y2="{top+plot_h}" stroke="#222"/><line x1="{left}" y1="{top+plot_h}" x2="{left+plot_w}" y2="{top+plot_h}" stroke="#222"/><polyline points="{points}" fill="none" stroke="#2563eb" stroke-width="3"/>{circles}{xlabels}<text x="400" y="425" text-anchor="middle">Phase-1 training steps (millions)</text><text x="20" y="205" text-anchor="middle" transform="rotate(-90 20 205)">Held-out circular MAE (degrees; lower is better)</text><text x="400" y="25" text-anchor="middle" font-size="18">LSTM hidden-state heading decodability</text></g></svg>''')


def main():
    if "--plot-only" in sys.argv:
        write_svg(json.loads(OUT.read_text())["checkpoints"]); print(PLOT); return
    rows=[]
    for steps in CHECKPOINTS:
        model=RecurrentPPO.load(RUN/f"phase1-{steps}.zip",device="cpu")
        x_train,y_train=collect(model,TRAIN_SEEDS); x_test,y_test=collect(model,TEST_SEEDS)
        metrics=score(y_test,ridge_fit_predict(x_train,y_train,x_test))
        row={"checkpoint_steps":steps,"train_samples":len(y_train),**metrics}; rows.append(row)
        print(json.dumps(row),flush=True)
    r2=np.asarray([r["sin_cos_r2"] for r in rows]); mae=np.asarray([r["circular_mae_deg"] for r in rows])
    report={"schema_version":1,"artifact_kind":"p3-v7-lstm-hidden-state-heading-probe",
            "status":"COMPLETE_DIAGNOSTIC_ONLY","policy_observation_modified":False,
            "privileged_target":"ground-truth yaw_rad; decoder target only",
            "decoder":{"type":"ridge linear regression on sin(yaw), cos(yaw)","alpha":RIDGE_ALPHA,
                       "actor_hidden_size":128,"sample_every_control_steps":SAMPLE_EVERY,
                       "train_seeds":[30000,30034],"held_out_test_seeds":[30035,30049]},
            "checkpoints":rows,"trend":{"r2_change_first_to_last":float(r2[-1]-r2[0]),
            "mae_change_deg_first_to_last":float(mae[-1]-mae[0]),
            "last_three_r2_range":float(np.ptp(r2[-3:]))},
            "interpretation_rule":"A clear continuing rise in held-out sin/cos R2 supports more budget. Flat poor R2 supports rejecting the claim that this LSTM is building a useful heading representation."}
    OUT.write_text(json.dumps(report,indent=2)+"\n")
    write_svg(rows)
    print(json.dumps({"output":str(OUT),"plot":str(PLOT),"trend":report["trend"]},indent=2))


if __name__ == "__main__": main()
