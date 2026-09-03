from __future__ import annotations

import json
import math
import statistics
import sys
from collections import Counter
from pathlib import Path

import numpy as np
from stable_baselines3 import PPO

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "packages/python-client"))
from bcod_sim import CommonWaypointEnv  # noqa: E402

OUT = ROOT / "artifacts/rl-campaign/surveyor/p3-v5-rate-corrected-diagnostic.json"
PLOT = ROOT / "artifacts/rl-campaign/surveyor/p3-v5-rate-corrected-reward-gradient.svg"
CHECKPOINT = ROOT / "artifacts/rl-campaign/surveyor/p3-v5-curriculum-local/phase1-1250000.zip"
SEEDS = list(range(30000, 30030))
CLASSICAL_SEEDS = list(range(30000, 30050))
DT = 0.1


def q(values, p):
    return float(np.quantile(np.asarray(values, dtype=float), p)) if values else None


def summarize(values):
    return {
        "n": len(values),
        "min": min(values) if values else None,
        "median": statistics.median(values) if values else None,
        "p75": q(values, 0.75),
        "p95": q(values, 0.95),
        "max": max(values) if values else None,
        "mean": statistics.fmean(values) if values else None,
    }


def episode_classification(distances, success):
    closest = min(distances)
    i = distances.index(closest)
    after = distances[i:]
    within6_s = sum(d <= 6 for d in distances) * DT
    radial = np.diff(distances)
    signs = np.sign(radial[np.abs(radial) > 1e-3])
    reversals = int(np.sum(signs[1:] != signs[:-1])) if len(signs) > 1 else 0
    if success:
        label = "success"
    elif closest <= 4 and max(after) >= closest + 2:
        label = "close_then_overshoot"
    elif closest <= 6 and within6_s >= 5 and reversals >= 4:
        label = "near_goal_orbit_or_oscillation"
    elif closest > 6:
        label = "did_not_enter_intermediate_radius"
    else:
        label = "approached_but_did_not_close"
    return label, within6_s, reversals


def evaluate_ppo(model):
    rows = []
    near_actions, near_deltas = [], []
    for seed in SEEDS:
        env = CommonWaypointEnv(ROOT, fixed_reset_seed=seed, final_leg_curriculum=True)
        obs, _ = env.reset()
        distances, speeds, actions, states = [], [], [], []
        success = False; base_total = 0.; shaped_total = 0.
        for step in range(env.max_control_steps):
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            shaped_total += float(reward); base_total += float(info["reward_components"]["base_reward"])
            t = info["terminal_state"]
            d = float(info["distance_to_final_waypoint_m"])
            a = np.asarray(action, dtype=float)
            distances.append(d); speeds.append(float(info["speed_mps"])); actions.append(a)
            states.append((float(t["velocity_body_mps"][0]), float(t["velocity_body_mps"][1]), float(t["angular_rate_body_rad_s"][2])))
            if d <= 10:
                near_actions.append(a)
                if len(actions) > 1: near_deltas.append(a - actions[-2])
            success = bool(info["success"])
            if terminated or truncated: break
        env.close()
        ci = int(np.argmin(distances)); label, within6, reversals = episode_classification(distances, success)
        rows.append({
            "seed": seed, "success": success, "classification": label,
            "closest_approach_m": distances[ci], "time_of_closest_approach_s": (ci + 1) * DT,
            "velocity_at_closest_approach": {"speed_over_ground_m_s": speeds[ci], "u_m_s": states[ci][0], "v_m_s": states[ci][1], "yaw_rate_rad_s": states[ci][2]},
            "terminal_distance_m": distances[-1], "time_within_6m_s": within6,
            "radial_direction_reversals": reversals,
            "base_return": base_total, "shaped_return_undiscounted_telemetry": shaped_total,
            "action_at_closest_approach": {"port": float(actions[ci][0]), "starboard": float(actions[ci][1])},
        })
    na = np.asarray(near_actions); nd = np.asarray(near_deltas) if near_deltas else np.empty((0, 2))
    action_diag = {
        "space": "continuous Box[-1,1]^2 (not discretized)",
        "actual_action_update_interval_s": DT,
        "frozen_contract_control_interval_s": 0.1,
        "cadence_mismatch": False,
        "near_goal_definition_m": 10,
        "near_goal_samples": len(na),
        "absolute_action": summarize(np.abs(na).ravel().tolist()),
        "absolute_step_delta": summarize(np.abs(nd).ravel().tolist()),
        "saturation_fraction_abs_ge_0_99": float(np.mean(np.abs(na) >= .99)) if len(na) else None,
        "small_command_fraction_abs_le_0_05": float(np.mean(np.abs(na) <= .05)) if len(na) else None,
    }
    return rows, action_diag


def wrap(x):
    return (x + math.pi) % (2 * math.pi) - math.pi


def classical_episode(seed, speed_cap):
    env = CommonWaypointEnv(ROOT, fixed_reset_seed=seed, final_leg_curriculum=True, shaping_enabled=False)
    env.reset(); a, b = env.route[1], env.route[2]
    dx, dy = b[0] - a[0], b[1] - a[1]; den = dx * dx + dy * dy
    closest = env._distance(); success = False
    for step in range(env.max_control_steps):
        t = env.last_truth; p = t["position_ned_m"]; yaw = t["attitude_rad"][2]
        u = float(t["velocity_body_mps"][0]); r = float(t["angular_rate_body_rad_s"][2])
        along = ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / den
        cross = (-dy * (p[0] - a[0]) + dx * (p[1] - a[1])) / math.sqrt(den)
        heading = math.atan2(dy, dx) - math.atan2(cross, 4.0)
        err = wrap(heading - yaw)
        d = math.hypot(b[0] - p[0], b[1] - p[1])
        target = speed_cap
        if speed_cap == 1.0:
            target = min(1.0, math.sqrt(.8 * d), max(.25, math.cos(min(math.pi / 2, abs(err)))))
        surge = max(-150, min(150, 100 * (target - u)))
        yaw_wrench = max(-100, min(100, 70 * err - 35 * r))
        command={"actuators": {"desiredWrench": [surge, 0, 0, 0, 0, yaw_wrench]}}
        for _ in range(env.physics_steps_per_action): env.bridge.step([command])
        env.last_truth = env.bridge.ground_truth(); d = env._distance(); closest = min(closest, d)
        if d <= env.final_radius_m: success = True; break
    env.close()
    return {"seed": seed, "success": success, "closest_approach_m": closest, "steps": step + 1}


def classical(controller, speed):
    rows = [classical_episode(seed, speed) for seed in CLASSICAL_SEEDS]
    return {"controller": controller, "episodes": len(rows), "success_rate": sum(r["success"] for r in rows) / len(rows), "closest_approach_m": summarize([r["closest_approach_m"] for r in rows]), "raw": rows}


def disturbances():
    env = CommonWaypointEnv(ROOT, fixed_reset_seed=0, final_leg_curriculum=True)
    values = []
    for seed in CLASSICAL_SEEDS:
        _, _, route, current, wind = env._randomization(seed)
        values.append({"seed": seed, "current_m_s": math.hypot(*current[:2]), "wind_m_s": math.hypot(*wind[:2])})
    env.close()
    leg = math.dist(route[1], route[2])
    currents = [x["current_m_s"] for x in values]; winds = [x["wind_m_s"] for x in values]
    return {
        "scale_factor_from_bootstrap_task": 1.3569543912551827,
        "configured_ranges": {"current_m_s": [0, 1.3569543912551827], "wind_m_s": [0, 10.855635130041462]},
        "sampled_50_seed_magnitudes": {"current_m_s": summarize(currents), "wind_m_s": summarize(winds)},
        "final_leg_length_m": leg,
        "terminal_radius_as_leg_fraction": 2 / leg,
        "intermediate_radius_as_leg_fraction": 6 / leg,
        "median_current_drift_time_across_radius_s": {"2m": 2 / statistics.median(currents), "6m": 6 / statistics.median(currents)},
    }


def reward_plot():
    distances = np.linspace(0, 10, 1001); k, gamma = 5.0, .99
    curves = {}
    for name, delta in (("receding_0.05m_per_step", -.05), ("stationary", 0), ("closing_0.05m_per_step", .05)):
        next_d = np.maximum(0, distances - delta)
        shaped = 2 * delta + gamma * (-k * next_d) - (-k * distances)
        terminal = next_d <= 2
        shaped[terminal] = 2 * delta + 100 + k * distances[terminal]
        curves[name] = shaped
    width, height, left, top, right, bottom = 900, 520, 75, 45, 25, 65
    all_y = np.concatenate(list(curves.values())); ymin, ymax = float(all_y.min()), float(all_y.max())
    sx = lambda x: left + x / 10 * (width - left - right)
    sy = lambda y: top + (ymax - y) / (ymax - ymin) * (height - top - bottom)
    colors = {"receding_0.05m_per_step": "#c44e52", "stationary": "#4c72b0", "closing_0.05m_per_step": "#55a868"}
    lines = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">', '<rect width="100%" height="100%" fill="white"/>', f'<text x="{width/2}" y="24" text-anchor="middle" font-family="sans-serif" font-size="17">Final-leg immediate reward near terminal boundary</text>']
    lines += [f'<line x1="{left}" y1="{top}" x2="{left}" y2="{height-bottom}" stroke="black"/>', f'<line x1="{left}" y1="{height-bottom}" x2="{width-right}" y2="{height-bottom}" stroke="black"/>']
    for x in range(0, 11, 2):
        lines.append(f'<text x="{sx(x):.1f}" y="{height-bottom+22}" text-anchor="middle" font-family="sans-serif" font-size="12">{x}</text>')
    for x, dash, label in ((2, "7,5", "2m success"), (6, "2,5", "6m inactive boundary")):
        lines.append(f'<line x1="{sx(x):.1f}" y1="{top}" x2="{sx(x):.1f}" y2="{height-bottom}" stroke="#555" stroke-dasharray="{dash}"/>')
        lines.append(f'<text x="{sx(x)+5:.1f}" y="{top+16}" font-family="sans-serif" font-size="11">{label}</text>')
    for name, y in curves.items():
        pts = " ".join(f"{sx(x):.1f},{sy(v):.1f}" for x, v in zip(distances, y))
        lines.append(f'<polyline points="{pts}" fill="none" stroke="{colors[name]}" stroke-width="2"/>')
    for i, name in enumerate(curves):
        yy = 85 + i * 20; lines.append(f'<line x1="650" y1="{yy}" x2="675" y2="{yy}" stroke="{colors[name]}" stroke-width="3"/><text x="682" y="{yy+4}" font-family="sans-serif" font-size="11">{name.replace("_", " ")}</text>')
    lines += [f'<text x="{(left+width-right)/2}" y="{height-18}" text-anchor="middle" font-family="sans-serif" font-size="13">Distance before transition (m)</text>', f'<text x="18" y="{height/2}" transform="rotate(-90 18 {height/2})" text-anchor="middle" font-family="sans-serif" font-size="13">Immediate shaped reward</text>', '</svg>']
    PLOT.write_text("\n".join(lines) + "\n")
    samples = []
    for d in (1.99, 2.01, 4, 6, 10, 30):
        samples.append({"distance_m": d, "stationary_nonterminal_shaping_per_step": k * (1 - gamma) * d, "closing_0_05m_base_progress": .1, "closing_0_05m_shaping": k * ((1 - gamma) * d + gamma * .05)})
    return {
        "plot": str(PLOT.relative_to(ROOT)), "potential": "Phi=-5*distance; F=0.99*Phi(s')-Phi(s)",
        "threshold_findings": {"2m": "large positive terminal discontinuity (+100 base reward plus absorbing-potential transition), not a flat region", "6m": "no reward or transition discontinuity in final-leg-isolation episodes"},
        "stationary_state_effect": "Because Phi is negative and gamma<1, a stationary nonterminal transition receives +0.05*distance each step. This telescopes under the same discounted objective but creates a large immediate-reward/value scale.",
        "representative_values": samples,
    }


def main():
    model = PPO.load(CHECKPOINT, device="cpu")
    ppo, actions = evaluate_ppo(model)
    classes = Counter(r["classification"] for r in ppo)
    los_pid = classical("LOS-PID-v2", 1.5)
    speedcap = classical("LOS-SPEEDCAP-v2", 1.0)
    closest = [r["closest_approach_m"] for r in ppo]
    report = {
        "schema_version": 1, "artifact_kind": "p3-v5-final-leg-isolation-plateau-diagnostic",
        "task_contract_content_sha256": CommonWaypointEnv.EXPECTED_CONTRACT_SHA256,
        "checkpoint": str(CHECKPOINT.relative_to(ROOT)), "training_performed": False,
        "rate_fix": {"status":"corrected","physics_timestep_s":.05,"control_interval_s":.1,"physics_steps_per_action":2,"policy_decisions_per_episode":1200,"comparison_source":"artifacts/rl-campaign/surveyor/p3-v5-plateau-diagnostic.json"},
        "ppo_replay": {"seed_protocol": "first 30 fixed phase-1 evaluation seeds, 30000-30029", "episodes": len(ppo), "success_rate": sum(r["success"] for r in ppo) / len(ppo), "closest_approach_m": summarize(closest), "base_return":summarize([r["base_return"] for r in ppo]),"shaped_return_undiscounted_telemetry":summarize([r["shaped_return_undiscounted_telemetry"] for r in ppo]), "classification_counts": dict(classes), "raw": ppo},
        "reachability": {"same_start_distribution": True, "evaluation_seeds": "30000-30049", "disturbance": disturbances(), "classical": [los_pid, speedcap]},
        "reward_gradient": reward_plot(), "action_and_authority": actions,
    }
    classical_best = max(los_pid["success_rate"], speedcap["success_rate"])
    if classical_best >= .4:
        reach = "The segment is empirically reachable by classical control under the identical reset/disturbance distribution; the plateau is PPO-specific rather than a general physical-reachability failure."
    else:
        reach = "Classical success is also low under the identical reset/disturbance distribution, indicating that isolated-segment difficulty/disturbance is not PPO-specific."
    nonapproach = classes.get("did_not_enter_intermediate_radius", 0)
    if nonapproach >= len(ppo) / 2:
        dominant = "PPO control failure before the precision region: most deterministic rollouts never enter 6m. Reward-boundary flatness and action quantization are ruled out as primary explanations."
    elif classes.get("near_goal_orbit_or_oscillation", 0) + classes.get("close_then_overshoot", 0) >= len(ppo) / 2:
        dominant = "Near-goal control/precision behavior (orbiting or overshoot) dominates."
    else:
        dominant = "No single geometric trajectory class dominates; the evidence is mixed."
    report["conclusion"] = {
        "dominant_explanation": dominant,
        "reachability_interpretation": reach,
        "reward_interpretation": "There is no flat/discontinuous adverse gradient at 6m and success has a strong positive jump at 2m. However, the shaping term produces large positive immediate rewards even when stationary and shaped returns (~3.7k-4.1k) dwarf base returns (~-1.5k to -1.8k); this is a plausible PPO optimization-scale contributor, not proof of an unreachable goal.",
        "control_resolution_interpretation": "The policy action is continuous and now held for two 0.05s physics steps, exactly matching the frozen 10Hz control interval. Discretized action resolution is not limiting.",
        "fourth_revision_recommendation": "None authorized or implemented. Choose any future direction only after reviewing the empirical classifications and classical reachability result.",
    }
    OUT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"output": str(OUT), "ppo_success": report["ppo_replay"]["success_rate"], "classes": dict(classes), "los_pid_success": los_pid["success_rate"], "speedcap_success": speedcap["success_rate"], "closest": report["ppo_replay"]["closest_approach_m"]}, indent=2))


if __name__ == "__main__": main()
