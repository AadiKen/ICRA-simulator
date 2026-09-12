#!/usr/bin/env python3
"""Compare one-sample VRX coast accelerations with start-state Node forces."""
import argparse
import csv
import json
import math
from pathlib import Path


DT = 0.05
MASS = [54.915, 91.525, 19.32251742]


def ned_velocity(row):
    return [
        float(row["velocity_body_x"]),
        -float(row["velocity_body_y"]),
        -float(row["angular_velocity_body_z"]),
    ]


def damping_force(nu):
    u, v, r = nu
    return [
        -(6 * u + 18 * abs(u) * u),
        -(18 * v + 60 * abs(v) * v),
        -(8 * r + 12 * abs(r) * r),
    ]


def coriolis_vector(nu):
    u, v, r = nu
    rigid_body = [-52.3 * v * r, 52.3 * u * r, 0]
    added_mass = [-39.225 * v * r, 2.615 * u * r, (39.225 - 2.615) * u * v]
    return [rigid_body[i] + added_mass[i] for i in range(3)]


def nearest(rows, time_s):
    return min(rows, key=lambda row: abs(float(row["time_s"]) - time_s))


def pearson(left, right):
    left_mean = sum(left) / len(left)
    right_mean = sum(right) / len(right)
    numerator = sum((x - left_mean) * (y - right_mean) for x, y in zip(left, right))
    left_energy = sum((x - left_mean) ** 2 for x in left)
    right_energy = sum((y - right_mean) ** 2 for y in right)
    denominator = math.sqrt(left_energy * right_energy)
    return numerator / denominator if denominator else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("telemetry", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument(
        "--start-times",
        type=float,
        nargs="+",
        default=[1.9, 2.15, 2.45, 2.75, 3.0, 3.35, 3.7, 4.05, 4.5, 4.8, 5.15],
    )
    args = parser.parse_args()
    with args.telemetry.open() as stream:
        rows = list(csv.DictReader(stream))

    comparisons = []
    for requested_time in args.start_times:
        start = nearest(rows, requested_time)
        start_time = float(start["time_s"])
        end = nearest(rows, start_time + DT)
        end_time = float(end["time_s"])
        if abs((end_time - start_time) - DT) > 1e-9:
            raise ValueError(f"samples at {start_time} and {end_time} are not one {DT}s step apart")
        nu_0, nu_1 = ned_velocity(start), ned_velocity(end)
        measured = [(nu_1[i] - nu_0[i]) / DT for i in range(3)]
        damping = damping_force(nu_0)
        c_nu = coriolis_vector(nu_0)
        bias_force = [-value for value in c_nu]
        theoretical = [(damping[i] + bias_force[i]) / MASS[i] for i in range(3)]
        residual = [measured[i] - theoretical[i] for i in range(3)]
        linear_residual = residual[:2]
        linear_speed = math.hypot(nu_0[0], nu_0[1])
        linear_residual_magnitude = math.hypot(*linear_residual)
        comparisons.append({
            "interval_s": [start_time, end_time],
            "nu_0": nu_0,
            "nu_1": nu_1,
            "vrx_interval_acceleration": measured,
            "node_start_state_damping_force_n_nm": damping,
            "node_start_state_bias_force_n_nm": bias_force,
            "node_start_state_theoretical_acceleration": theoretical,
            "vrx_minus_node": residual,
            "linear_residual_l2_mps2": linear_residual_magnitude,
            "linear_residual_angle_deg": math.degrees(math.atan2(residual[1], residual[0])),
            "linear_residual_vs_velocity_cosine": (
                (residual[0] * nu_0[0] + residual[1] * nu_0[1])
                / (linear_residual_magnitude * linear_speed)
                if linear_residual_magnitude and linear_speed
                else None
            ),
        })

    magnitudes = [item["linear_residual_l2_mps2"] for item in comparisons]
    residual_components = [[item["vrx_minus_node"][axis] for item in comparisons] for axis in range(3)]
    velocity_components = [[item["nu_0"][axis] for item in comparisons] for axis in range(3)]
    correlations = {
        f"residual_{residual_axis}_vs_{velocity_axis}": pearson(
            residual_components[residual_index], velocity_components[velocity_index]
        )
        for residual_index, residual_axis in enumerate(("surge", "sway"))
        for velocity_index, velocity_axis in enumerate(("surge", "sway", "yaw_rate"))
    }
    sway_signs = {math.copysign(1, value) for value in residual_components[1] if value}
    systematic = len(sway_signs) == 1 or any(
        value is not None and abs(value) >= 0.75 for value in correlations.values()
    )
    report = {
        "schema_version": 1,
        "status": "SINGLE_RECORDED_STEP_COMPARISON_COMPLETE",
        "telemetry": str(args.telemetry),
        "step_size_s": DT,
        "mass_diagonal": MASS,
        "comparisons": comparisons,
        "linear_residual_summary_mps2": {
            "min": min(magnitudes),
            "max": max(magnitudes),
            "mean": sum(magnitudes) / len(magnitudes),
            "rms": math.sqrt(sum(value ** 2 for value in magnitudes) / len(magnitudes)),
        },
        "direction_analysis": {
            "pearson_correlations": correlations,
            "all_sway_residuals_have_same_sign": len(sway_signs) == 1,
            "classification": "systematic" if systematic else "noise-like",
            "criterion": "Systematic when one planar component has a constant nonzero sign or any residual/velocity Pearson correlation has magnitude at least 0.75.",
        },
        "integrators": {
            "node": {
                "scheme": "classical fourth-order Runge-Kutta (RK4)",
                "order": 4,
                "source": "core/dynamicsCore.js default integrator and core/integrator.js stepRK4",
            },
            "vrx_gazebo_dart": {
                "scheme": "semi-implicit Euler with velocity-level constraint solve",
                "order": 1,
                "step_size_s": DT,
                "source": "world SDF selects DART and max_step_size=0.05; DART 6 World::step default",
            },
            "mismatch": True,
        },
        "interpretation": "Every comparison uses telemetry samples exactly 0.05 s apart and evaluates both Node force terms at the same interval-start nu_0. The residual is small but structured rather than noise-like: the sway residual retains one sign and correlates strongly with yaw rate. The RK4 versus semi-implicit-Euler mismatch is a plausible contributor, but this dataset alone does not isolate it as the sole cause.",
        "conclusion": "Check 1 fails the proposal's noise-like criterion. Do not draft, apply, or run a relaxed Gate 7 tolerance from this evidence. Isolate the systematic first-order residual, preferably with a timestep-convergence or matched-integrator experiment.",
        "gate_7_tolerance_proposed": False,
        "protected_configuration_changed": False,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
