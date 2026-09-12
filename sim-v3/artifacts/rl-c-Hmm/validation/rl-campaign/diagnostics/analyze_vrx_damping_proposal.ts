import { readFileSync, writeFileSync } from "node:fs";
import { resolve } from "node:path";

const root = resolve("artifacts/rl-campaign/vrx-damping-proposal");
const load = (name: string) =>
  readFileSync(resolve(root, name, "raw.jsonl"), "utf8")
    .trim()
    .split(/\n+/)
    .map((line) => JSON.parse(line));
const mean = (values: number[]) =>
  values.reduce((sum, value) => sum + value, 0) / values.length;
const summarize = (name: string) => {
  const all = load(name),
    rows = all.slice(10),
    tail = all.slice(-200),
    d = (row: any) => row.diagnostic_6dof;
  return {
    observations: all.length,
    duration_s: all.at(-1).time_s,
    all_finite: rows.every((row) => d(row).finite),
    max_abs_roll_deg:
      (Math.max(...rows.map((row) => Math.abs(d(row).rpy_enu_rad[0]))) * 180) /
      Math.PI,
    max_abs_pitch_deg:
      (Math.max(...rows.map((row) => Math.abs(d(row).rpy_enu_rad[1]))) * 180) /
      Math.PI,
    z_range_m: [
      Math.min(...rows.map((row) => d(row).position_enu_m[2])),
      Math.max(...rows.map((row) => d(row).position_enu_m[2])),
    ],
    last_10_s: {
      roll_rms_deg:
        (Math.sqrt(mean(tail.map((row) => d(row).rpy_enu_rad[0] ** 2))) * 180) /
        Math.PI,
      pitch_rms_deg:
        (Math.sqrt(mean(tail.map((row) => d(row).rpy_enu_rad[1] ** 2))) * 180) /
        Math.PI,
      vertical_speed_rms_m_s: Math.sqrt(
        mean(tail.map((row) => d(row).linear_velocity_body_flu_mps[2] ** 2)),
      ),
      angular_speed_rms_rad_s: Math.sqrt(
        mean(
          tail.map((row) =>
            d(row).angular_velocity_body_flu_rad_s.reduce(
              (sum: number, value: number) => sum + value * value,
              0,
            ),
          ),
        ),
      ),
      mean_z_m: mean(tail.map((row) => d(row).position_enu_m[2])),
    },
  };
};
const hydrostatic = {
  heave_stiffness_n_m: 10196,
  roll_stiffness_n_m_rad: 1094,
  pitch_stiffness_n_m_rad: 2080,
};
const critical = {
  zW: 2 * Math.sqrt(52.3 * hydrostatic.heave_stiffness_n_m),
  kP: 2 * Math.sqrt(5.3090425 * hydrostatic.roll_stiffness_n_m_rad),
  mQ: 2 * Math.sqrt(13.090471693745972 * hydrostatic.pitch_stiffness_n_m_rad),
};
const report = {
  schema_version: 1,
  artifact_kind: "vrx-out-of-plane-damping-proposal",
  status: "PROPOSAL_ONLY_NOT_APPLIED",
  scope: {
    seed: 20000,
    zero_thrust: true,
    disturbances: "Gate 7 seed-20000 wind and current",
    production_model_modified: false,
    gate_7_modified: false,
  },
  timestep_isolation: {
    baseline: {
      max_step_size_s: 0.05,
      articulated_blowup_s: 35.6,
      merged_blowup_s: 49,
    },
    fine_step_5_ms: summarize("step-0.005"),
    fine_step_1_ms: summarize("step-0.001"),
    conclusion:
      "Both fine-step runs completed 60 s with finite state; decreasing the substep strongly reduced attitude excursion. This confirms timestep-sensitive numerical instability.",
  },
  proposal: {
    classification:
      "engineering estimate; not manufacturer data, measured Surveyor data, or Node planar3 parity data",
    method:
      "Choose damping ratio 0.25 of linearized critical damping c_crit=2*sqrt(m_eff*k_hydrostatic), then round to practical coefficients before any runtime result is observed.",
    hydrostatic_stiffness: hydrostatic,
    effective_mass_and_inertia: {
      heave_mass_kg: 52.3,
      roll_inertia_kg_m2: 5.3090425,
      pitch_inertia_kg_m2: 13.090471693745972,
    },
    critical_damping: {
      zW_n_s_m: critical.zW,
      kP_n_m_s_rad: critical.kP,
      mQ_n_m_s_rad: critical.mQ,
    },
    proposed: { zW: 365, kP: 40, mQ: 85 },
    reasoning: {
      zW: "25% of critical heave damping based on displaced mass and the Surface waterplane-force slope.",
      kP: "25% of critical roll damping based on generated base-link roll inertia and the source-exact small-angle Surface moment slope.",
      mQ: "25% of critical pitch damping based on generated base-link pitch inertia and the source-exact small-angle Surface moment slope.",
    },
  },
  coarse_step_trial: {
    max_step_size_s: 0.05,
    result: summarize("coarse-proposed"),
    assessment:
      "Completed 60 s with finite state and bounded small attitude. The last-10-s RMS motion is small, consistent with settling rather than blow-up.",
  },
  decision:
    "Review required before adding these coefficients to renderSurveyorVrxModel(). No production coefficient has been changed.",
};
writeFileSync(
  resolve(root, "proposal.json"),
  JSON.stringify(report, null, 2) + "\n",
);
console.log(
  JSON.stringify(
    {
      status: report.status,
      timestep_isolation: report.timestep_isolation,
      proposal: report.proposal,
      coarse_step_trial: report.coarse_step_trial,
    },
    null,
    2,
  ),
);
