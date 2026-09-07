#!/bin/bash
#SBATCH --job-name=vrx-gate-d
#SBATCH --partition=gpu-a30-csl-cnice
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=32
#SBATCH --time=00:30:00
#SBATCH --output=artifacts/rl-campaign/vrx-gate-d-%j.log
set -euo pipefail
export PATH="${CODENIMBUS_NODE_BIN:?Set CODENIMBUS_NODE_BIN to the native Node bin directory}:$PATH"
cd "${BCOD_REPO:?Set BCOD_REPO to the isolated CodeNimbus benchmark checkout}"
if [[ "${VRX_GATE_D_RESUME_SCREENING:-0}" == "1" ]]; then
  gate_d_args=(--resume-screening)
  if [[ -n "${VRX_GATE_D_FORCE_COUNT:-}" ]]; then
    gate_d_args+=(--force-selected-count "$VRX_GATE_D_FORCE_COUNT")
  fi
  if [[ "${VRX_GATE_D_FINALIZE_EXISTING:-0}" == "1" ]]; then
    gate_d_args+=(--finalize-existing)
  fi
  if [[ -n "${VRX_GATE_D_STARTUP_STAGGER_S:-}" ]]; then
    gate_d_args+=(--startup-stagger-s "$VRX_GATE_D_STARTUP_STAGGER_S")
  fi
  if [[ -n "${VRX_GATE_D_OUTPUT:-}" ]]; then
    gate_d_args+=(--output "$VRX_GATE_D_OUTPUT")
  fi
  "${BCOD_PYTHON:?Set BCOD_PYTHON to the CodeNimbus x86_64 project interpreter}" validation/rl-campaign/run_vrx_gate_d.py "${gate_d_args[@]}"
else
  "${BCOD_PYTHON:?Set BCOD_PYTHON to the CodeNimbus x86_64 project interpreter}" validation/rl-campaign/run_vrx_gate_d.py
fi
