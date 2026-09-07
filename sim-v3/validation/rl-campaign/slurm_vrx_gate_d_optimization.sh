#!/bin/bash
#SBATCH --job-name=vrx-gated-opt
#SBATCH --partition=gpu-a30-csl-cnice
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=32
#SBATCH --time=00:30:00
#SBATCH --output=artifacts/rl-campaign/vrx-gate-d-optimization-%j.log
set -u
export PATH="${CODENIMBUS_NODE_BIN:?Set CODENIMBUS_NODE_BIN to the native Node bin directory}:$PATH"
cd "${BCOD_REPO:?Set BCOD_REPO to the isolated CodeNimbus benchmark checkout}"
python_bin="${BCOD_PYTHON:?Set BCOD_PYTHON to the CodeNimbus x86_64 project interpreter}"
baseline="artifacts/rl-campaign/vrx-gate-d-throughput.json"
staggered="artifacts/rl-campaign/vrx-gate-d-16-staggered.json"
lean="artifacts/rl-campaign/vrx-gate-d-16-staggered-lean.json"
cp "$baseline" "$staggered"
cp "$baseline" "$lean"
VRX_LEAN_TRANSPORT=0 "$python_bin" validation/rl-campaign/run_vrx_gate_d.py \
  --resume-screening --force-selected-count 16 --startup-stagger-s 1 --output "$staggered"
staggered_status=$?
VRX_LEAN_TRANSPORT=1 "$python_bin" validation/rl-campaign/run_vrx_gate_d.py \
  --resume-screening --force-selected-count 16 --startup-stagger-s 1 --output "$lean"
lean_status=$?
echo "staggered_status=$staggered_status lean_status=$lean_status"
exit $((staggered_status || lean_status))
