#!/bin/bash
#SBATCH --job-name=vrx-gate-d
#SBATCH --partition=gpu-a30-csl-cnice
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=32
#SBATCH --gres=gpu:nvidia_a30:1
#SBATCH --time=02:20:00
#SBATCH --output=artifacts/rl-campaign/vrx-gate-d-%j.log
set -euo pipefail
export PATH="${CODENIMBUS_NODE_BIN:?Set CODENIMBUS_NODE_BIN to the native Node bin directory}:$PATH"
cd "${BCOD_REPO:?Set BCOD_REPO to the isolated CodeNimbus benchmark checkout}"
"${BCOD_PYTHON:?Set BCOD_PYTHON to the CodeNimbus x86_64 project interpreter}" validation/rl-campaign/run_vrx_gate_d.py
