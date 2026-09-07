#!/bin/bash
#SBATCH --job-name=ppo15-seeds
#SBATCH --partition=gpu-a30-csl-cnice
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=32
#SBATCH --gres=gpu:nvidia_a30:1
#SBATCH --time=24:00:00
#SBATCH --array=0-2
#SBATCH --output=artifacts/rl-campaign/surveyor/15-field-cluster-rerun/slurm-seed-%A_%a.log
set -euo pipefail
export PATH="${CODENIMBUS_NODE24_BIN:-/opt/nodejs/24/bin}:$PATH"
cd "${BCOD_REPO:?Set BCOD_REPO to the CodeNimbus checkout}"
.venv/bin/python validation/rl-campaign/run_15field_cluster_seed.py
