#!/bin/bash
#SBATCH --job-name=ppo15-summary
#SBATCH --partition=gpu-a30-csl-cnice
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --time=00:10:00
#SBATCH --output=artifacts/rl-campaign/surveyor/15-field-cluster-rerun/slurm-summary-%j.log
set -euo pipefail
export PATH="${CODENIMBUS_NODE24_BIN:-/opt/nodejs/24/bin}:$PATH"
cd "${BCOD_REPO:?Set BCOD_REPO to the CodeNimbus checkout}"
.venv/bin/python validation/rl-campaign/summarize_15field_cluster.py
