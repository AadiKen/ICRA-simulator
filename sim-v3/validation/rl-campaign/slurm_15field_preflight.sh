#!/bin/bash
#SBATCH --job-name=ppo15-preflight
#SBATCH --partition=cpu
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=64
#SBATCH --time=00:20:00
#SBATCH --output=artifacts/rl-campaign/surveyor/15-field-cluster-rerun/slurm-preflight-%j.log
set -euo pipefail
cd "${BCOD_REPO:?Set BCOD_REPO to the Delta checkout}"
.venv/bin/python -c 'import sb3_contrib; print(sb3_contrib.__version__)'
.venv/bin/python validation/rl-campaign/cluster_preflight_15field.py
