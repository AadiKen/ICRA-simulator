#!/bin/bash
#SBATCH --job-name=ppo15-seeds
#SBATCH --partition=cpu
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=64
#SBATCH --time=24:00:00
#SBATCH --array=0-2
#SBATCH --output=artifacts/rl-campaign/surveyor/15-field-cluster-rerun/slurm-seed-%A_%a.log
set -euo pipefail
cd "${BCOD_REPO:?Set BCOD_REPO to the Delta checkout}"
.venv/bin/python validation/rl-campaign/run_15field_cluster_seed.py
