#!/usr/bin/env bash
#SBATCH --job-name=e2-bcod-full
#SBATCH --partition=gpu-csl-cnice
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=32
#SBATCH --gres=gpu:1
#SBATCH --exclusive
#SBATCH --time=01:00:00
#SBATCH --output=artifacts/e2-throughput/raw/e2-bcod-full-%j.log
set -euo pipefail
root="${BCOD_REPOSITORY_ROOT:?Set BCOD_REPOSITORY_ROOT}/sim-v3"; cd "$root"
mkdir -p artifacts/e2-throughput/raw
hostname > "artifacts/e2-throughput/raw/e2-bcod-full-${SLURM_JOB_ID}-hostname.txt"
lscpu > "artifacts/e2-throughput/raw/e2-bcod-full-${SLURM_JOB_ID}-lscpu.txt"
free -b > "artifacts/e2-throughput/raw/e2-bcod-full-${SLURM_JOB_ID}-memory.txt"
nvidia-smi -q > "artifacts/e2-throughput/raw/e2-bcod-full-${SLURM_JOB_ID}-nvidia-smi.txt"
scontrol show job "$SLURM_JOB_ID" -dd > "artifacts/e2-throughput/raw/e2-bcod-full-${SLURM_JOB_ID}-scontrol.txt"
export PATH="${CODENIMBUS_NODE24_BIN:-$HOME/.local/node-v24.20.0-linux-x64/bin}:$PATH"
.venv/bin/python validation/e2-throughput/run_bcod_full.py --output artifacts/e2-throughput/bcod-full.json
scontrol show job "$SLURM_JOB_ID" -dd > "artifacts/e2-throughput/raw/e2-bcod-full-${SLURM_JOB_ID}-scontrol-final.txt"
