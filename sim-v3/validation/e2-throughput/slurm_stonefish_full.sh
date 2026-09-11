#!/usr/bin/env bash
#SBATCH --job-name=e2-stone-full
#SBATCH --partition=gpu-csl-cnice
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=32
#SBATCH --gres=gpu:1
#SBATCH --exclusive
#SBATCH --time=02:00:00
#SBATCH --output=artifacts/e2-throughput/raw/e2-stone-full-%j.log
set -euo pipefail
root="${BCOD_REPOSITORY_ROOT:?Set BCOD_REPOSITORY_ROOT}/sim-v3"; cd "$root"; mkdir -p artifacts/e2-throughput/raw
prefix="artifacts/e2-throughput/raw/e2-stone-full-${SLURM_JOB_ID}"; hostname > "$prefix-hostname.txt"; lscpu > "$prefix-lscpu.txt"; free -b > "$prefix-memory.txt"; nvidia-smi -q > "$prefix-nvidia-smi.txt"; scontrol show job "$SLURM_JOB_ID" -dd > "$prefix-scontrol.txt"
.venv/bin/python validation/e2-throughput/run_external_full.py --backend stonefish --counts 1 2 4 8 16 32 --output artifacts/e2-throughput/stonefish-full.json
scontrol show job "$SLURM_JOB_ID" -dd > "$prefix-scontrol-final.txt"
