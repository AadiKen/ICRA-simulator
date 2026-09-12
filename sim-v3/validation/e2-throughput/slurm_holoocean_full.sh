#!/usr/bin/env bash
#SBATCH --job-name=e2-holo-full
#SBATCH --partition=gpu-csl-cnice
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=32
#SBATCH --gres=gpu:1
#SBATCH --exclusive
#SBATCH --time=02:30:00
#SBATCH --output=artifacts/e2-throughput/raw/e2-holo-full-%j.log
set -euo pipefail
root="${BCOD_REPOSITORY_ROOT:?Set BCOD_REPOSITORY_ROOT}/sim-v3"; cd "$root"; mkdir -p artifacts/e2-throughput/raw
prefix="artifacts/e2-throughput/raw/e2-holo-full-${SLURM_JOB_ID}"; hostname > "$prefix-hostname.txt"; lscpu > "$prefix-lscpu.txt"; free -b > "$prefix-memory.txt"; nvidia-smi -q > "$prefix-nvidia-smi.txt"; scontrol show job "$SLURM_JOB_ID" -dd > "$prefix-scontrol.txt"
export PYTHONPATH="$HOME/holoocean-kickoff/.venv-holoocean/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
read -r -a counts <<< "${E2_COUNTS:-1 2 4 8 16 32}"
output="${E2_OUTPUT:-artifacts/e2-throughput/holoocean-full.json}"
.venv/bin/python validation/e2-throughput/run_external_full.py --backend holoocean --counts "${counts[@]}" --output "$output"
scontrol show job "$SLURM_JOB_ID" -dd > "$prefix-scontrol-final.txt"
