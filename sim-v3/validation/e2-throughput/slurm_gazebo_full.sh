#!/usr/bin/env bash
#SBATCH --job-name=e2-gazebo-full
#SBATCH --partition=gpu-csl-cnice
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=32
#SBATCH --gres=gpu:1
#SBATCH --exclusive
#SBATCH --time=02:30:00
#SBATCH --output=artifacts/e2-throughput/raw/e2-gazebo-full-%j.log
set -euo pipefail
root="${BCOD_REPOSITORY_ROOT:?Set BCOD_REPOSITORY_ROOT}/sim-v3"; cd "$root"; mkdir -p artifacts/e2-throughput/raw
prefix="artifacts/e2-throughput/raw/e2-gazebo-full-${SLURM_JOB_ID}"; hostname > "$prefix-hostname.txt"; lscpu > "$prefix-lscpu.txt"; free -b > "$prefix-memory.txt"; nvidia-smi -q > "$prefix-nvidia-smi.txt"; scontrol show job "$SLURM_JOB_ID" -dd > "$prefix-scontrol.txt"
export BCOD_GAZEBO_NATIVE=1; export PATH="${CODENIMBUS_NODE24_BIN:-$HOME/.local/node-v24.20.0-linux-x64/bin}:$PATH"
command -v gz >/dev/null || { apt-get update -qq; DEBIAN_FRONTEND=noninteractive apt-get install -y -qq gz-harmonic; }
.venv/bin/python validation/e2-throughput/run_external_full.py --backend gazebo-harmonic --counts 1 2 4 8 16 32 --output artifacts/e2-throughput/gazebo-full.json
scontrol show job "$SLURM_JOB_ID" -dd > "$prefix-scontrol-final.txt"
