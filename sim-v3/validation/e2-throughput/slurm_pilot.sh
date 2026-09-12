#!/usr/bin/env bash
#SBATCH --job-name=bcod-e2-pilot
#SBATCH --partition=gpu-a30-csl-cnice
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=32
#SBATCH --gres=gpu:nvidia_a30:1
#SBATCH --time=00:15:00
#SBATCH --output=artifacts/e2-throughput/slurm-%j.log
set -euo pipefail

repository_root="${BCOD_REPOSITORY_ROOT:?Set BCOD_REPOSITORY_ROOT to the cluster checkout}"
export PATH="${CODENIMBUS_NODE24_BIN:-$HOME/.local/node-v24.20.0-linux-x64/bin}:$PATH"
cd "$repository_root/sim-v3"
mkdir -p artifacts/e2-throughput
.venv/bin/python validation/e2-throughput/run.py \
  --batch-sizes 1 2 4 8 16 32 64 128 256 512 1024 2048 4096 8192 \
  --sample-steps 16 --warmup-steps 4 --rollout-steps 300 --full-rollouts 13 \
  --output artifacts/e2-throughput/a30-heuristic-pilot.json
