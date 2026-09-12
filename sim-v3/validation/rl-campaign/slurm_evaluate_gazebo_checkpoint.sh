#!/usr/bin/env bash
#SBATCH --job-name=gazebo-ppo-eval
#SBATCH --partition=gpu-a30-csl-cnice
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=8
#SBATCH --gres=gpu:nvidia_a30:1
#SBATCH --time=02:30:00
#SBATCH --output=artifacts/rl-campaign/training-runs/slurm-logs/%x-%j.log
set -euo pipefail

if [[ $# -ne 2 ]]; then
  echo "usage: $0 CHECKPOINT OUTPUT_DIRECTORY" >&2
  exit 2
fi

repository_root="${BCOD_REPOSITORY_ROOT:?Set BCOD_REPOSITORY_ROOT}"
export BCOD_HOST_CLASS=cluster
export BCOD_GAZEBO_NATIVE=1
export PATH="${CODENIMBUS_NODE24_BIN:-$HOME/.local/node-v24.20.0-linux-x64/bin}:$PATH"
apt-get update -qq
DEBIAN_FRONTEND=noninteractive apt-get install -y -qq gz-harmonic >/tmp/bcod-gazebo-evaluation-install.log
cd "$repository_root/sim-v3"
exec .venv/bin/python validation/rl-campaign/evaluate_portable_checkpoint.py \
  --backend gazebo-harmonic \
  --checkpoint "$1" \
  --output "$2" \
  --episodes 50 \
  --first-seed 10000 \
  --device cpu \
  --disturbance-mode zero
