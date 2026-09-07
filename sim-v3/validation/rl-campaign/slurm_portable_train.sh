#!/usr/bin/env bash
#SBATCH --job-name=portable-ppo
#SBATCH --partition=gpu-a30-csl-cnice
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=32
#SBATCH --gres=gpu:nvidia_a30:1
#SBATCH --time=24:00:00
#SBATCH --output=sim-v3/artifacts/rl-campaign/training-runs/slurm-logs/%x-%j.log
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "usage: slurm_portable_train.sh BACKEND TIMESTEPS [runner options...]" >&2
  exit 2
fi

backend="$1"
timesteps="$2"
shift 2

repository_root="${BCOD_REPOSITORY_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
export BCOD_HOST_CLASS=cluster
export PATH="${CODENIMBUS_NODE24_BIN:-/opt/nodejs/24/bin}:$PATH"

exec "$repository_root/train-portable.sh" \
  --backend "$backend" \
  --timesteps "$timesteps" \
  "$@"
