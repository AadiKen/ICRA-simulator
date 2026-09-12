#!/usr/bin/env bash
#SBATCH --job-name=portable-ppo-ff
#SBATCH --partition=gpu-a30-csl-cnice
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=32
#SBATCH --gres=gpu:nvidia_a30:1
#SBATCH --time=02:25:00
#SBATCH --output=artifacts/rl-campaign/training-runs/slurm-logs/%x-%j.log
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "usage: slurm_portable_feedforward_train.sh BACKEND TIMESTEPS [runner options...]" >&2
  exit 2
fi

backend="$1"
timesteps="$2"
shift 2

repository_root="${BCOD_REPOSITORY_ROOT:?Set BCOD_REPOSITORY_ROOT to the cluster checkout}"
python_bin="${BCOD_PYTHON:-$repository_root/sim-v3/.venv/bin/python}"
export BCOD_HOST_CLASS=cluster
export PATH="${CODENIMBUS_NODE24_BIN:-$HOME/.local/node-v24.20.0-linux-x64/bin}:$PATH"
if [[ -n "${BCOD_EXTRA_PYTHONPATH:-}" ]]; then
  export PYTHONPATH="${BCOD_EXTRA_PYTHONPATH}${PYTHONPATH:+:$PYTHONPATH}"
fi

cd "$repository_root/sim-v3"
if [[ "$backend" == "gazebo-harmonic" ]]; then
  apt-get update -qq
  DEBIAN_FRONTEND=noninteractive apt-get install -y -qq gz-harmonic \
    >/tmp/bcod-gazebo-install.log
  export BCOD_GAZEBO_NATIVE=1
fi
exec "$python_bin" validation/rl-campaign/train_portable_ppo_feedforward.py \
  --backend "$backend" \
  --timesteps "$timesteps" \
  "$@"
