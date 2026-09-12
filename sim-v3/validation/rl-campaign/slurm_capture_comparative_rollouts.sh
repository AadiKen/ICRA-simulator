#!/usr/bin/env bash
#SBATCH --job-name=comparative-rollouts
#SBATCH --partition=gpu-a30-csl-cnice
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=8
#SBATCH --gres=gpu:nvidia_a30:1
#SBATCH --time=02:20:00
#SBATCH --output=artifacts/rl-campaign/training-runs/slurm-logs/%x-%j.log
set -euo pipefail
backend="$1"
repo="${BCOD_REPOSITORY_ROOT:?Set BCOD_REPOSITORY_ROOT}"
export BCOD_HOST_CLASS=cluster
export PATH="${CODENIMBUS_NODE24_BIN:-$HOME/.local/node-v24.20.0-linux-x64/bin}:$PATH"
cd "$repo/sim-v3"
if [[ "$backend" == "holoocean" ]]; then
  export PYTHONPATH="$HOME/holoocean-kickoff/.venv-holoocean/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
fi
if [[ "$backend" == "gazebo-harmonic" ]]; then
  image="$HOME/.cache/enroot/gz-harmonic-noble.sqsh"
  base="${SLURM_TMPDIR:-/tmp}/${USER}-comparative-rollouts-${SLURM_JOB_ID}"
  mkdir -p "$base"/{runtime,temp,data}
  export ENROOT_RUNTIME_PATH="$base/runtime" ENROOT_TEMP_PATH="$base/temp" ENROOT_DATA_PATH="$base/data"
  container="comparative-rollouts-${SLURM_JOB_ID}"
  enroot create -n "$container" "$image"
  exec enroot start --root -m "$repo:$repo" -m "$HOME:$HOME" "$container" sh -lc \
    "cd '$repo/sim-v3'; export BCOD_HOST_CLASS=cluster BCOD_GAZEBO_NATIVE=1 PATH='$PATH'; exec .venv/bin/python validation/rl-campaign/capture_comparative_rollouts.py --backend gazebo-harmonic --output artifacts/rl-campaign/comparative-rollouts-v2"
fi
exec .venv/bin/python validation/rl-campaign/capture_comparative_rollouts.py \
  --backend "$backend" --output artifacts/rl-campaign/comparative-rollouts-v2
