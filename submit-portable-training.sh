#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "usage: $0 BACKEND TIMESTEPS [runner options...]" >&2
  exit 2
fi

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$repository_root/sim-v3/artifacts/rl-campaign/training-runs/slurm-logs"
backend="$1"

account_args=()
if [[ -n "${SLURM_ACCOUNT:-}" ]]; then
  account_args=(--account="$SLURM_ACCOUNT")
fi

container_args=()
if [[ "$backend" == "gazebo-harmonic" ]]; then
  gazebo_image="${BCOD_GAZEBO_SQSH:-$HOME/containers/vrx-surveyor-patched-v3.0.1.sqsh}"
  if [[ ! -f "$gazebo_image" ]]; then
    echo "Gazebo base container was not found at: $gazebo_image" >&2
    exit 2
  fi
  container_args=(
    --container-image="$gazebo_image"
    --container-remap-root
    --container-writable
    --container-mounts="$HOME:$HOME"
    --container-workdir="$repository_root/sim-v3"
  )
  has_environment_count=false
  for argument in "$@"; do
    if [[ "$argument" == "--n-envs" || "$argument" == --n-envs=* ]]; then
      has_environment_count=true
    fi
  done
  if [[ "$has_environment_count" == false ]]; then
    set -- "$@" --n-envs 8
  fi
fi

sbatch \
  "${account_args[@]}" \
  "${container_args[@]}" \
  --export=ALL,BCOD_REPOSITORY_ROOT="$repository_root" \
  "$repository_root/sim-v3/validation/rl-campaign/slurm_portable_train.sh" \
  "$@"
