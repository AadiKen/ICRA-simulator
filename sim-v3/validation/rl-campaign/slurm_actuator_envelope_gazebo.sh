#!/usr/bin/env bash
#SBATCH --job-name=act-env-gaz
#SBATCH --partition=gpu-csl-cnice
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --time=00:45:00
#SBATCH --output=artifacts/common-suite/actuator-envelope/cluster-logs/%x-%j.log
set -euo pipefail
root="${BCOD_REPOSITORY_ROOT:?Set BCOD_REPOSITORY_ROOT}/sim-v3"
cd "$root"; mkdir -p artifacts/common-suite/actuator-envelope/cluster-logs
node_bin="${CODENIMBUS_NODE24_BIN:-$HOME/.local/node-v24.20.0-linux-x64/bin}"
image="$HOME/.cache/enroot/gz-harmonic-noble.sqsh"
[[ -s "$image" ]] || { echo "missing image: $image" >&2; exit 1; }
base="${SLURM_TMPDIR:-/tmp}/${USER}-act-envelope-${SLURM_JOB_ID}"
mkdir -p "$base"/{runtime,temp,data}
export ENROOT_RUNTIME_PATH="$base/runtime" ENROOT_TEMP_PATH="$base/temp" ENROOT_DATA_PATH="$base/data"
container="act-envelope-gaz-${SLURM_JOB_ID}"
enroot create -n "$container" "$image"
exec enroot start --root -m "$root:$root" -m "$HOME:$HOME" "$container" sh -lc \
  "cd '$root'; export BCOD_HOST_CLASS=cluster BCOD_GAZEBO_NATIVE=1 PATH='$node_bin':\$PATH; exec .venv/bin/python validation/rl-campaign/calibrate_actuator_envelope.py --backend gazebo-harmonic --condition-contract artifacts/common-suite/zero-current-nominal-diagnostic.json --output artifacts/common-suite/actuator-envelope/gazebo-harmonic.json"
