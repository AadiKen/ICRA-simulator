#!/usr/bin/env bash
#SBATCH --job-name=zero200-gaz
#SBATCH --partition=gpu-csl-cnice
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --array=0-15%8
#SBATCH --time=02:30:00
#SBATCH --output=artifacts/common-suite/zero-current-nominal-200/cluster-logs/%x-%A_%a.log
set -euo pipefail

repository_root="${BCOD_REPOSITORY_ROOT:?Set BCOD_REPOSITORY_ROOT}"
cd "$repository_root/sim-v3"
mkdir -p artifacts/common-suite/zero-current-nominal-200/cluster-logs
export BCOD_HOST_CLASS=cluster BCOD_GAZEBO_NATIVE=1
node_bin="${CODENIMBUS_NODE24_BIN:-$HOME/.local/node-v24.20.0-linux-x64/bin}"
export PATH="$node_bin:$PATH"
shard="$SLURM_ARRAY_TASK_ID"
if (( shard < 8 )); then episodes=13; first_seed=$((42000 + shard * 13)); else episodes=12; first_seed=$((42104 + (shard - 8) * 12)); fi
command=(.venv/bin/python validation/rl-campaign/evaluate_portable_checkpoint.py \
  --backend gazebo-harmonic \
  --checkpoint artifacts/common-suite/native-three-policy-20260909/gazebo/model-final.zip \
  --output "artifacts/common-suite/zero-current-nominal-200/gazebo-shard-$shard" \
  --episodes "$episodes" --first-seed "$first_seed" --device cpu --disturbance-mode zero \
  --condition-contract artifacts/common-suite/zero-current-nominal-diagnostic.json \
  --allow-unconformant-diagnostic)
if command -v gz >/dev/null; then
  exec "${command[@]}"
fi
image="$HOME/.cache/enroot/gz-harmonic-noble.sqsh"
[[ -s "$image" ]] || { echo "missing Gazebo Harmonic enroot image: $image" >&2; exit 1; }
enroot_base="${SLURM_TMPDIR:-/tmp}/${USER}-zero200-enroot-${SLURM_ARRAY_JOB_ID}-${shard}"
mkdir -p "$enroot_base"/{runtime,temp,data}
export ENROOT_RUNTIME_PATH="$enroot_base/runtime" ENROOT_TEMP_PATH="$enroot_base/temp" ENROOT_DATA_PATH="$enroot_base/data"
container="zero200-gazebo-${SLURM_ARRAY_JOB_ID}-${shard}"
enroot create -n "$container" "$image"
quoted_command=$(printf ' %q' "${command[@]}")
exec enroot start --root -m "$repository_root:$repository_root" -m "$HOME:$HOME" "$container" \
  sh -lc "cd '$repository_root/sim-v3'; export BCOD_HOST_CLASS=cluster BCOD_GAZEBO_NATIVE=1 PATH='$node_bin':\$PATH; exec$quoted_command"
