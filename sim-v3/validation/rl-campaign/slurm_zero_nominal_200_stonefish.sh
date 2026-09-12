#!/usr/bin/env bash
#SBATCH --job-name=zero200-stone
#SBATCH --partition=gpu-csl-cnice
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --gres=gpu:1
#SBATCH --time=00:45:00
#SBATCH --output=artifacts/common-suite/zero-current-nominal-200/cluster-logs/%x-%j.log
set -euo pipefail

repository_root="${BCOD_REPOSITORY_ROOT:?Set BCOD_REPOSITORY_ROOT}"
cd "$repository_root/sim-v3"
mkdir -p artifacts/common-suite/zero-current-nominal-200/cluster-logs
export BCOD_HOST_CLASS=cluster
exec .venv/bin/python validation/rl-campaign/evaluate_portable_checkpoint.py \
  --backend stonefish \
  --checkpoint artifacts/common-suite/native-three-policy-20260909/stonefish/model-final.zip \
  --output artifacts/common-suite/zero-current-nominal-200/stonefish \
  --episodes 200 --first-seed 42000 --device cpu --disturbance-mode zero \
  --stonefish-executable /mnt/shared/gpfs/home/aadik3/stonefish-vehicle-a-build/stonefish_vehicle_a_bridge \
  --stonefish-data-dir /mnt/shared/gpfs/home/aadik3/stonefish-src/Tests/Data \
  --stonefish-lib /mnt/shared/gpfs/home/aadik3/stonefish-install/lib \
  --stonefish-deps-lib /mnt/shared/gpfs/home/aadik3/stonefish-deps/lib \
  --stonefish-physics-threads 1 \
  --condition-contract artifacts/common-suite/zero-current-nominal-diagnostic.json \
  --allow-unconformant-diagnostic
