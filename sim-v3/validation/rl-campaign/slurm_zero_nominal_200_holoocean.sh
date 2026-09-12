#!/usr/bin/env bash
#SBATCH --job-name=zero200-holo
#SBATCH --partition=gpu-csl-cnice
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --gres=gpu:1
#SBATCH --time=02:30:00
#SBATCH --output=artifacts/common-suite/zero-current-nominal-200/cluster-logs/%x-%j.log
set -euo pipefail

repository_root="${BCOD_REPOSITORY_ROOT:?Set BCOD_REPOSITORY_ROOT}"
cd "$repository_root/sim-v3"
mkdir -p artifacts/common-suite/zero-current-nominal-200/cluster-logs
export BCOD_HOST_CLASS=cluster
export PYTHONPATH="$HOME/holoocean-kickoff/.venv-holoocean/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
exec .venv/bin/python validation/rl-campaign/evaluate_portable_checkpoint.py \
  --backend holoocean \
  --checkpoint artifacts/common-suite/native-three-policy-20260909/holoocean/model-final.zip \
  --output artifacts/common-suite/zero-current-nominal-200/holoocean \
  --episodes 200 --first-seed 42000 --device cpu \
  --disturbance-mode zero --holoocean-wind-mode off \
  --condition-contract artifacts/common-suite/zero-current-nominal-diagnostic.json \
  --allow-unconformant-diagnostic
