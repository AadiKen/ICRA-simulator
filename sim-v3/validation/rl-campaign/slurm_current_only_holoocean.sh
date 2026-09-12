#!/usr/bin/env bash
#SBATCH --job-name=gate1-current-holo
#SBATCH --partition=gpu-csl-cnice
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --gres=gpu:1
#SBATCH --time=02:30:00
#SBATCH --output=artifacts/common-suite/gate1-current-only-saturation/holoocean-%j.log
set -euo pipefail

repository_root="${BCOD_REPOSITORY_ROOT:?Set BCOD_REPOSITORY_ROOT}"
condition_contract="${1:-artifacts/common-suite/current-only-nominal-probe.json}"
output_directory="${2:-artifacts/common-suite/gate1-current-only-saturation/holoocean}"
cd "$repository_root/sim-v3"
export PYTHONPATH="$HOME/holoocean-kickoff/.venv-holoocean/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
exec .venv/bin/python validation/rl-campaign/evaluate_portable_checkpoint.py \
  --backend holoocean \
  --checkpoint artifacts/common-suite/native-three-policy-20260909/holoocean/model-final.zip \
  --output "$output_directory" \
  --episodes 200 \
  --first-seed 42000 \
  --device cpu \
  --disturbance-mode seeded \
  --holoocean-wind-mode off \
  --condition-contract "$condition_contract"
