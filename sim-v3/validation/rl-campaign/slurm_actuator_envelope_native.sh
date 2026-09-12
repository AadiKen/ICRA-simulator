#!/usr/bin/env bash
#SBATCH --job-name=act-envelope
#SBATCH --partition=gpu-csl-cnice
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --gres=gpu:1
#SBATCH --time=00:45:00
#SBATCH --output=artifacts/common-suite/actuator-envelope/cluster-logs/%x-%j.log
set -euo pipefail
backend="${1:?backend required}"
root="${BCOD_REPOSITORY_ROOT:?Set BCOD_REPOSITORY_ROOT}/sim-v3"
cd "$root"; mkdir -p artifacts/common-suite/actuator-envelope/cluster-logs
common=(--backend "$backend" --condition-contract artifacts/common-suite/zero-current-nominal-diagnostic.json --output "artifacts/common-suite/actuator-envelope/$backend.json")
if [[ "$backend" == holoocean ]]; then
  export PYTHONPATH="$HOME/holoocean-kickoff/.venv-holoocean/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
  exec .venv/bin/python validation/rl-campaign/calibrate_actuator_envelope.py "${common[@]}"
elif [[ "$backend" == stonefish ]]; then
  exec .venv/bin/python validation/rl-campaign/calibrate_actuator_envelope.py "${common[@]}" \
    --stonefish-executable /mnt/shared/gpfs/home/aadik3/stonefish-vehicle-a-build/stonefish_vehicle_a_bridge \
    --stonefish-data-dir /mnt/shared/gpfs/home/aadik3/stonefish-src/Tests/Data \
    --stonefish-lib /mnt/shared/gpfs/home/aadik3/stonefish-install/lib \
    --stonefish-deps-lib /mnt/shared/gpfs/home/aadik3/stonefish-deps/lib
else
  echo "unsupported backend: $backend" >&2; exit 2
fi
