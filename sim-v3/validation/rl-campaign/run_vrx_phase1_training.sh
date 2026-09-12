#!/usr/bin/env bash
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repository_root"

python_bin="${BCOD_PYTHON:-$repository_root/.venv/bin/python}"
parallel_envs="${BCOD_VRX_N_ENVS:-4}"
run_name="${BCOD_VRX_RUN_NAME:-vrx-path-v2-phase1-$(date -u +%Y%m%dT%H%M%SZ)}"
run_dir="$repository_root/artifacts/rl-campaign/training-runs/$run_name"

if [[ ! -x "$python_bin" ]]; then
  echo "Training Python was not found at $python_bin" >&2
  exit 2
fi
if [[ -e "$run_dir" ]]; then
  echo "Refusing to overwrite existing run directory: $run_dir" >&2
  exit 2
fi
if docker ps --format '{{.Names}}' | grep -q '^icra27-gazebo-gym-'; then
  echo "A VRX/Gazebo runtime is already active; stop or account for it before launching." >&2
  exit 2
fi

exec "$python_bin" validation/rl-campaign/train_portable_ppo_feedforward.py \
  --backend vrx \
  --runtime-command "$python_bin validation/rl-campaign/ports/vrx_gym_runtime.py" \
  --final-leg-curriculum \
  --timesteps 1500000 \
  --n-envs "$parallel_envs" \
  --checkpoint-freq 250000 \
  --curve-eval-freq 250000 \
  --curve-eval-episodes 50 \
  --eval-first-seed 30000 \
  --output "$run_dir"
