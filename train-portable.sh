#!/usr/bin/env bash
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
sim_root="$repository_root/sim-v3"
python_bin="${BCOD_PYTHON:-$sim_root/.venv/bin/python}"

if [[ ! -x "$python_bin" ]]; then
  echo "Training Python was not found at: $python_bin" >&2
  echo "Set BCOD_PYTHON to the Python executable on CodeNimbus." >&2
  exit 2
fi

cd "$sim_root"
exec "$python_bin" validation/rl-campaign/train_portable_ppo.py "$@"
