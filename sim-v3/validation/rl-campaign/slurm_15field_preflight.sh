#!/bin/bash
#SBATCH --job-name=ppo15-preflight
#SBATCH --partition=gpu-a30-csl-cnice
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=32
#SBATCH --gres=gpu:nvidia_a30:1
#SBATCH --time=00:20:00
#SBATCH --output=artifacts/rl-campaign/surveyor/15-field-cluster-rerun/slurm-preflight-%j.log
set -euo pipefail
export PATH="${CODENIMBUS_NODE24_BIN:-/opt/nodejs/24/bin}:$PATH"
cd "${BCOD_REPO:?Set BCOD_REPO to the CodeNimbus checkout}"

node_version="$(node --version)"
node_major="${node_version#v}"
node_major="${node_major%%.*}"
if [[ ! "$node_major" =~ ^[0-9]+$ ]] || (( node_major < 24 )); then
  printf 'Node.js >=24 is required; found %s\n' "$node_version" >&2
  exit 1
fi
printf 'node=%s\n' "$node_version"
.venv/bin/python -c 'import sb3_contrib; print(sb3_contrib.__version__)'
.venv/bin/python validation/rl-campaign/cluster_preflight_15field.py
