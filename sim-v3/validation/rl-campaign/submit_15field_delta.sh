#!/bin/bash
set -euo pipefail

: "${BCOD_REPO:?Set BCOD_REPO to the absolute CodeNimbus checkout path}"
: "${SLURM_ACCOUNT:?Set SLURM_ACCOUNT to the CodeNimbus allocation account}"

cd "$BCOD_REPO"
mkdir -p artifacts/rl-campaign/surveyor/15-field-cluster-rerun

preflight_job="$(sbatch --parsable --account="$SLURM_ACCOUNT" --export=ALL,BCOD_REPO="$BCOD_REPO" validation/rl-campaign/slurm_15field_preflight.sh)"
training_job="$(sbatch --parsable --account="$SLURM_ACCOUNT" --dependency="afterok:$preflight_job" --export=ALL,BCOD_REPO="$BCOD_REPO" validation/rl-campaign/slurm_15field_train.sh)"
summary_job="$(sbatch --parsable --account="$SLURM_ACCOUNT" --dependency="afterok:$training_job" --export=ALL,BCOD_REPO="$BCOD_REPO" validation/rl-campaign/slurm_15field_summarize.sh)"

printf 'preflight_job=%s\ntraining_array_job=%s\nsummary_job=%s\n' "$preflight_job" "$training_job" "$summary_job"
