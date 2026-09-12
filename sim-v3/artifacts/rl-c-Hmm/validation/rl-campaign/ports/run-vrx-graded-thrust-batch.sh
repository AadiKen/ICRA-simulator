#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd "$(dirname "$0")/../../.." && pwd)
patched_image=leadcat/vrx:surveyor-patched-v3.0.1
configuration=$(docker image inspect "$patched_image" --format '{{index .Config.Labels "org.leadcat.vrx.configuration"}}')
eligibility=$(docker image inspect "$patched_image" --format '{{index .Config.Labels "org.leadcat.paper.baseline-eligible"}}')
[[ "$configuration" == "vrx:surveyor-patched" && "$eligibility" == "false" ]] || {
  echo "refusing ambiguous VRX image: expected patched, baseline-ineligible labels" >&2
  exit 2
}
label=${1:?scale label required}
action_scale=${2:?action scale required}
shift 2

run_one() {
  local seed=$1
  local domain=$((seed % 200 + 150))
  local episode="$repo_root/artifacts/rl-campaign/vrx-gate7-full/graded/scale-$label/seed-$seed"
  node --experimental-strip-types "$repo_root/validation/rl-campaign/ports/prepare-vrx-episode.ts" "$seed" "$episode" 1 "$action_scale" >/dev/null
  docker run --rm --platform linux/amd64 --name "vrx-g7-grade-$label-$seed" \
    -v "$episode:/episode" \
    -v "$repo_root/validation/rl-campaign/ports:/ports:ro" \
    -e "ROS_DOMAIN_ID=$domain" -e "GZ_PARTITION=vrx-g7-grade-$label-$seed" \
    -e GZ_SIM_RESOURCE_PATH=/episode/models:/opt/vrx_ws/install/share/vrx_gz/models \
    -e GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/leadcat/vrx-surveyor-patched/lib:/opt/vrx_ws/install/lib \
    -e LD_LIBRARY_PATH=/opt/leadcat/vrx-surveyor-patched/lib:/opt/vrx_ws/install/lib:/opt/ros/jazzy/lib \
    --entrypoint bash "$patched_image" \
    /ports/run-vrx-gate7-episode.sh /episode /episode/raw.jsonl
}

export repo_root label action_scale patched_image
export -f run_one
printf '%s\n' "$@" | xargs -n 1 -P 4 bash -c 'run_one "$1"' _
