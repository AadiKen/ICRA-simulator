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
mode=${1:?on or off required}
shift
scale=1
domain_offset=50
if [[ "$mode" == "off" ]]; then scale=0; domain_offset=100; fi

run_one() {
  local seed=$1
  local domain=$((seed % 200 + domain_offset))
  local episode="$repo_root/artifacts/rl-campaign/vrx-gate7-full/$mode/seed-$seed"
  node --experimental-strip-types "$repo_root/validation/rl-campaign/ports/prepare-vrx-episode.ts" "$seed" "$episode" "$scale" >/dev/null
  docker run --rm --platform linux/amd64 --name "vrx-g7-$mode-$seed" \
    -v "$episode:/episode" \
    -v "$repo_root/validation/rl-campaign/ports:/ports:ro" \
    -e "ROS_DOMAIN_ID=$domain" -e "GZ_PARTITION=vrx-g7-$mode-$seed" \
    -e GZ_SIM_RESOURCE_PATH=/episode/models:/opt/vrx_ws/install/share/vrx_gz/models \
    -e GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/leadcat/vrx-surveyor-patched/lib:/opt/vrx_ws/install/lib \
    -e LD_LIBRARY_PATH=/opt/leadcat/vrx-surveyor-patched/lib:/opt/vrx_ws/install/lib:/opt/ros/jazzy/lib \
    --entrypoint bash "$patched_image" \
    /ports/run-vrx-gate7-episode.sh /episode /episode/raw.jsonl
}

export repo_root mode scale domain_offset patched_image
export -f run_one
printf '%s\n' "$@" | xargs -n 1 -P 4 bash -c 'run_one "$1"' _
