#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd "$(dirname "$0")/../../.." && pwd)
stock_source=leadcat/vrx:v3.0.1
stock_image=leadcat/vrx:stock-v3.0.1
surveyor_stock_physics_image=leadcat/vrx:surveyor-stock-physics-v3.0.1
patched_image=leadcat/vrx:surveyor-patched-v3.0.1

# A tag operation creates no layer: the stock identity remains byte-for-byte the
# pinned, unmodified VRX build.
docker tag "$stock_source" "$stock_image"
docker build --platform linux/amd64 \
  --file "$repo_root/validation/rl-campaign/ports/Dockerfile.vrx-surveyor-stock-physics" \
  --tag "$surveyor_stock_physics_image" \
  "$repo_root"
docker build --platform linux/amd64 \
  --file "$repo_root/validation/rl-campaign/ports/Dockerfile.vrx-surveyor-patched" \
  --tag "$patched_image" \
  "$repo_root"

docker image inspect "$stock_source" "$stock_image" "$surveyor_stock_physics_image" "$patched_image" \
  --format '{{.RepoTags}} {{.Id}} {{index .Config.Labels "org.leadcat.vrx.configuration"}} {{index .Config.Labels "org.leadcat.paper.baseline-eligible"}} {{index .Config.Labels "org.leadcat.vrx.custom-dynamics"}}'
