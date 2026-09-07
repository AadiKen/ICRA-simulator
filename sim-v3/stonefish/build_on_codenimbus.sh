#!/usr/bin/env bash
set -euo pipefail

stonefish_source="${STONEFISH_SOURCE:-$HOME/stonefish-src}"
stonefish_prefix="${STONEFISH_PREFIX:-$HOME/stonefish-install}"
dependency_prefix="${STONEFISH_DEPS:-$HOME/stonefish-deps}"
bridge_build="${STONEFISH_BRIDGE_BUILD:-$HOME/stonefish-vehicle-a-build}"
project_source="${1:-$PWD}"
seed_patch="$project_source/patches/0001-sensor-rng-seed-hook.patch"

if git -C "$stonefish_source" apply --reverse --check "$seed_patch" 2>/dev/null; then
    echo "Stonefish seed patch already applied"
else
    git -C "$stonefish_source" apply --check "$seed_patch"
    git -C "$stonefish_source" apply "$seed_patch"
fi

export CMAKE_PREFIX_PATH="$dependency_prefix:$stonefish_prefix"
export CPLUS_INCLUDE_PATH="$dependency_prefix/include${CPLUS_INCLUDE_PATH:+:$CPLUS_INCLUDE_PATH}"
export LD_LIBRARY_PATH="$dependency_prefix/lib:$stonefish_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

cmake -S "$stonefish_source" -B "$HOME/stonefish-build-install" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$stonefish_prefix" \
    -DCMAKE_PREFIX_PATH="$dependency_prefix" \
    -DBUILD_TESTS=OFF \
    -DEMBED_RESOURCES=ON
cmake --build "$HOME/stonefish-build-install" -j "$(nproc)"
cmake --install "$HOME/stonefish-build-install"

cmake -S "$project_source" -B "$bridge_build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="$dependency_prefix;$stonefish_prefix"
cmake --build "$bridge_build" -j "$(nproc)"

echo "Bridge executable: $bridge_build/stonefish_vehicle_a_bridge"
