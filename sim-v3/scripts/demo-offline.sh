#!/usr/bin/env bash
set -euo pipefail
npm run test:migration-gate
npm run campaign:vehicle-c-allocation
npm run run:usv-bench-36
npm run run:offline-baselines
npm run test:ui
npm run generate:release
echo "Offline BCOD-Sim evidence and research UI reproduced. External validation gates remain explicit."
