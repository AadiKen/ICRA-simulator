# Portable training runner

`train_portable_ppo.py` is the single training entry point for bcod-sim,
Gazebo Harmonic, HoloOcean, and Stonefish. It uses the same frozen RecurrentPPO
configuration for all three. The requested budget is always supplied with
`--timesteps`.

On CodeNimbus, commands can be launched from the checkout root
(`~/ICRA-simulator`). No `cd sim-v3` or npm lookup is needed:

```bash
./train-portable.sh --backend bcod-sim --timesteps 300000 \
  --eval-episodes 50 --device cuda
```

To submit the same run to Slurm (one A30, 32 CPUs, 24 hours):

```bash
./submit-portable-training.sh bcod-sim 300000 \
  --eval-episodes 50 --device cuda --n-envs 32
```

Set `SLURM_ACCOUNT` before submission if the cluster requires an explicit
allocation account. Set `BCOD_PYTHON` if the training environment is not at
`sim-v3/.venv/bin/python`.

Alternatively, from `sim-v3`, run:

```bash
# bcod-sim
npm run train:portable -- --backend bcod-sim --timesteps 3000000 \
  --eval-episodes 50 --device cpu

# Gazebo Harmonic (uses the repository's pinned Docker runtime automatically)
npm run train:portable -- --backend gazebo-harmonic --timesteps 3000000 \
  --eval-episodes 50 --device cpu

# HoloOcean
npm run train:portable -- --backend holoocean --timesteps 3000000 \
  --eval-episodes 50 --device cuda

# Stonefish (PPO on GPU; headless simulator processes remain on CPU)
npm run train:portable -- --backend stonefish --timesteps 3000000 \
  --eval-episodes 50 --device cuda --n-envs 32 \
  --stonefish-executable "$HOME/stonefish-vehicle-a-build/stonefish_vehicle_a_bridge" \
  --stonefish-data-dir "$HOME/stonefish-src/Tests/Data" \
  --stonefish-lib "$HOME/stonefish-install/lib" \
  --stonefish-deps-lib "$HOME/stonefish-deps/lib"
```

Use `--output /path/to/run` to select an explicit output directory. Otherwise
the runner creates a UTC-timestamped directory below
`artifacts/rl-campaign/training-runs/`. The output directory must not already
exist, which prevents accidental overwrite of a prior run.

Each completed run contains:

- `run-manifest.json`: backend, seed, exact algorithm configuration and hash,
  Git revision, command, requested/actual timesteps, timing, device, and the
  Gate 5 action-fairness condition;
- `model-final.zip` and periodic `checkpoints/*.zip`;
- `metrics/progress.csv` and `metrics/progress.json`: learning-curve data;
- `metrics/episodes-env-*.monitor.csv`: per-environment training episode
  return/length data;
- `evaluation-episodes.csv` and `evaluation-episodes.json`: the common
  figure-ready episode schema, when `--eval-episodes` is nonzero;
- `evaluation-summary.json`: success rate and median return.

Useful controls include `--base-seed`, `--eval-first-seed`,
`--checkpoint-freq`, `--n-envs`, and `--device`. Run `npm run
train:portable -- --help` for the complete interface.

Gazebo training retains the repository's scientific safety gate: the Gazebo
conformance artifact must pass before training starts. The default Gazebo
runtime requires Docker and the pinned image referenced by
`BCOD_GAZEBO_IMAGE`. HoloOcean requires its licensed runtime/package to be
installed on the instance. These checks fail before policy optimization rather
than producing incomparable data.

Stonefish requires the four installation paths shown above and verifies the
committed passing Gate D result before training. The bridge deliberately hides
CUDA from each headless Stonefish subprocess; `--device cuda` applies to PPO.
Gate D measured peak simulator throughput with 32 isolated instances on its
32-core host, hence the example's `--n-envs 32`. Select a count appropriate to
the CPU allocation on a different instance.
