# Portable training runner

`train_portable_ppo.py` is the single training entry point for bcod-sim,
Gazebo Harmonic, and HoloOcean. It uses the same frozen RecurrentPPO
configuration for all three. The requested budget is always supplied with
`--timesteps`.

From `sim-v3`, run:

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
- `metrics/episodes.monitor.csv`: training episode return/length data;
- `evaluation-episodes.csv` and `evaluation-episodes.json`: the common
  figure-ready episode schema, when `--eval-episodes` is nonzero;
- `evaluation-summary.json`: success rate and median return.

Useful controls include `--base-seed`, `--eval-first-seed`,
`--checkpoint-freq`, and `--device`. Run `npm run
train:portable -- --help` for the complete interface.

Gazebo training retains the repository's scientific safety gate: the Gazebo
conformance artifact must pass before training starts. The default Gazebo
runtime requires Docker and the pinned image referenced by
`BCOD_GAZEBO_IMAGE`. HoloOcean requires its licensed runtime/package to be
installed on the instance. These checks fail before policy optimization rather
than producing incomparable data.
