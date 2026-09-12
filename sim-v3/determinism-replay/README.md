# Deterministic replay evidence status

The versioned fixture is `../fixtures/determinism-replay-route-v1.json`. Run the
implemented native arm with `npm run sweep:determinism-replay -- --simulator bcod-sim
--n 30 --out artifacts/determinism-replay`. Each replay creates a fresh simulation,
uses the same open-loop commanded thrust schedule, and writes native-step samples.
The BCOD engine's bounded world requires an internal (10000,10000) NED origin; the
exported trajectory is translated back to the fixture's (0,0) route-local frame,
and the translation is recorded in every log.

The `all` sweep intentionally fails before writing when a requested native adapter
is missing. Gazebo, VRX, HoloOcean, and Stonefish do **not** yet have Vehicle A Otter
90-second open-loop adapters in this directory. In particular, the existing VRX
path renders Surveyor, not Otter; relabeling it as Vehicle A would invalidate the
five-arm comparison. Each adapter must consume the shared fixture, initialize the
same vehicle and calm environment, record its actual plant configuration, and return
the common `TrajectoryLog` schema. Partial crash logs are retained and make the
sweep exit unsuccessfully.

The figure script is `../figures/production/determinism_replay.py`. It requires all
five complete sets of 30 logs by default; it refuses missing, crashed, or
provenance-mismatched data. No empty or synthetic panels are rendered. It computes
signed normal-offset envelopes and RMSE from the raw logs, with a correction for
native-step nearest-neighbor quantization so bit-identical replays measure zero.
The publication PDF and PNG land in `../figures/out/publication/` only after all
five native arms are present.
