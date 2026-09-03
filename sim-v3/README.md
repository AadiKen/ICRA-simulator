# bcod-sim

`bcod-sim` is a deterministic, extensible autonomous-surface-vessel research simulator. Its production path combines typed experiment schemas, vehicle and sensor plugin registries, Node and Python/tensor execution, checkpoint/replay, environmental-data adapters, immutable run artifacts, and a React/Three.js research console.

The simulator is a software research platform, not a full naval simulator or a Gazebo replacement. Read [Validation scope](#validation-scope) before making physical-fidelity claims.

## Quick start

Requirements:

- Node.js 24 (also recorded in `.nvmrc`)
- npm 11 or a compatible npm version

From a fresh clone of the repository:

```bash
cd ICRA-simulator/sim-v3
npm ci
npm run build
npm run serve:web
```

Open `http://localhost:8000/`. The Vite development server uses port 8000 and exits clearly if that port is occupied.

Verify the production build without opening a browser:

```bash
npm run test:ui
```

Do not open HTML files directly from disk. ES module imports, WebGL assets, and browser security rules require an HTTP server.

## Full development setup

The full software gate additionally requires Python 3.12 and [`uv`](https://docs.astral.sh/uv/). From `sim-v3`:

```bash
uv sync --extra hydrodynamics
npm test
```

`npm test` is the authoritative end-to-end software gate. It covers the legacy smoke and physics tests, typed offline runtime, ICRA-focused contracts, Python/tensor equivalence, external-data fixtures, release generation, the UI, long mission artifacts, and migration goldens. Some independent validation commands require software or datasets that are intentionally not part of the default test gate; see [External evidence](#external-evidence).

The original pre-migration goldens remain the comparison basis, and the migration gate currently reports zero unreviewed deltas. Release outputs include paper tables and figures, an anonymous-release manifest, and a scripted offline demonstration.

Useful focused commands:

```bash
npm run build
npm run test:ui
npm run test:smoke
npm run test:physics
npm run test:icra
npm run test:python
npm run test:release
npm run test:migration-gate
```

## Architecture

The supported research architecture is organized around these boundaries:

| Area | Location | Responsibility |
| --- | --- | --- |
| Experiment contract | `packages/experiment-schema` | Typed, immutable experiment configuration and validation |
| Dynamics | `packages/core` | `planar3` and `coupled6` simulation, forces, integration, allocation, checkpoint/replay, and vector execution |
| Vehicles | `packages/vehicle-sdk` | Vehicle registry and the A/B/C configurations |
| Sensors | `packages/sensor-sdk` | Sensor plugins, domains, runtime services, and deterministic checkpoint state |
| Environment | `packages/environment` | Weather, geography, provenance, caching, and real-data-source adapters |
| Metrics/artifacts | `packages/metrics` | Run outputs, checksums, events, summaries, and Parquet artifacts |
| Node backend | `backends/node` | Typed execution, worker lifecycle, recovery, and sensor integration |
| Tensor backend | `backends/tensor` | Python/Gymnasium and CPU tensor execution |
| Research UI | `apps/research-ui` | React/Three.js experiment console and replay UI |
| Benchmarks | `benchmarks/usv-bench-36` | USV-Bench-36 definitions, runner, manifests, and tests |
| Validation | `validation` | Reference comparisons, external datasets, migration gates, and scoped evidence |

The runtime supports two dynamics modes:

- `coupled6` is the interactive default, with NED position, quaternion attitude, six body velocities, 6×6 inertia, Coriolis, damping, hydrostatics, current, wind, and six-axis actuator wrenches.
- `planar3` preserves the surge/sway/yaw plant used by legacy regression and the pinned MSS maneuvering comparison.

In the current `coupled6` scope, rendered waves are visual when `waveCoupling="none"`; they do not apply physical six-axis wave excitation.

Implemented deterministic software contracts include integer-step scheduling, immutable migration goldens, checkpoint/replay, vector execution, worker failure recovery, and Node/CPU tensor equivalence. The Python interface provides Gymnasium single and vector environments with checkpoint and oracle separation. These are software guarantees, not claims of physical fidelity.

## Vehicles and benchmark

The offline platform contains three vehicle configurations:

- Vehicle A: Otter reference configuration.
- Vehicle B: single-rudder production MMG path.
- Vehicle C: dual-azimuth production allocation path, including singular, near-singular, failed-off, and stuck-thruster cases.

USV-Bench-36 defines 36 scenarios across these three vehicles: 108 base configurations plus a fixed deterministic confidence subset. Each retained run carries immutable configuration; state, actuator, and metric Parquet records; events; replay data; checksums; failure metadata; and validation-scope metadata. The repository includes conventional PID, MPC, and CPU PPO baselines; they are baselines, not novelty claims. Run the benchmark tests with:

```bash
npm run test:usv-bench-36
```

The full campaign runner is:

```bash
npm run run:usv-bench-36
```

## Environmental data and credentials

Offline environmental fixtures and cache tests run without network credentials. Live ERA5 historical replay requires a local Copernicus CDS personal access token. Configure `environment.data_sources.era5.credentials` using one of these untracked sources:

- `{ "source": "environment", "env_var": "BCOD_CDS_API_KEY" }`, with `BCOD_CDS_API_KEY` exported in the shell.
- `{ "source": "file", "path": "~/.cdsapirc" }`, with this content:

  ```yaml
  url: https://cds.climate.copernicus.eu/api
  key: <PERSONAL-ACCESS-TOKEN>
  ```

Accept the ERA5 dataset licence on the CDS website before the first request. Credentials are not copied into resolved experiments or provenance. Missing credentials fail with `CDS API key not configured`. NWS live requests separately require a non-secret application/contact identity in `environment.data_sources.nws.user_agent`.

The repository retains an explicit `field-data-unavailable` telemetry artifact so absent real-USV evidence is recorded rather than silently treated as a passing validation result.

## Validation scope

Passing the software suite establishes reproducible software execution; it does not establish physical accuracy for every vehicle and model.

| Vehicle | Current evidence | Required wording |
| --- | --- | --- |
| Vehicle A | Pinned MSS comparison for `planar3` | Validated only within that exact model/reference/test scope |
| Vehicle B | Six MARIN model-scale trajectories with measured rudder and sample-wise RPM | Current parameterization fails the fixed trajectory/IMO limits; USV-scale coefficient validation remains blocked |
| Vehicle C | Allocation, composability, and external-track descriptive campaigns | Dynamics are behaviorally unvalidated; no same-vessel validation claim |
| `coupled6` generally | Software invariants, regression gates, and limited external comparisons | Software-verified, not physically validated as a complete simulator |

The Unity comparison is measured but limited by disabled added mass, WAM-V geometry substitution, a coarse rudder proxy, and a Unity-version deviation. Marginal or negative findings must remain reported as observed.

## External evidence

The following are not implied by a passing `npm test` and remain separate evidence or infrastructure gates:

- reviewed Vehicle B/C hull meshes and Capytaine mesh convergence;
- independent Vehicle B USV maneuver and free-decay data;
- independent Vehicle C dual-azimuth command/trajectory data;
- missing KVLCC2 L7 operating-point inputs or restricted SIMMAN trajectories;
- pinned Linux ROS 2 execution;
- NVIDIA CUDA and Apple MPS runners;
- VRX installation and equivalent-scenario measurements;
- approved live NOAA/USGS execution;
- fresh MSS/Octave regeneration from the pinned source checkout;
- genuine real-USV telemetry.

The official MSS source revision is pinned in `validation/mss-reference.json`. Frozen traces from that exact revision are the independent oracle; the local Python model is not. With GNU Octave and the pinned checkout available:

```bash
MSS_ROOT=/absolute/path/to/MSS npm run generate:mss-golden
npm run test:mss-acceptance
npm run plot:mss
```

## Legacy browser compatibility surfaces

The root-level `threeSim.html` and `demo.html` remain for compatibility and manual inspection. They use `schema.js`, the root `core/` facade, vendored Three.js assets, and procedural rendering. They are not the primary typed research UI.

To serve them separately from the repository’s `sim-v3` directory:

```bash
python3 -m http.server 8001
```

Then open `http://localhost:8001/threeSim.html` or `http://localhost:8001/demo.html`.

Optional legacy sensor streaming is enabled with `?streamSensors=1` or `localStorage.setItem("bcodStreamSensors", "1")`. It publishes camera, LiDAR, and telemetry to `ws://127.0.0.1:8765`, `:8766`, and `:8767`, respectively. Example receivers are in the historically named `data recievers/` directory; quote that path because it contains a space.

## Repository policy

- Keep credentials in environment variables or untracked local files.
- Treat manifests, checksums, seeds, versions, and validation scope as part of every reported result.
- Do not describe software regression evidence as physical validation.
- Use the typed package/plugin boundaries for new vehicles, sensors, experiments, and backends.
- Keep generated paper claims traceable to retained artifacts.

`README.md` is the authoritative description of setup, architecture, and validation scope. Historical planning documents may contain superseded implementation details.
