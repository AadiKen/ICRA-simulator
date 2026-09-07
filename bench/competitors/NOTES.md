# Competitor container bring-up log

This log is chronological. Failed attempts are retained because setup friction is part of the later ICRA 2027 comparison evidence.

## 2026-08-13 — Initial audit

- Repository base commit: `adb48e979def0d4a9eac26db9d114e803e47024b` (`main`).
- Host: macOS on Apple Silicon (`arm64`, Darwin 25.5.0).
- Docker CLI: 29.6.1, context `desktop-linux`.
- Pre-existing working-tree changes are confined to Phase 0 files under `sim-v3/` and the Unity-validation submodule; this task will not modify them.

### Failure 1 — sandbox cannot access Docker Desktop socket

Command:

```text
docker version
```

Exact error:

```text
permission denied while trying to connect to the docker API at unix:///Users/aadikenchammanaold/.docker/run/docker.sock
```

Status: environment access restriction. Docker build/run commands require approved execution outside the filesystem sandbox. This is not a Gazebo or VRX failure.

### Fairness checkpoint identified

The host is ARM64. Before choosing image platforms, the official images and VRX dependency support must be checked for both `linux/arm64` and `linux/amd64`. Forcing x86 emulation for only one competitor could bias setup friction, so no platform override will be selected silently.

## Gazebo Harmonic

### Official-source findings

- Gazebo's Harmonic documentation lists `gz-sim` 8.x and Ubuntu Jammy/Noble on AMD64 as officially supported; ARM is best-effort.
- The official headless launch documented by Gazebo is `gz sim -s shapes.sdf -v 4`.
- The official Ubuntu binary instructions install the `gz-harmonic` metapackage from `packages.osrfoundation.org`.

### Failure 2 — guessed Docker Hub repository does not exist or is inaccessible

Command (first command in a chained read-only manifest probe):

```text
docker buildx imagetools inspect gazebosim/gz-sim:harmonic
```

Exact error:

```text
ERROR: pull access denied, repository does not exist or may require authorization: server message: insufficient_scope: authorization failed
```

Status: unresolved tag/repository lookup. No Dockerfile uses this guessed repository. The user explicitly required checking rather than guessing, so the next step is to locate the exact image name from official Gazebo-owned sources.

### Pin resolution

- Official Ubuntu 24.04 AMD64 manifest: `ubuntu@sha256:019e8eb29a85e74d64925745884f2ec79aa27e3feab36353d24656f4d6b89467` (resolved 2026-08-16 from the Docker Official Image index).
- Gazebo repository signing-key SHA-256: `15d0300460e0c9c0efb21fa9770c0bf228988a07d71c4fac985c2161954a25a4`.
- Exact Noble metapackage: `gz-harmonic=1.0.0-1~noble`.
- The metapackage currently resolves to Gazebo Sim 8.15.0 (`gz-sim8-cli=8.15.0-1~noble` and `libgz-sim8=8.15.0-1~noble`).

### Failure 3 — Docker Desktop daemon was not running

The first ephemeral APT-version probe failed before container creation.

Exact error:

```text
Cannot connect to the Docker daemon at unix:///Users/aadikenchammanaold/.docker/run/docker.sock. Is the docker daemon running?
```

Fix: start Docker Desktop, wait for the daemon, then repeat the identical probe. This is host-environment friction, not a Gazebo package failure.

### First image build

Command:

```text
docker build --progress=plain --platform linux/amd64 -t icra27-gazebo-harmonic:harmonic-8.15.0 -f bench/competitors/Dockerfile.gazebo .
```

Result: succeeded without manual intervention. Complete output is retained in `evidence/gazebo-build.log`.

BuildKit emitted `FromPlatformFlagConstDisallowed` because the Dockerfile fixes `linux/amd64`. This is deliberate: the required definition-of-done build command does not pass `--platform`, and silently selecting the ARM64 host platform would violate the approved common-platform methodology. BuildKit also classified the public signing-key checksum ARG as a possible secret; it is a public integrity digest, not a credential.

### First headless scenario launch

Command:

```text
docker run --rm --platform linux/amd64 icra27-gazebo-harmonic:harmonic-8.15.0 timeout --signal=INT 15s gz sim --force-version 8 -s -r -v 4 shapes.sdf
```

Result: the server reported `Gazebo Sim Server v8.15.0`, loaded `/usr/share/gz/gz-sim8/worlds/shapes.sdf`, loaded DART physics, and reported `World [shapes] initialized with [default_physics] physics profile.` It then started simulation runner threads and shut down cleanly on the intentional 15-second timeout. Exit status `124` is the expected result of the bounded verification command, not a simulator crash. Complete output is retained in `evidence/gazebo-run.log`.

The sample emitted warnings that DART does not directly support ellipsoid and cone collision shapes and that Gazebo generated mesh substitutes. These are nonfatal warnings from the official sample world; the world still initialized and ran. A separate clock/iteration query is being captured to prove that the initialized server advances simulation time.

The clock/iteration query subsequently returned `sim_time { sec: 7 nsec: 438000000 }`, `iterations: 7438`, and `step_size { nsec: 1000000 }`. This independently confirms that the simulation advanced after initialization. Output is retained in `evidence/gazebo-clock.log`.

The complete installed package inventory contains 817 package/version records and is retained in `evidence/gazebo-packages.log`.

## VRX

### Official-source findings

- The current official `jazzy` branch and wiki describe Gazebo Harmonic + ROS 2 Jazzy as the recommended configuration for new users.
- The official source setup clones `osrf/vrx`, sources `/opt/ros/jazzy/setup.bash`, and builds with `colcon build --merge-install`.
- The official scenario command is `ros2 launch vrx_gz competition.launch.py world:=sydney_regatta`.
- The July 2025 Docker wiki still shows `ghcr.io/osrf/vrx-devel:latest`, which conflicts with this task's no-`latest` reproducibility rule. A digest or immutable version must replace it.

### Fairness-relevant ambiguity — pending check-in

Gazebo Harmonic officially supports Ubuntu AMD64; ARM is best-effort. The host is ARM64. Choosing native ARM for one tool and emulated AMD64 for the other could bias setup friction. No platform override has been selected.

The official VRX wiki's presumed explicit `jazzy` tag does not exist:

```text
$ docker buildx imagetools inspect ghcr.io/osrf/vrx-devel:jazzy
ERROR: ghcr.io/osrf/vrx-devel:jazzy: not found
```

The documented `latest` image currently resolves to immutable digest `sha256:6d018ab0c1cf0d6fe1f3bf266c9f1efac09cf9f6f40bc13ca437e5bbdaf7d1c4`. Registry metadata shows it is a single-platform `linux/amd64` Ubuntu 24.04 image with ROS 2 Jazzy and Gazebo Harmonic. It was created 2025-11-26. A Dockerfile may pin this digest without using the mutable `latest` tag, but doing so requires AMD64 emulation on this ARM64 host.

### Required check-in before builds

Two official-source conflicts affect methodology:

1. There is no current Gazebo Harmonic Docker Official Image / verified `gazebosim` base. Docker Hub's Gazebo Official Image has no supported current tags and only retains Gazebo Classic 11 images. Gazebo Harmonic's own official instructions use Ubuntu binary packages instead.
2. VRX's official Docker instructions use a mutable `latest` tag and publish only AMD64, while this task forbids `latest` and the test host is ARM64.

Per the task guardrail, iteration was paused before selecting a substitute base or platform.

## 2026-08-16 — Methodology approved

The user approved the following fairness-preserving substitution:

- Build and run both competitors as `linux/amd64`.
- Gazebo: use the official Ubuntu 24.04 image pinned by digest, then install the exact published `gz-harmonic` package version from Gazebo's official APT repository.
- VRX: use the official VRX development image pinned by digest and clone an exact VRX source commit.
- Record plainly that a current official Gazebo Harmonic base image does not exist.
- Capture the full installed APT package/version inventory for Gazebo so its dependency state is auditable alongside VRX's source pin.

### Standalone T7.2 timing caveat — AMD64 emulation

Both containers are intentionally run as `linux/amd64` under emulation on an Apple Silicon (`arm64`) host. This applies the same architecture handicap to both competitors for bring-up fairness, so it does not block this task. It **does** invalidate unqualified native-install or native-runtime timing claims. Any later T7.2 install-friction timing collected from these builds must explicitly say it was measured under AMD64 emulation on ARM64, or be repeated on a native AMD64 host.

### Failure 4 — first digest-pinned VRX base pull did not finish

The initial `docker run` began pulling the large VRX base, but the client/tool session ended before Docker registered a complete local image. A subsequent `docker image inspect` returned:

```text
Error response from daemon: No such image: ghcr.io/osrf/vrx-devel@sha256:6d018ab0c1cf0d6fe1f3bf266c9f1efac09cf9f6f40bc13ca437e5bbdaf7d1c4
```

Fix: resume explicitly with `docker pull --platform linux/amd64` and wait until every layer is complete. Docker then reported the expected digest and `Downloaded newer image`.

### Failure 5 — Docker Desktop content-store I/O error after successful pull

The first attempt to run the fully pulled digest failed before the container process started.

Exact error:

```text
docker: Error response from daemon: rpc error: code = Unknown desc = blob sha256:6d018ab0c1cf0d6fe1f3bf266c9f1efac09cf9f6f40bc13ca437e5bbdaf7d1c4 expected at /var/lib/desktop-containerd/daemon/io.containerd.content.v1.content/blobs/sha256/6d018ab0c1cf0d6fe1f3bf266c9f1efac09cf9f6f40bc13ca437e5bbdaf7d1c4: open /var/lib/desktop-containerd/daemon/io.containerd.content.v1.content/blobs/sha256/6d018ab0c1cf0d6fe1f3bf266c9f1efac09cf9f6f40bc13ca437e5bbdaf7d1c4: input/output error
```

Status: under host-level diagnosis. This names Docker Desktop's internal content store, not a VRX program or dependency error. Full output is retained in `evidence/vrx-base-inspection.log`.

The error was not limited to VRX: `docker system df` and a repeat run of the already proven Gazebo image both failed on another internal blob with the same `input/output error`. This confirms a Docker Desktop content-store failure rather than competitor behavior.

Fix: `docker desktop restart`. After restart, Docker could inspect the VRX digest and the Gazebo control again returned version `8.15.0`. Retrying the VRX base inspection then succeeded.

### Failure 6 — partial clone checkout needed network access

An exploratory host-side `git clone --filter=blob:none --no-checkout` succeeded, but the first sandboxed checkout of the pinned commit needed to retrieve promised blobs and failed with:

```text
fatal: unable to access 'https://github.com/osrf/vrx.git/': Could not resolve host: github.com
fatal: could not fetch 356e0dd2cdcb6cdc8f0cc374e3f4591b3bd017d9 from promisor remote
```

Fix: repeat the exact checkout with approved network access. Commit `fda35961463c2ed8c44d6c646fe8ce773a3eaff1` then checked out successfully. This was a local sandbox-network restriction, not an upstream VRX failure.

### Official base inspection findings

- `ROS_DISTRO=jazzy`.
- `ros_gz_sim` is installed at `/opt/ros/jazzy`.
- Gazebo Sim reports version `8.10.0`.
- `/etc/profile.d/ros_gz_pythons.sh` only exports `GZ_CONFIG_PATH`; it does not source ROS.
- The configured `/entrypoint-ros.sh` exists but is a zero-byte executable. The derived image must explicitly source `/opt/ros/jazzy/setup.bash` and its own workspace setup. This is an ambiguity/defect in the official development image, and the workaround is explicit in `Dockerfile.vrx`.
- No VRX source tree is included in the base, so the exact stable release source must be cloned and built.

The pinned v3.0.1 source confirms that `competition.launch.py` supports `headless:=True`; the official scenario can therefore be launched without GUI changes or an improvised launch file.

### First derived-image build

Command:

```text
docker build --progress=plain --platform linux/amd64 -t icra27-vrx:v3.0.1 -f bench/competitors/Dockerfile.vrx .
```

Result: succeeded without manual intervention. Five VRX packages built successfully: `vrx_gazebo`, `vrx_ros`, `wamv_description`, `wamv_gazebo`, and `vrx_gz`. Complete output is retained in `evidence/vrx-build.log`.

BuildKit emitted the same deliberate `FromPlatformFlagConstDisallowed` warning as the Gazebo build. The fixed platform prevents the required no-flag definition-of-done command from silently selecting ARM64.

### Failure 7 — inherited zero-byte entrypoint prevents container start

First scenario command:

```text
docker run --rm --platform linux/amd64 icra27-vrx:v3.0.1 timeout --signal=INT 45s bash -lc 'source /opt/ros/jazzy/setup.bash && source /opt/vrx_ws/install/setup.bash && exec ros2 launch vrx_gz competition.launch.py world:=sydney_regatta headless:=True'
```

Exact error, before the requested command started:

```text
exec /entrypoint-ros.sh: exec format error
```

Cause: the official base config declares `/entrypoint-ros.sh`, but that file is zero bytes and has no executable format. Fix: explicitly reset the inherited entrypoint with `ENTRYPOINT []`; the image's CMD continues to source ROS Jazzy and the built VRX workspace itself.

### Failure 8 — cold Fuel downloads exceeded short proof windows

The first 60-second and later 180-second full ROS scenario probes repeatedly printed `Requesting list of world names` and were intentionally interrupted before the world appeared. A verbose direct server probe exposed the cause: the checked-in `sydney_regatta.sdf` references multiple public models by unversioned Gazebo Fuel URL, and a clean container must download and extract them before advertising the world. This was not Docker authentication; the public downloads proceeded without login.

The direct diagnostic command was:

```text
docker run --rm --platform linux/amd64 icra27-vrx:v3.0.1 timeout --signal=INT 90s bash -lc 'source /opt/ros/jazzy/setup.bash && source /opt/vrx_ws/install/setup.bash && exec gz sim -s -r -v 4 sydney_regatta.sdf'
```

Although initialization completed after the timeout signal while extraction was draining, its log proves the server loaded the installed VRX world, downloaded Fuel models, initialized VRX wave/buoyancy systems, reported `World [sydney_regatta] initialized with [4ms] physics profile.`, and started simulation worker threads. Full output is in `evidence/vrx-gz-direct.log`.

Fix for the bounded full-stack proof: use a named Docker volume at `/root/.gz/fuel` for the public first-run asset cache, populate it with the same installed Sydney Regatta world, and then run the unchanged official ROS launch command against that cache. This does not alter the image, source, world, or runtime dependencies. It exposes an important install-friction fact: a nominally built VRX image still has a large, network-dependent first scenario start.

### Successful full VRX scenario launch

Command:

```text
docker run --rm --platform linux/amd64 \
  -v icra27-vrx-fuel-proof:/root/.gz/fuel \
  icra27-vrx:v3.0.1 \
  timeout --kill-after=20s --signal=INT 90s \
  bash -lc 'source /opt/ros/jazzy/setup.bash && source /opt/vrx_ws/install/setup.bash && exec ros2 launch vrx_gz competition.launch.py world:=sydney_regatta headless:=True'
```

Result: the launch started Gazebo Sim 8.10.0, loaded the installed `sydney_regatta.sdf`, initialized VRX wind, wave, buoyancy, hydrodynamics, acoustic-pinger, and ball-shooter systems, created entity `wamv`, reported `Robot initialized`, and created ROS/Gazebo bridges for the vehicle pose, joint state, cameras, lidar, NavSat, IMU, thrusters, acoustics, and ball shooter. Full output is retained in `evidence/vrx-run-cached.log`.

A live world-statistics query returned simulation time `1.704000000` seconds, iteration `426`, and step size `0.004` seconds. This proves that the full Sydney Regatta + WAM-V scenario advanced simulation rather than merely starting processes. Output is retained in `evidence/vrx-clock.log`.

Nonfatal upstream runtime warnings are retained in the log: the detachable-joint plugin repeatedly could not find `dummy_upper`, KDL warned about inertia on the root link, Ogre ignored reserved visibility-mask bits, and Ogre initially reported no display before using headless mode. None prevented world initialization, WAM-V creation, bridge creation, sensor advertisement, or clock advancement.

## Final pinned versions

- Common platform: `linux/amd64` under emulation on an ARM64 Apple Silicon host.
- Gazebo base: Docker Official Ubuntu 24.04 AMD64 manifest `ubuntu@sha256:019e8eb29a85e74d64925745884f2ec79aa27e3feab36353d24656f4d6b89467`.
- Gazebo metapackage: `gz-harmonic=1.0.0-1~noble`; resolved Gazebo Sim package `gz-sim8-cli=8.15.0-1~noble`.
- Gazebo repository key SHA-256: `15d0300460e0c9c0efb21fa9770c0bf228988a07d71c4fac985c2161954a25a4`.
- Gazebo installed dependency inventory: all 817 package/version records in `evidence/gazebo-packages.log`.
- VRX base: `ghcr.io/osrf/vrx-devel@sha256:6d018ab0c1cf0d6fe1f3bf266c9f1efac09cf9f6f40bc13ca437e5bbdaf7d1c4` (`linux/amd64`, Ubuntu 24.04, ROS 2 Jazzy, Gazebo Sim 8.10.0).
- ROS distribution: Jazzy, from `/opt/ros/jazzy` in the immutable official VRX base.
- VRX source: release v3.0.1 commit `fda35961463c2ed8c44d6c646fe8ce773a3eaff1`.

The VRX world file's public Fuel URLs are not version-qualified in upstream source. The downloaded cache resolved concrete model revisions during this run, but the Dockerfile deliberately does not vendor or silently rewrite those upstream URLs. This is a reproducibility ambiguity in the official scenario and should be considered before later determinism work.

## Final definition-of-done verification

Both requested commands were run verbatim against the delivered files and succeeded:

```text
docker build -f bench/competitors/Dockerfile.gazebo .
docker build -f bench/competitors/Dockerfile.vrx .
```

Their complete terminal captures are `evidence/gazebo-build-final.log` and `evidence/vrx-build-final.log`. Both retain the deliberate fixed-platform BuildKit warning described above. Gazebo also retains BuildKit's false-positive secret warning for the public repository signing-key digest; no credential or secret is present.

## 2026-08-21 reproducibility and clean-install follow-up

### Failure 9 — a genuinely empty VRX Fuel cache cannot fetch the official scenario on this host

For the requested cold install-to-first-step measurement, the Docker build cache and only the competitor images were removed, then a new empty, competitor-specific volume (`icra27-vrx-fuel-timing-20260821`) was mounted at `/root/.gz/fuel`. The full official headless launch then failed to obtain its public scenario assets. Exact error:

```text
libcurl: (6) Could not resolve host: fuel.gazebosim.org
[Err] [FuelClient.cc:695] Failed to download model.
  Server: https://fuel.gazebosim.org
  Route: openrobotics/models/sydney_regatta/tip/sydney_regatta.zip
```

The command ran for 4159.45 seconds before its bounded launch completed without a usable fetched scenario; its full output is `evidence/vrx-first-step-20260821.log`. This is an environment/network-DNS gap, not a successful clean timing. No cached run is substituted for that measurement.

### Determinism negative-control result — unvalidated for the selected stock scenarios

Gazebo Harmonic was run with `shapes.sdf`, 50 iterations, `--seed 7319`, and recorded dynamic poses. VRX was run with the official `sydney_regatta` world and WAM-V, 50 iterations, `--seed 7319`, using the pre-existing proof-only Fuel cache; the exact launcher arguments and raw recordings are under `evidence/determinism/`.

For VRX, the two same-seed recordings had identical WAM-V positions at all 15 common recorded timestamps (maximum Euclidean difference `0`), but changing the seed to `7320` also had identical WAM-V positions at all 17 common timestamps (maximum difference `0`). Therefore a mere seed change does not exercise stochastic trajectory behavior in this short stock scenario. The requested negative-control discipline was **not satisfied**, and the same-seed observation must not be presented as a validated no-divergence result. Raw `.tlog` hashes also differ between repeat runs because the logging container is not canonical; comparisons used replayed dynamic-pose messages instead.

The complete measurement/status record is `comparison-results.json`.

### Follow-up root cause — VRX seed wiring works; the original physical window was seed-insensitive

The underlying Gazebo server—not merely the top-level CLI—logged `Setting seed value: 7319` for both 7319 runs and `Setting seed value: 7320` for the altered-seed run. Thus the seed argument reaches Gazebo correctly.

The same startup logs also identify the VRX `USVWind` plugin's independent `Random seed value = 10`. The pinned `sydney_regatta.sdf` hard-codes `<random_seed>10</random_seed>`, while the active configuration has zero variable-wind gain and zero wave amplitude. A passive 50-iteration WAM-V dynamic-pose trace therefore has no active seed-consuming physical perturbation. This is a genuinely seed-insensitive short stock window, not broken launch wiring.

To make the negative control substantive, three 100-iteration official Sydney Regatta + WAM-V runs recorded the native configured-Gaussian-noise IMU stream. The two `--seed 7319` streams were byte-identical (41 samples, SHA-256 `c6ea21a60fbde93de572f1bd3ab4357702cb438d8087b3837aa4ad8e048907ee`). `--seed 7320` diverged at sample 1 / 4 ms, with maximum IMU-vector L2 difference `0.19169947183025843` and SHA-256 `5243c7b0542bd1a32ff388ec2dddb2e7f2a2d9facca27152bf69858e5ba389aa`. The negative control therefore passes for the native stochastic sensor path. Evidence: `evidence/determinism/vrx-imu/`.

### Cold Fuel failure timing clarification

The 2026-08-21 empty-cache cold VRX attempt has an exact measured command duration of `4159.45` seconds before its unsuccessful outcome, with the documented Fuel DNS error. It was not instrumented to timestamp the *first* individual error line, so that value is reported only as unsuccessful install-friction elapsed time—not as a false time-to-first-error.

On 2026-08-22, a fresh bounded empty-cache probe did **not** reproduce DNS failure: Fuel download began. It was stopped before first simulation step and is not used as a clean timing. This demonstrates that the prior DNS issue was transient host/network state; it does not erase the retained failed-attempt observation.
