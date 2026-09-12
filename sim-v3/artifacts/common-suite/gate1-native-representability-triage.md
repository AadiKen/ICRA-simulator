# Gate 1 native representability triage

Status: **STOPPED — triage complete; no fix code started**

Update: the stale-contract wiring was subsequently fixed across all four arms.
The post-change result is recorded in
`gate1-representability-after-contract-rewire.json`. Route/range binding and the
150-second limit now pass everywhere; the missing disturbance-injection and
energy-accounting items remain open.

Candidate nominal contract: `eval/common_suite/contracts/nominal.json`  
Candidate SHA-256: `e998e32f5ee00f6296fce04e01066fa3e9a40d3737dec66b6aae79ef212f119b`

The native task wrappers are currently bound to
`artifacts/rl-campaign/surveyor/task-contract-frozen.json` (SHA-256
`2eff3e87da1c789f048711faf42972d7b66b130a939878a0a80d1b778924bb36`), not the
candidate nominal contract. That older contract specifies a different route and
disturbance ranges, 2,400 physics steps / 120 seconds, and no energy cap.

## Classification by runtime

| Runtime | Contract/route and disturbance-range ingestion | Current | Wind | 150 s time cap | 50,000 Ns energy cap |
|---|---|---|---|---|---|
| BCOD | **Config bug.** `CommonWaypointEnv` reads only the older frozen task contract; its route and ranges therefore do not match the candidate. | **Config bug.** The Node plant already accepts and applies arbitrary NED current vectors, but values come from the older contract. | **Config bug.** The Node plant already accepts and applies arbitrary NED wind vectors, but values come from the older contract. | **Config bug.** The wrapper derives 2,400 steps from the older contract and also hard-codes `duration_s: 120`. Requested minus applied: **+30 s**. | **Missing implementation.** The wrapper neither accumulates `dt * (|port thrust| + |starboard thrust|)` nor includes the cap in success/termination. |
| Gazebo | **Config bug.** The Gym wrapper supplies the older task definition; the episode generator is also invoked with its default 2,400-step shape. | **Missing implementation.** `gazebo_gym_runtime.py` records the requested vector but loads no current-force system and reports applied current `[0,0,0]`. Requested magnitude range **0–0.45 m/s**; applied **0 m/s**. | **Missing implementation.** The runtime loads no wind-force system and reports applied wind `[0,0,0]`. Requested magnitude range **0–5 m/s**; applied **0 m/s**. | **Config bug.** Timeout is inherited from the older 2,400-step contract. Requested minus applied: **+30 s**. | **Missing implementation.** No propulsion impulse accumulator or cap-aware success rule exists in the native Gym/task path. |
| HoloOcean | **Config bug.** `HoloOceanVehicleAEnv` requires the older contract hash, so it cannot ingest the candidate route/ranges. | **Config bug.** Native `set_ocean_currents` support exists and seeded current reaches the plant, but its values come from the older contract. | **Missing implementation.** The prebuilt runtime exposes no external-force command. The wrapper represents only longitudinal wind by adding equal force through both thrusters; lateral aerodynamic load is explicitly unsupported. This cannot enforce arbitrary candidate wind direction literally. | **Config bug.** `timeout_steps` is read from the older 2,400-step contract. Requested minus applied: **+30 s**. | **Missing implementation.** No propulsion impulse accumulator or cap-aware success rule exists. Wind force is also mixed into the thruster command, so propulsion accounting must distinguish policy propulsion from environmental load. |
| Stonefish | **Config bug.** `StonefishCommonTask` requires the older contract hash and derives route/ranges from it. | **Missing implementation.** The task samples current, then reports zero applied current; neither the bridge protocol nor the checked-in native manager exposes a current injection path. Requested magnitude range **0–0.45 m/s**; applied **0 m/s**. | **Missing implementation.** The task samples wind, then reports zero applied wind; neither the bridge protocol nor the checked-in native manager exposes a wind-force injection path. Requested magnitude range **0–5 m/s**; applied **0 m/s**. | **Config bug.** `timeout_steps` is read from the older 2,400-step contract. Requested minus applied: **+30 s**. | **Missing implementation.** No propulsion impulse accumulator or cap-aware success rule exists. |

## Evidence locations

- BCOD contract binding, disturbance vectors, hard-coded duration, and timeout:
  `packages/python-client/bcod_sim/common_task_env.py:20-40, 95-110`
- HoloOcean contract binding and timeout:
  `packages/python-client/bcod_sim/holoocean_vehicle_a_env.py:102-125`
- HoloOcean native current application:
  `packages/python-client/bcod_sim/holoocean_vehicle_a_env.py:509-515`
- HoloOcean wind limitation and thruster-based approximation:
  `packages/python-client/bcod_sim/holoocean_vehicle_a_env.py:372-397, 564-576`
- Stonefish sampled-but-zero disturbances:
  `stonefish/python/stonefish_task.py:120-140`
- Stonefish timeout and success logic:
  `stonefish/python/stonefish_task.py:83, 175-181`
- Gazebo requested-versus-applied diagnostic:
  `validation/rl-campaign/ports/gazebo_gym_runtime.py:123, 320`
- Gazebo episode generator default:
  `validation/rl-campaign/ports/prepare-gazebo-episode.ts:7, 17`

## Gate consequence

No calibration episodes may be launched yet. Per the handoff, each runtime must
be repaired and immediately rechecked for literal nominal-contract
representability before moving to the next runtime. Saturation remains not
computable, the five condition contracts remain unfrozen, and Gates 2–4 remain
unattempted.
