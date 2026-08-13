# Unity validation harness

This harness compares the production bcod-sim Vehicle B coupled6/MMG path
with `unity_asv_sim` in a planar `t,N,E,yaw,u,v,r` projection.

## Prerequisites

- Unity Editor **2023.2.20f1**.
- The coefficient values supplied for the Vehicle B or Vehicle C run. Do not
  guess or fill any `null` in `coefficient_mapping.json`.
- An absolute, writable output path for the Unity trajectory CSV.

## Open the Unity project

1. In Unity Hub, choose **Add project from disk**.
2. Select this README's `unity_asv_sim/` directory.
3. Open it with Unity 2023.2.20f1 and allow Package Manager to resolve the
   dependencies.
4. Open `Assets/Scenes/Demo Scene/Scene.unity`, the enabled build scene.

## Configure the vessel

1. In the Hierarchy, select the vessel root GameObject that owns the
   `Rigidbody` and `ShipController` components.
2. Add `ReplayThruster` and `TrajectoryLogger` to that same GameObject using
   **Add Component**.
3. Keep `Assets/Scripts/Controller/ShipController.cs` unchanged. For a replay
   run, disable the **ShipController component checkbox** and enable
   **ReplayThruster**. Re-enable ShipController for an interactive run. Never
   enable both during the same run because both components apply forces.
4. Confirm ReplayThruster has found the engine and propeller transforms. If
   either field remains empty, assign the vessel's engine joint and its
   `PropellerJoint` manually in the Inspector.

## Apply coefficients

1. Open `coefficient_mapping.json` beside the Unity project.
2. For every entry whose `coupled6_value` has been supplied and reviewed,
   copy that value into the Inspector field named by `unity_field`.
3. Enter added-mass and damping values on the vessel's `FossenDynamics`
   component (`XdotU`, `YdotV`, `NdotR`, `Xu`, `Yv`, `Nr`, and the applicable
   quadratic fields).
4. Enter mass, inertia tensor, and center of mass values on `Rigidbody` using
   the mapped Unity fields, not the SNAME letter names directly. The exact
   body-axis conversion is:
   - SNAME `x` forward -> Unity local `+z`, so `Ix -> inertiaTensor.z`.
   - SNAME `y` starboard -> Unity local `+x`, so `Iy -> inertiaTensor.x`.
   - SNAME `z` down -> Unity local `-y`, so `Iz -> inertiaTensor.y` (axis sign
     does not change a scalar diagonal moment of inertia).
   - A nonzero SNAME CG `[x,y,z]` maps to Unity local `[y,-z,x]`.
5. Enter the sourced `propellerDiameter`, `waterDensity`, `k0`, `k1`, `k2`,
   `wP0`, `thrustDeduction`, and `lagTimeConstant` values on ReplayThruster.

## Prepare a command replay

Convert an existing Vehicle B USV-Bench actuator artifact to physical command
CSV:

```bash
node adaptVehicleBActuatorLog.js \
  --input /absolute/path/to/actuator.jsonl \
  --out /absolute/path/to/vehicle-b-commands.csv
```

The adapter writes one command per simulation step using:

```text
time_s,propeller_rps,rudder_rad
```

Set **Edit > Project Settings > Time > Fixed Timestep** to the timestep
reported by the adapter.

ReplayThruster now consumes this physical schema directly and computes the
MMG open-water thrust curve from live local surge velocity and replayed RPS.
Set **Command File Path** to the absolute path of the CSV.

The USV-Bench adapter extracts `applied_command`, which is already lagged and
rate-limited by coupled6. When replaying an adapted applied-command log, set
`lagTimeConstant` to `0` to avoid applying lag twice. For a raw commanded-RPS
sequence such as `synthetic_test_commands.csv`, retain the sourced `0.35 s`
lag value.

Replay input schema:

```text
time_s,propeller_rps,rudder_rad
```

## Log and compare the Unity trajectory

1. On `TrajectoryLogger`, set **Output Path** to an absolute writable path,
   for example `/tmp/unity_trajectory.csv`.
2. Leave **Log Interval S** at `0` to log every FixedUpdate, or set it to the
   same interval used by bcod-sim.
3. Confirm ShipController is disabled and ReplayThruster and
   TrajectoryLogger are enabled.
4. Enter Play mode and let the command sequence finish, then exit Play mode.
   `TrajectoryLogger` flushes and closes its CSV when the component is
   destroyed at Play-mode exit.
5. Find the Unity CSV at the exact absolute **Output Path** entered above.
6. Compare it with the coupled6 trajectory:

```bash
python3 compare_trajectories.py \
  /absolute/path/to/coupled6_trajectory.csv \
  /absolute/path/to/unity_trajectory.csv \
  --out /absolute/path/to/comparison-report.md \
  --plot /absolute/path/to/divergence.png
```

Initial N/E/yaw pose differences are normalized by default. Add
`--no-normalize` only when debugging absolute scene placement.
