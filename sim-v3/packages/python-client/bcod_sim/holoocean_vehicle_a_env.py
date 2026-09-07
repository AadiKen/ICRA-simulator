"""Oracle-safe HoloOcean adapter for the frozen Vehicle A observation contract.

This is a bring-up wrapper, not a declaration of cross-simulator conformance.
Only ``GPSSensor``, ``IMUSensor``, and ``MagnetometerSensor`` payloads enter
the sensor-derived portion of the policy observation.  In particular, no pose,
location, dynamics, velocity, orientation, or rotation sensor is configured.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Callable

import gymnasium as gym
import numpy as np

try:
    from .holoocean_termination import HoloOceanTerminationMonitor
    from .common_task import (
        CompletionTracker,
        classify_termination,
        compute_reward,
        cross_track_distance,
        passed_waypoint_plane,
    )
    from .common_task_env import Mulberry32
except ImportError:  # Direct validation-script import from this directory.
    from holoocean_termination import HoloOceanTerminationMonitor
    from common_task import (
        CompletionTracker,
        classify_termination,
        compute_reward,
        cross_track_distance,
        passed_waypoint_plane,
    )
    from common_task_env import Mulberry32


class HoloOceanVehicleAEnv(gym.Env[np.ndarray, np.ndarray]):
    """Minimal Gymnasium wrapper for HoloOcean's twin-thruster SurfaceVessel."""

    metadata = {"render_modes": []}
    backend_type = "holoocean"
    vehicle_preset = "vehicle-a-otter"
    EXPECTED_CONTRACT_SHA256 = (
        "2eff3e87da1c789f048711faf42972d7b66b130a939878a0a80d1b778924bb36"
    )
    FIELD_NAMES = (
        "linear_accel_x",
        "linear_accel_y",
        "linear_accel_z",
        "angular_rate_x",
        "angular_rate_y",
        "angular_rate_z",
        "orientation_yaw_rad",
        "relative_goal_north_m",
        "relative_goal_east_m",
        "fix_valid",
        "previous_effector_0",
        "previous_effector_1",
        "previous_steer_0",
        "previous_steer_1",
        "normalized_time_remaining",
    )
    FORBIDDEN_POLICY_SENSORS = frozenset(
        {
            "PoseSensor",
            "LocationSensor",
            "DynamicsSensor",
            "VelocitySensor",
            "OrientationSensor",
            "RotationSensor",
        }
    )
    GPS_MAX_AGE_S = 0.10
    # Literal +/-95 N force parity was tested first, but the packaged SurfaceVessel's
    # hardcoded 200 kg mass and hybrid Unreal/quadratic damping cannot be configured
    # to match bcod-sim's 55 kg, 24u + 2|u|u Vehicle A plant.  ``max_thrust_n`` is
    # therefore an explicit performance-envelope calibration parameter.  Its frozen
    # value is selected by calibrate_holoocean_thrust.py for approximately 1 m/s.
    MAX_THRUST_N = 501.1328125
    MOTOR_TIME_CONSTANT_S = 0.25
    HOLOOCEAN_NATIVE_MAX_THRUST_N = 1500.0

    def __init__(
        self,
        repository: str | Path,
        *,
        base_seed: int = 0,
        fixed_reset_seed: int | None = None,
        environment_factory: Callable[..., Any] | None = None,
        wind_mode: str = "off",
        wind_drag_coefficient: float = 1.0,
        wind_frontal_area_m2: float = 0.60,
        air_density_kg_m3: float = 1.225,
        settle_physics_steps: int = 0,
        shaping_enabled: bool = True,
        max_thrust_n: float | None = None,
    ) -> None:
        self.root = Path(repository)
        document = json.loads(
            (self.root / "artifacts/rl-campaign/surveyor/task-contract-frozen.json").read_text()
        )
        if document.get("content_sha256") != self.EXPECTED_CONTRACT_SHA256:
            raise ValueError("HoloOcean adapter requires the frozen 15-field contract")
        self.contract = next(
            task for task in document["tasks"] if task["task_id"] == "common-waypoint-transit-v1"
        )
        if sum(component["size"] for component in self.contract["observation"]["components"]) != 15:
            raise ValueError("unexpected observation size in frozen contract")

        timing = self.contract["timing"]
        self.physics_timestep_s = float(timing["physics_timestep_s"])
        self.control_interval_s = float(timing["control_interval_s"])
        self.physics_steps_per_action = round(self.control_interval_s / self.physics_timestep_s)
        self.timeout_steps = int(timing["episode_length_steps"])
        shaping = self.contract["reward"]["potential_shaping"]
        self.shaping_k = float(shaping["k"])
        self.shaping_gamma = float(shaping["gamma"])
        if not math.isclose(self.shaping_gamma, 1.0, rel_tol=0.0, abs_tol=0.0):
            raise ValueError("HoloOcean reward port requires shaping gamma exactly 1.0")
        self.shaping_enabled = bool(shaping_enabled)
        self.max_thrust_n = float(self.MAX_THRUST_N if max_thrust_n is None else max_thrust_n)
        if not 0.0 < self.max_thrust_n <= self.HOLOOCEAN_NATIVE_MAX_THRUST_N:
            raise ValueError("max_thrust_n must be in (0, 1500]")
        terminal = self.contract["learnability"]["absolute_success_rate_threshold"][
            "terminal_definition"
        ]
        self.final_radius_m = float(terminal["radius_m"])
        self.base_seed = int(base_seed)
        self.fixed_reset_seed = fixed_reset_seed
        self.episode = 0
        self._environment_factory = environment_factory
        if wind_mode not in {"off", "surge_equivalent"}:
            raise ValueError("wind_mode must be 'off' or 'surge_equivalent'")
        self.wind_mode = wind_mode
        self.wind_drag_coefficient = float(wind_drag_coefficient)
        self.wind_frontal_area_m2 = float(wind_frontal_area_m2)
        self.air_density_kg_m3 = float(air_density_kg_m3)
        if settle_physics_steps < 0:
            raise ValueError("settle_physics_steps must be non-negative")
        self.settle_physics_steps = int(settle_physics_steps)
        self._env: Any | None = None
        self._rng = np.random.default_rng(self.base_seed)
        self._previous_action = np.zeros(2, dtype=np.float32)
        self._physics_steps = 0
        self._yaw_ned_rad = 0.0
        self._route_nwu: list[list[float]] = []
        self._route_ned: list[list[float]] = []
        self._start_ned = np.zeros(2, dtype=np.float64)
        self._waypoint = 0
        self._previous_distance_m = 0.0
        self._previous_final_distance_m = 0.0
        self._control_steps = 0
        self._cross_track_sum_m = 0.0
        self._last_state: dict[str, Any] = {}
        self._last_randomization: dict[str, Any] = {}
        self._last_gps_sample_time_s: float | None = None
        self._last_gps_position_ned_m: np.ndarray | None = None
        self._ground_velocity_ned_mps = np.zeros(2, dtype=np.float64)
        self._last_wind_force_n = 0.0
        self._thruster_state_n = np.zeros(2, dtype=np.float64)
        self._termination = HoloOceanTerminationMonitor()

        self.action_space = gym.spaces.Box(-1.0, 1.0, (2,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (15,), dtype=np.float32)

    def _draw_randomization(self, seed: int) -> dict[str, Any]:
        ranges = self.contract["reset_randomization"]
        rng = Mulberry32(seed)

        def uniform(bounds):
            low, high = bounds
            return low + (high - low) * rng.next()

        route_angle_ned = math.radians(uniform(ranges["route_rotation_deg"]))
        offset_north = uniform(ranges["start_position_offset_m"])
        offset_east = uniform(ranges["start_position_offset_m"])

        # Contract world is NED. HoloOcean world is NWU: north=x, east=-y.
        route_nwu = []
        for north, east in ranges["route_relative_m"]:
            rotated_north = north * math.cos(route_angle_ned) - east * math.sin(route_angle_ned)
            rotated_east = north * math.sin(route_angle_ned) + east * math.cos(route_angle_ned)
            route_nwu.append([offset_north + rotated_north, -(offset_east + rotated_east)])

        # Keep the literal bcod-sim draw order: route, start N/E, current
        # magnitude/direction, wind magnitude/direction, then heading.
        current_speed = uniform(ranges["current_speed_m_s"])
        current_direction_ned = 2.0 * math.pi * rng.next()
        # Convert the horizontal NED vector to HoloOcean NWU.
        current_nwu = [
            current_speed * math.cos(current_direction_ned),
            -current_speed * math.sin(current_direction_ned),
            0.0,
        ]
        wind_speed = uniform(ranges["wind_speed_m_s"])
        wind_direction_ned = 2.0 * math.pi * rng.next()
        heading_ned = math.radians(uniform(ranges["start_heading_deg"]))
        return {
            "seed": seed,
            "spawn_nwu_m": [offset_north, -offset_east, 0.15],
            "heading_ned_rad": heading_ned,
            "route_nwu_m": route_nwu,
            "current_nwu_mps": current_nwu,
            "wind_ned_mps": [
                wind_speed * math.cos(wind_direction_ned),
                wind_speed * math.sin(wind_direction_ned),
                0.0,
            ],
        }

    def _scenario(self, randomization: dict[str, Any]) -> dict[str, Any]:
        sensors = [
            {
                "sensor_type": "GPSSensor",
                "sensor_name": "GPSSensor",
                "socket": "Platform",
                "Hz": round(1.0 / self.physics_timestep_s),
            },
            {
                "sensor_type": "IMUSensor",
                "sensor_name": "IMUSensor",
                "socket": "Platform",
                "Hz": round(1.0 / self.physics_timestep_s),
            },
            {
                "sensor_type": "MagnetometerSensor",
                "sensor_name": "MagnetometerSensor",
                "socket": "Platform",
                "Hz": round(1.0 / self.physics_timestep_s),
                "configuration": {
                    "Sigma": 0.0,
                    "MagneticVector": [1.0, 0.0, 0.0],
                },
            },
            {
                "sensor_type": "CollisionSensor",
                "sensor_name": "CollisionSensor",
                "socket": "Platform",
                "Hz": round(1.0 / self.physics_timestep_s),
            },
        ]
        configured = {sensor["sensor_type"] for sensor in sensors}
        if configured & self.FORBIDDEN_POLICY_SENSORS:
            raise AssertionError("privileged sensor configured on the policy path")
        return {
            "name": "holoocean_vehicle_a_oracle_safe",
            "package_name": "Ocean",
            "world": "OpenWater",
            "main_agent": "vehicle_a",
            "ticks_per_sec": round(1.0 / self.physics_timestep_s),
            "frames_per_sec": False,
            "agents": [
                {
                    "agent_name": "vehicle_a",
                    "agent_type": "SurfaceVessel",
                    "control_scheme": 0,
                    "location": randomization["spawn_nwu_m"],
                    # NED positive yaw is clockwise; NWU positive yaw is counter-clockwise.
                    "rotation": [0.0, 0.0, -math.degrees(randomization["heading_ned_rad"])],
                    "sensors": sensors,
                }
            ],
        }

    def _make_environment(self, scenario: dict[str, Any]) -> Any:
        if self._environment_factory is not None:
            return self._environment_factory(scenario_cfg=scenario, show_viewport=False)
        import holoocean  # Imported lazily so contract/unit tests do not require HoloOcean.

        return holoocean.make(scenario_cfg=scenario, show_viewport=False)

    def _shutdown_environment(self) -> None:
        if self._env is None:
            return
        # HoloOcean 2.3.0 provides context-manager cleanup but no public close().
        exit_method = getattr(self._env, "__exit__", None)
        if exit_method is not None:
            exit_method(None, None, None)
        self._env = None

    def _reset_or_create_environment(self) -> tuple[dict[str, Any], str]:
        """Reset the native world without relaunching Unreal between episodes.

        HoloOcean's public ``Environment.reset()`` reconstructs the configured
        agents inside the existing process.  The new episode's randomized pose
        is then applied with the public agent ``set_physics_state`` buffer.  A
        clock-free tick commits that buffer without consuming task time.

        The recreation fallback keeps lightweight test doubles and older
        HoloOcean clients usable; HoloOcean 2.3.0 takes the native-reset path.
        """
        scenario = self._scenario(self._last_randomization)
        if self._env is None:
            self._env = self._make_environment(scenario)
            return self._env.reset(), "process-created"

        agents = getattr(self._env, "agents", None)
        tick = getattr(self._env, "tick", None)
        if not isinstance(agents, dict) or not callable(tick):
            self._shutdown_environment()
            self._env = self._make_environment(scenario)
            return self._env.reset(), "process-recreated-compatibility-fallback"

        self._env.reset()
        agent = self._env.agents.get("vehicle_a")
        set_physics_state = getattr(agent, "set_physics_state", None)
        if not callable(set_physics_state):
            self._shutdown_environment()
            self._env = self._make_environment(scenario)
            return self._env.reset(), "process-recreated-compatibility-fallback"

        set_physics_state(
            np.asarray(self._last_randomization["spawn_nwu_m"], dtype=np.float32),
            np.asarray(
                [
                    0.0,
                    0.0,
                    -math.degrees(self._last_randomization["heading_ned_rad"]),
                ],
                dtype=np.float32,
            ),
            np.zeros(3, dtype=np.float32),
            np.zeros(3, dtype=np.float32),
        )
        state = tick(publish=False, tick_clock=False)
        return state, "native-reset-and-set-physics-state"

    @staticmethod
    def _flu_to_frd(vector: np.ndarray) -> np.ndarray:
        """Convert HoloOcean Platform-socket FLU values to contract body FRD."""
        return np.asarray([vector[0], -vector[1], -vector[2]], dtype=np.float64)

    def _state_time(self, state: dict[str, Any]) -> float:
        value = state.get("t", self._physics_steps * self.physics_timestep_s)
        return float(value) if isinstance(value, (int, float)) else self._physics_steps * self.physics_timestep_s

    def _read_gps(self, state: dict[str, Any]) -> tuple[np.ndarray | None, bool]:
        """Return the current GPS sample and a receipt-time freshness decision.

        HoloOcean 2.3.0 exposes neither a source timestamp nor a sequence number.
        Because this adapter configures GPS at every physics tick, a finite payload
        present in the rebuilt state dictionary is treated as received at ``state['t']``.
        The last sample is retained only to enforce a maximum age if the key disappears.
        """
        now = self._state_time(state)
        payload = state.get("GPSSensor")
        if isinstance(payload, np.ndarray) and payload.shape == (3,) and np.all(np.isfinite(payload)):
            position_ned = np.asarray([payload[0], -payload[1]], dtype=np.float64)
            if self._last_gps_position_ned_m is not None and self._last_gps_sample_time_s is not None:
                dt = now - self._last_gps_sample_time_s
                if dt > 1e-12:
                    self._ground_velocity_ned_mps = (
                        position_ned - self._last_gps_position_ned_m
                    ) / dt
            self._last_gps_position_ned_m = position_ned
            self._last_gps_sample_time_s = now
        if self._last_gps_position_ned_m is None or self._last_gps_sample_time_s is None:
            return None, False
        age = now - self._last_gps_sample_time_s
        fresh = -1e-12 <= age <= self.GPS_MAX_AGE_S + 1e-12
        return self._last_gps_position_ned_m.copy(), fresh

    def _wind_surge_force_n(self) -> float:
        """Map relative longitudinal wind to an equivalent native surge force.

        The prebuilt HoloOcean binary has no external force command. This mapping
        therefore injects the representable surge load equally through the two
        fixed thrusters. Lateral aerodynamic force is not representable and is not
        silently converted into yaw.
        """
        if self.wind_mode == "off":
            return 0.0
        wind_ned = np.asarray(self._last_randomization["wind_ned_mps"][:2], dtype=np.float64)
        forward_ned = np.asarray(
            [math.cos(self._yaw_ned_rad), math.sin(self._yaw_ned_rad)], dtype=np.float64
        )
        relative_longitudinal_mps = float(
            np.dot(wind_ned - self._ground_velocity_ned_mps, forward_ned)
        )
        return (
            0.5
            * self.air_density_kg_m3
            * self.wind_drag_coefficient
            * self.wind_frontal_area_m2
            * relative_longitudinal_mps
            * abs(relative_longitudinal_mps)
        )

    def _advance_thruster_state(self, target_n: np.ndarray) -> np.ndarray:
        """Apply bcod-sim's exact un-rate-limited first-order actuator step."""
        clamped_target = np.clip(target_n, -self.max_thrust_n, self.max_thrust_n)
        alpha = 1.0 - math.exp(
            -self.physics_timestep_s / self.MOTOR_TIME_CONSTANT_S
        )
        self._thruster_state_n += (clamped_target - self._thruster_state_n) * alpha
        return self._thruster_state_n.copy()

    def _observation(self, state: dict[str, Any]) -> np.ndarray:
        imu = state.get("IMUSensor")
        accel_frd = np.zeros(3, dtype=np.float64)
        rate_frd = np.zeros(3, dtype=np.float64)
        if isinstance(imu, np.ndarray) and imu.shape[0] >= 2 and imu.shape[1] == 3:
            accel_frd = self._flu_to_frd(imu[0])
            rate_frd = self._flu_to_frd(imu[1])

        magnetic_flu = state.get("MagnetometerSensor")
        if (
            isinstance(magnetic_flu, np.ndarray)
            and magnetic_flu.shape == (3,)
            and np.all(np.isfinite(magnetic_flu))
        ):
            # The configured north-pointing world field expressed in the Platform
            # FLU frame gives clockwise-positive NED heading directly.
            self._yaw_ned_rad = math.atan2(
                float(magnetic_flu[1]), float(magnetic_flu[0])
            )

        relative_goal = np.zeros(2, dtype=np.float64)
        fix_valid = 0.0
        gps_position_ned, gps_fresh = self._read_gps(state)
        if gps_fresh and gps_position_ned is not None:
            target = self._route_nwu[self._waypoint]
            relative_goal[0] = target[0] - gps_position_ned[0]
            relative_goal[1] = -target[1] - gps_position_ned[1]
            fix_valid = 1.0

        remaining = max(0.0, (self.timeout_steps - self._physics_steps) / self.timeout_steps)
        return np.asarray(
            [
                *accel_frd,
                *rate_frd,
                self._yaw_ned_rad,
                *relative_goal,
                fix_valid,
                self._previous_action[0],
                self._previous_action[1],
                0.0,
                0.0,
                remaining,
            ],
            dtype=np.float32,
        )

    def _distance_to_waypoint(self, waypoint: int) -> float:
        if self._last_gps_position_ned_m is None:
            raise RuntimeError("reward evaluation requires a valid GPS position")
        return float(
            np.linalg.norm(
                np.asarray(self._route_ned[waypoint], dtype=np.float64)
                - self._last_gps_position_ned_m
            )
        )

    def _cross_track_distance(self) -> float:
        if self._last_gps_position_ned_m is None:
            raise RuntimeError("reward evaluation requires a valid GPS position")
        leg_start = self._start_ned if self._waypoint == 0 else self._route_ned[self._waypoint - 1]
        return cross_track_distance(
            self._last_gps_position_ned_m.tolist(), leg_start, self._route_ned[self._waypoint]
        )

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None):
        super().reset(seed=seed)
        actual_seed = (
            int(self.fixed_reset_seed)
            if self.fixed_reset_seed is not None
            else int(seed) if seed is not None else self.base_seed + self.episode
        )
        self.episode += 1
        self._rng = np.random.default_rng(actual_seed)
        self.action_space.seed(actual_seed)
        self._last_randomization = self._draw_randomization(actual_seed)
        self._route_nwu = self._last_randomization["route_nwu_m"]
        self._route_ned = [[point[0], -point[1]] for point in self._route_nwu]
        self._start_ned = np.asarray(
            [
                self._last_randomization["spawn_nwu_m"][0],
                -self._last_randomization["spawn_nwu_m"][1],
            ],
            dtype=np.float64,
        )
        self._waypoint = 0
        # Policy yaw must be sensor-derived; do not seed it from spawn ground truth.
        self._yaw_ned_rad = 0.0
        self._physics_steps = 0
        self._previous_action[:] = 0.0
        self._last_gps_sample_time_s = None
        self._last_gps_position_ned_m = None
        self._ground_velocity_ned_mps[:] = 0.0
        self._last_wind_force_n = 0.0
        self._thruster_state_n[:] = 0.0
        self._termination = HoloOceanTerminationMonitor()
        self._control_steps = 0
        self._cross_track_sum_m = 0.0
        self._last_state, reset_mode = self._reset_or_create_environment()
        for _ in range(self.settle_physics_steps):
            self._last_state = self._env.step(np.zeros(2, dtype=np.float64), 1)
            self._observation(self._last_state)
        # Settling is outside the episode. Apply disturbance only afterward so
        # settling does not consume a randomized-current trajectory segment.
        self._env.set_ocean_currents("vehicle_a", self._last_randomization["current_nwu_mps"])
        observation = self._observation(self._last_state)
        self._previous_distance_m = self._distance_to_waypoint(self._waypoint)
        self._previous_final_distance_m = self._distance_to_waypoint(
            len(self._route_ned) - 1
        )
        self._completion_tracker = CompletionTracker.for_route(
            self._start_ned, self._route_ned
        )
        return observation, {
            "seed": actual_seed,
            "oracle_safe_policy_sensors": [
                "GPSSensor",
                "IMUSensor",
                "MagnetometerSensor",
            ],
            "yaw_mapping": "atan2(mag_y, mag_x) from Platform-FLU MagnetometerSensor",
            "magnetometer_caveat": "simplified model: Gaussian noise only; no bias, drift, calibration error, field variation, or magnetic interference",
            "termination_sensors": ["CollisionSensor", "IMUSensor"],
            "collision_mapping": "native boolean CollisionSensor maps to generic collision; grounding versus object is unavailable because counterpart identity is discarded",
            "instability_mapping": "IMU gravity-direction combined tilt > 60 degrees for 1.0 continuous simulation second; non-finite IMU is immediate",
            "allocation_mapping": "requested versus wrapper-applied native thruster force residual > 20% for 1.0 continuous simulation second; no achieved-force feedback exists",
            "actuator_mapping": f"normalized command linearly maps to +/-{self.max_thrust_n:g} N per thruster; performance-calibrated for approximately 1 m/s HoloOcean cruise after literal +/-95 N force matching proved dynamically incomparable; exact exponential first-order lag tau=0.25 s applied every 0.05 s physics tick",
            "reward_mapping": "shared common_task.compute_reward; shaping gamma=1.0 and k from frozen contract",
            "gps_freshness": "wrapper receipt time; HoloOcean payload has no timestamp or sequence",
            "current_applied": True,
            "wind_mode": self.wind_mode,
            "wind_applied": self.wind_mode != "off",
            "wind_mapping": "longitudinal aerodynamic force injected equally through native fixed thrusters; lateral load unavailable",
            "settle_physics_steps": self.settle_physics_steps,
            "simulator_reset_mode": reset_mode,
        }

    def step(self, action: np.ndarray):
        if self._env is None:
            raise RuntimeError("reset() must be called before step()")
        raw_action = np.asarray(action, dtype=np.float64)
        if raw_action.shape != (2,):
            raise ValueError("Vehicle A action must have shape (2,)")
        normalized = np.clip(raw_action, -1.0, 1.0)
        previous_action = self._previous_action.copy()
        self._last_wind_force_n = self._wind_surge_force_n()
        requested_thruster_force = normalized * self.max_thrust_n
        thruster_force = self._thruster_state_n.copy()
        colliding = False
        if np.all(np.isfinite(requested_thruster_force)):
            for _ in range(self.physics_steps_per_action):
                propulsion_force = self._advance_thruster_state(requested_thruster_force)
                thruster_force = np.clip(
                    propulsion_force + self._last_wind_force_n / 2.0,
                    -self.HOLOOCEAN_NATIVE_MAX_THRUST_N,
                    self.HOLOOCEAN_NATIVE_MAX_THRUST_N,
                )
                self._last_state = self._env.step(thruster_force, 1)
                collision_payload = self._last_state.get("CollisionSensor")
                colliding = colliding or bool(
                    isinstance(collision_payload, np.ndarray)
                    and collision_payload.size == 1
                    and bool(collision_payload.reshape(-1)[0])
                )
        self._physics_steps += self.physics_steps_per_action
        self._previous_action = normalized.astype(np.float32)
        observation = self._observation(self._last_state)
        simulator_reason = self._termination.update(
            dt_s=self.control_interval_s,
            acceleration_body_frd_mps2=observation[:3],
            colliding=colliding,
            # Allocation health compares the post-actuator command presented to
            # HoloOcean with the value HoloOcean can accept. Comparing against
            # the pre-lag target falsely classifies the intentional 0.25 s motor
            # response as an allocation failure during ordinary action changes.
            commanded_thrust_n=self._thruster_state_n + self._last_wind_force_n / 2.0,
            applied_thrust_n=thruster_force,
        )
        distance_m = self._distance_to_waypoint(self._waypoint)
        cross_track_m = self._cross_track_distance()
        self._cross_track_sum_m += cross_track_m
        if (
            self._waypoint < len(self._route_ned) - 1
            and (
                distance_m <= 6.0
                or passed_waypoint_plane(
                    self._last_gps_position_ned_m.tolist(),
                    self._start_ned if self._waypoint == 0 else self._route_ned[self._waypoint - 1],
                    self._route_ned[self._waypoint],
                )
            )
        ):
            self._waypoint += 1
            next_previous_distance_m = self._distance_to_waypoint(self._waypoint)
        else:
            next_previous_distance_m = distance_m
        final_distance_m = self._distance_to_waypoint(len(self._route_ned) - 1)
        success = self._waypoint == len(self._route_ned) - 1 and final_distance_m <= self.final_radius_m
        timed_out = self._physics_steps >= self.timeout_steps
        termination_reason = classify_termination(
            success=success,
            collision_type="object_collision" if simulator_reason == "collision" else None,
            allocation_failed=simulator_reason == "allocation_failure",
            unstable=simulator_reason == "instability",
            timed_out=timed_out,
        )
        terminated = termination_reason not in ("running", "timeout")
        truncated = termination_reason == "timeout"
        scored = compute_reward(
            self._previous_distance_m,
            distance_m,
            cross_track_m,
            normalized,
            previous_action,
            termination_reason,
            previous_final_distance_m=self._previous_final_distance_m,
            final_distance_m=final_distance_m,
            shaping_k=self.shaping_k,
            shaping_gamma=self.shaping_gamma,
            shaping_enabled=self.shaping_enabled,
        )
        completion_fraction = self._completion_tracker.update(scored.progress_reward)
        self._previous_distance_m = next_previous_distance_m
        self._previous_final_distance_m = final_distance_m
        self._control_steps += 1
        if self._waypoint > 0:
            observation = self._observation(self._last_state)
        return observation, scored.reward, terminated, truncated, {
            "diagnostic_only": True,
            "reward_port_active": True,
            "physics_steps": self._physics_steps,
            "gps_fix_valid": bool(observation[9]),
            "wind_mode": self.wind_mode,
            "wind_surge_force_n": self._last_wind_force_n,
            "applied_thruster_force_n": thruster_force.tolist(),
            "actuator_target_force_n": requested_thruster_force.tolist(),
            "actuator_lagged_force_n": self._thruster_state_n.tolist(),
            "actuator_time_constant_s": self.MOTOR_TIME_CONSTANT_S,
            "wind_lateral_force_supported": False,
            "termination_reason": termination_reason,
            "success": success,
            "completion_fraction": completion_fraction,
            "waypoints_reached": self._waypoint + (1 if success else 0),
            "current_waypoint_index": self._waypoint,
            "final_leg_active": self._waypoint == len(self._route_ned) - 1,
            "distance_to_current_waypoint_m": self._distance_to_waypoint(self._waypoint),
            "distance_to_final_waypoint_m": final_distance_m,
            "mean_cross_track_m": self._cross_track_sum_m / self._control_steps,
            "control_steps": self._control_steps,
            "reward_components": scored.components(),
            "collision_type": "unclassified" if colliding else "none",
            "collision_classification_available": False,
        }

    def close(self) -> None:
        self._shutdown_environment()
