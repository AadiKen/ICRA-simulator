from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import gymnasium as gym
import numpy as np


@dataclass(frozen=True)
class EnvConfig:
    timestep_s: float = 0.02
    duration_s: float = 60.0
    seed: int = 0
    mass_kg: float = 100.0
    linear_damping: float = 10.0
    oracle_access: bool = False
    plant: str = "coupled6"


class BCODGymEnv(gym.Env[np.ndarray, np.ndarray]):
    """Deterministic Gymnasium contract for the local process backend.

    The state is a transport-neutral 6-DoF pose/velocity vector. Production
    hydrodynamics are supplied by the resolved backend; this class owns only
    Gym lifecycle, termination, seeding, and observation isolation.
    """

    metadata = {"render_modes": []}

    def __init__(self, config: EnvConfig | None = None) -> None:
        self.config = config or EnvConfig()
        if self.config.timestep_s <= 0 or self.config.duration_s <= 0:
            raise ValueError("timestep_s and duration_s must be positive")
        if self.config.plant not in {"planar3", "coupled6"}:
            raise ValueError("plant must be planar3 or coupled6")
        self._duration_steps = int(np.ceil(self.config.duration_s / self.config.timestep_s - 1e-12))
        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(6,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, shape=(12,), dtype=np.float32)
        self._state = np.zeros(12, dtype=np.float64)
        self._elapsed = 0.0
        self._steps = 0

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None):
        super().reset(seed=self.config.seed if seed is None else seed)
        self._state.fill(0.0)
        self._elapsed = 0.0
        self._steps = 0
        if options and "initial_state" in options:
            initial = np.asarray(options["initial_state"], dtype=np.float64)
            if initial.shape != (12,) or not np.isfinite(initial).all():
                raise ValueError("initial_state must contain 12 finite values")
            self._state[:] = initial
        return self._observation(), {"seed": seed if seed is not None else self.config.seed}

    def step(self, action: np.ndarray):
        command = np.asarray(action, dtype=np.float64)
        if command.shape != (6,) or not np.isfinite(command).all():
            raise ValueError("action must contain 6 finite values")
        dt = self.config.timestep_s
        velocity = self._state[6:]
        acceleration = (command - self.config.linear_damping * velocity) / self.config.mass_kg
        velocity += acceleration * dt
        self._state[:6] += velocity * dt
        self._steps += 1
        self._elapsed = self._steps * dt
        truncated = self._steps >= self._duration_steps
        if self.config.plant == "planar3":
            self._state[[2, 3, 4, 8, 9, 10]] = 0
        info = {"elapsed_s": self._elapsed, "steps": self._steps}
        if self.config.oracle_access:
            info["ground_truth"] = self.get_ground_truth()
        pose_cost = float(np.dot(self._state[:6], self._state[:6]))
        velocity_cost = float(np.dot(self._state[6:], self._state[6:]))
        effort_cost = float(np.dot(command, command))
        reward = -(pose_cost + 0.1 * velocity_cost + 0.01 * effort_cost) * dt
        return self._observation(), reward, False, truncated, info

    def _observation(self) -> np.ndarray:
        return self._state.astype(np.float32, copy=True)

    def get_ground_truth(self) -> np.ndarray:
        return self._state.copy()

    def save_checkpoint(self) -> dict[str, Any]:
        return {"version": 1, "state": self._state.tolist(), "steps": self._steps, "rng_state": self.np_random.bit_generator.state}

    def load_checkpoint(self, checkpoint: dict[str, Any]) -> None:
        if checkpoint.get("version") != 1:
            raise ValueError("unsupported checkpoint version")
        state = np.asarray(checkpoint.get("state"), dtype=np.float64)
        if state.shape != (12,) or not np.isfinite(state).all():
            raise ValueError("checkpoint state must contain 12 finite values")
        steps = checkpoint.get("steps")
        if not isinstance(steps, int) or steps < 0:
            raise ValueError("checkpoint steps must be a non-negative integer")
        self._state[:] = state
        self._steps = steps
        self._elapsed = steps * self.config.timestep_s
        self.np_random.bit_generator.state = checkpoint["rng_state"]


class BCODSim(BCODGymEnv):
    """Canonical local single-environment entry point."""


class BCODVectorEnv(gym.vector.SyncVectorEnv):
    def __init__(self, count: int, config: EnvConfig | None = None) -> None:
        if count < 1:
            raise ValueError("count must be positive")
        super().__init__([lambda config=config: BCODGymEnv(config) for _ in range(count)])
