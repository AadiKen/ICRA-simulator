"""Gymnasium adapter for the validated Stonefish common-waypoint task."""

from __future__ import annotations

from pathlib import Path

import gymnasium as gym
import numpy as np

from stonefish_bridge import StonefishBridge
from stonefish_task import StonefishCommonTask


class StonefishGymEnv(gym.Env[np.ndarray, np.ndarray]):
    """Expose Stonefish's two live thrusters as the portable policy action."""

    metadata = {"render_modes": []}
    backend_type = "stonefish"
    vehicle_preset = "vehicle-a-otter"

    def __init__(
        self,
        repository: str | Path,
        *,
        executable: str | Path,
        data_dir: str | Path,
        library_dirs: tuple[str | Path, ...] = (),
        physics_threads: int | None = 1,
        sensor_noise: bool = False,
        base_seed: int = 0,
        fixed_reset_seed: int | None = None,
    ) -> None:
        self.repository = Path(repository)
        self.base_seed = int(base_seed)
        self.fixed_reset_seed = fixed_reset_seed
        self.episode = 0
        self.bridge = StonefishBridge(
            executable, data_dir, library_dirs=library_dirs,
            physics_threads=physics_threads, sensor_noise=sensor_noise,
        )
        self.task = StonefishCommonTask(
            self.bridge,
            self.repository / "artifacts/rl-campaign/surveyor/task-contract-frozen.json",
        )
        self.action_space = gym.spaces.Box(-1.0, 1.0, (2,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(
            -np.inf, np.inf, (15,), dtype=np.float32)

    def reset(self, *, seed: int | None = None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            episode_seed = int(seed)
        elif self.fixed_reset_seed is not None:
            episode_seed = int(self.fixed_reset_seed)
        else:
            episode_seed = self.base_seed + self.episode
        self.episode += 1
        observation, info = self.task.reset(episode_seed)
        return np.asarray(observation, dtype=np.float32), info

    def step(self, action):
        policy_action = np.asarray(action, dtype=np.float32)
        if policy_action.shape != (2,):
            raise ValueError("Stonefish policy action must have shape (2,)")
        observation, reward, terminated, truncated, info = self.task.step(
            [float(policy_action[0]), float(policy_action[1]), 0.0, 0.0]
        )
        return (np.asarray(observation, dtype=np.float32), float(reward),
                bool(terminated), bool(truncated), info)

    def close(self) -> None:
        self.bridge.close()
