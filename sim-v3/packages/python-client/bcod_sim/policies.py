from __future__ import annotations
from dataclasses import dataclass
import numpy as np

@dataclass
class PIDPolicy:
    kp: float = .8
    kd: float = .25
    limit: float = 1.0
    def action(self, observation: np.ndarray, target: np.ndarray | None = None) -> np.ndarray:
        target = np.zeros(6) if target is None else np.asarray(target, dtype=np.float64)
        command = self.kp * (target - observation[:6]) - self.kd * observation[6:]
        return np.clip(command, -self.limit, self.limit).astype(np.float32)

@dataclass
class MPCPolicy:
    horizon_steps: int = 10
    effort_weight: float = .05
    def action(self, observation: np.ndarray, target: np.ndarray | None = None) -> np.ndarray:
        target = np.zeros(6) if target is None else np.asarray(target, dtype=np.float64)
        gain = self.horizon_steps / (self.horizon_steps + self.effort_weight)
        return np.clip(gain * (target - observation[:6]) - .5 * observation[6:], -1, 1).astype(np.float32)
