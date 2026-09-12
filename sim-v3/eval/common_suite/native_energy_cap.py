"""Shared native-evaluation propulsion-impulse cap."""

from __future__ import annotations

from typing import Any

import gymnasium as gym
import numpy as np


class EnergyCapWrapper(gym.Wrapper):
    """Apply one propulsion impulse budget consistently to every native arm.

    Native environments expose a two-thruster policy-only force sample in
    ``info['policy_propulsion_thrust_n']``. Environmental forces must never be
    included in that field.
    """

    def __init__(self, env: gym.Env, energy_cap_ns: float | None = None) -> None:
        super().__init__(env)
        contract = getattr(env.unwrapped, "contract", {})
        termination = contract.get("evaluation_termination", {})
        self.energy_cap_ns = float(
            termination.get("energy_cap_ns") if energy_cap_ns is None else energy_cap_ns
        )
        if not np.isfinite(self.energy_cap_ns) or self.energy_cap_ns <= 0:
            raise ValueError("energy_cap_ns must be finite and positive")
        self.control_interval_s = float(getattr(env.unwrapped, "control_interval_s"))
        self.propulsion_impulse_ns = 0.0

    def reset(self, **kwargs: Any):
        self.propulsion_impulse_ns = 0.0
        observation, info = self.env.reset(**kwargs)
        return observation, {
            **info,
            "energy_cap_ns": self.energy_cap_ns,
            "propulsion_impulse_ns": 0.0,
        }

    def step(self, action: Any):
        observation, reward, terminated, truncated, info = self.env.step(action)
        thrust = np.asarray(info.get("policy_propulsion_thrust_n"), dtype=np.float64)
        if thrust.shape != (2,) or not np.all(np.isfinite(thrust)):
            raise RuntimeError(
                "native arm did not expose two finite policy-only thruster forces"
            )
        self.propulsion_impulse_ns += self.control_interval_s * float(np.abs(thrust).sum())
        exceeded = self.propulsion_impulse_ns > self.energy_cap_ns
        updated = {
            **info,
            "energy_cap_ns": self.energy_cap_ns,
            "propulsion_impulse_ns": self.propulsion_impulse_ns,
            "energy_cap_exceeded": exceeded,
        }
        if exceeded and updated.get("success", False):
            updated["success"] = False
            updated["termination_reason"] = "energy_cap_exceeded"
        return observation, reward, terminated, truncated, updated
