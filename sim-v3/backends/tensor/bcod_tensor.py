from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import torch


@dataclass(frozen=True)
class VehicleAPlanar3Config:
    """MSS Otter parameters used by the production Vehicle A planar3 path."""

    environments: int
    timestep_s: float = 0.02
    device: str = "cpu"
    numerics_mode: str = "float64"
    experiment_seed: str | int = 0
    environment_indices: tuple[int, ...] | None = None
    mass_kg: float = 55.0
    cg_x_m: float = 0.2
    yaw_inertia_kg_m2: float = 15.95
    xu_dot: float = -5.28152100957639
    yv_dot: float = -82.5
    nr_dot: float = -23.375
    linear_damping: tuple[float, float, float] = (77.5334370139969, 137.5, 39.325)
    quadratic_damping: tuple[float, float, float] = (0.0, 0.0, 393.25)
    thruster_y_m: tuple[float, float] = (-0.54, 0.54)
    thruster_min_n: float = -95.0
    thruster_max_n: float = 95.0
    thruster_tau_s: float = 0.25


UINT32_MASK = 0xFFFFFFFF
MULBERRY32_INCREMENT = 0x6D2B79F5


def fnv1a_environment_seed(experiment_seed: str | int, environment_index: int) -> int:
    """Hash UTF-8(seed) + NUL + UTF-8("environment:") + UTF-8(decimal index)."""
    if isinstance(environment_index, bool) or not isinstance(environment_index, int) or environment_index < 0:
        raise ValueError("environment_index must be a non-negative integer")
    payload = str(experiment_seed).encode("utf-8") + b"\x00environment:" + str(environment_index).encode("utf-8")
    value = 0x811C9DC5
    for byte in payload:
        value = ((value ^ byte) * 0x01000193) & UINT32_MASK
    return value


class VehicleAPlanar3TensorBackend:
    """Batched float64 reproduction of production Vehicle A planar3 physics.

    Commands are normalized port/starboard commands in [-1, 1]. The production
    actuator advances once before RK4, so its resulting wrench is held constant
    through the four derivative evaluations.
    """

    def __init__(self, config: VehicleAPlanar3Config) -> None:
        if config.environments < 1:
            raise ValueError("environments must be positive")
        if not config.timestep_s > 0:
            raise ValueError("timestep_s must be positive")
        if config.device == "mps" and not torch.backends.mps.is_available():
            raise RuntimeError("MPS was requested but is unavailable")
        if config.device == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA was requested but is unavailable")
        self.config = config
        self.device = torch.device(config.device)
        if config.numerics_mode not in {"float64", "float32"}:
            raise ValueError("numerics_mode must be float64 or float32")
        if config.device == "mps" and config.numerics_mode != "float32":
            raise ValueError("MPS requires numerics_mode=float32")
        self.dtype = torch.float64 if config.numerics_mode == "float64" else torch.float32
        shape3 = (config.environments, 3)
        shape2 = (config.environments, 2)
        self.pose_ned_yaw = torch.zeros(shape3, dtype=self.dtype, device=self.device)
        self.body_velocity = torch.zeros(shape3, dtype=self.dtype, device=self.device)
        self.body_acceleration = torch.zeros(shape3, dtype=self.dtype, device=self.device)
        self.thruster_state = torch.zeros(shape2, dtype=self.dtype, device=self.device)
        self.energy_j = torch.zeros(config.environments, dtype=self.dtype, device=self.device)
        self.power_w = torch.zeros(config.environments, dtype=self.dtype, device=self.device)
        self.step_counters = torch.zeros(config.environments, dtype=torch.int64, device=self.device)
        self.active_mask = torch.ones(config.environments, dtype=torch.bool, device=self.device)
        indices = config.environment_indices if config.environment_indices is not None else tuple(range(config.environments))
        if len(indices) != config.environments or len(set(indices)) != len(indices):
            raise ValueError("environment_indices must contain one unique index per environment")
        if any(isinstance(index, bool) or not isinstance(index, int) or index < 0 for index in indices):
            raise ValueError("environment_indices must be non-negative integers")
        self.environment_indices = torch.tensor(indices, dtype=torch.int64, device=self.device)
        self.initial_rng_state = torch.tensor(
            [fnv1a_environment_seed(config.experiment_seed, index) for index in indices],
            dtype=torch.int64,
            device=self.device,
        )
        self.rng_state = self.initial_rng_state.clone()

        mass_matrix = self._planar_mass_matrix(config)
        # Mirror core/math.js invert3 operation ordering instead of delegating to
        # a backend-specific factorization.
        self.mass_matrix = mass_matrix
        self.mass_matrix_inv = self._invert3(mass_matrix)
        self.linear_damping = self._constant(config.linear_damping)
        self.quadratic_damping = self._constant(config.quadratic_damping)
        self.thruster_y_m = self._constant(config.thruster_y_m)

    def _constant(self, value: tuple[float, ...]) -> torch.Tensor:
        return torch.tensor(value, dtype=self.dtype, device=self.device)

    def _planar_mass_matrix(self, cfg: VehicleAPlanar3Config) -> torch.Tensor:
        return torch.tensor(
            [
                [cfg.mass_kg - cfg.xu_dot, 0.0, 0.0],
                [0.0, cfg.mass_kg - cfg.yv_dot, cfg.mass_kg * cfg.cg_x_m],
                [0.0, cfg.mass_kg * cfg.cg_x_m, cfg.yaw_inertia_kg_m2 - cfg.nr_dot],
            ],
            dtype=self.dtype,
            device=self.device,
        )

    @staticmethod
    def _invert3(matrix: torch.Tensor) -> torch.Tensor:
        a, b, c = matrix[0]
        d, e, f = matrix[1]
        g, h, i = matrix[2]
        det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
        inv_det = 1.0 / det
        return torch.stack(
            (
                torch.stack(((e * i - f * h) * inv_det, (c * h - b * i) * inv_det, (b * f - c * e) * inv_det)),
                torch.stack(((f * g - d * i) * inv_det, (a * i - c * g) * inv_det, (c * d - a * f) * inv_det)),
                torch.stack(((d * h - e * g) * inv_det, (b * g - a * h) * inv_det, (a * e - b * d) * inv_det)),
            )
        )

    @property
    def state(self) -> torch.Tensor:
        """Compatibility view: [N, E, yaw, u, v, r]."""
        return torch.cat((self.pose_ned_yaw, self.body_velocity), dim=1)

    def reset(self, mask: torch.Tensor | None = None) -> torch.Tensor:
        selected = torch.ones_like(self.active_mask) if mask is None else mask.to(device=self.device, dtype=torch.bool)
        if selected.shape != self.active_mask.shape:
            raise ValueError("reset mask has the wrong shape")
        for value in (
            self.pose_ned_yaw,
            self.body_velocity,
            self.body_acceleration,
            self.thruster_state,
            self.energy_j,
            self.power_w,
            self.step_counters,
        ):
            value[selected] = 0
        self.rng_state[selected] = self.initial_rng_state[selected]
        self.active_mask[selected] = True
        return self.state.clone()

    def random_uniform(self, mask: torch.Tensor | None = None) -> torch.Tensor:
        """Draw one independent Mulberry32 sample for each selected environment."""
        selected = self.active_mask.clone() if mask is None else mask.to(device=self.device, dtype=torch.bool)
        if selected.shape != self.active_mask.shape:
            raise ValueError("RNG mask has the wrong shape")
        state = (self.rng_state + MULBERRY32_INCREMENT) & UINT32_MASK
        z = state
        z = ((z ^ (z >> 15)) * (z | 1)) & UINT32_MASK
        z = (z ^ ((z + (((z ^ (z >> 7)) * (z | 61)) & UINT32_MASK)) & UINT32_MASK)) & UINT32_MASK
        output_bits = (z ^ (z >> 14)) & UINT32_MASK
        self.rng_state[selected] = state[selected]
        output = output_bits.to(dtype=self.dtype) / 4294967296.0
        return torch.where(selected, output, torch.zeros_like(output))

    def save_checkpoint(self) -> dict[str, Any]:
        """Return a detached checkpoint including every per-environment RNG stream."""
        return {
            "version": 1,
            "experiment_seed": str(self.config.experiment_seed),
            "environment_indices": self.environment_indices.detach().cpu().tolist(),
            "pose_ned_yaw": self.pose_ned_yaw.detach().cpu().clone(),
            "body_velocity": self.body_velocity.detach().cpu().clone(),
            "body_acceleration": self.body_acceleration.detach().cpu().clone(),
            "thruster_state": self.thruster_state.detach().cpu().clone(),
            "energy_j": self.energy_j.detach().cpu().clone(),
            "power_w": self.power_w.detach().cpu().clone(),
            "step_counters": self.step_counters.detach().cpu().clone(),
            "active_mask": self.active_mask.detach().cpu().clone(),
            "rng_state": self.rng_state.detach().cpu().clone(),
        }

    def load_checkpoint(self, checkpoint: dict[str, Any]) -> None:
        """Validate the complete checkpoint before mutating backend state."""
        if checkpoint.get("version") != 1:
            raise ValueError("unsupported tensor backend checkpoint version")
        if checkpoint.get("experiment_seed") != str(self.config.experiment_seed):
            raise ValueError("checkpoint experiment seed does not match backend")
        if checkpoint.get("environment_indices") != self.environment_indices.detach().cpu().tolist():
            raise ValueError("checkpoint environment indices do not match backend")
        specifications = {
            "pose_ned_yaw": (self.pose_ned_yaw, self.dtype),
            "body_velocity": (self.body_velocity, self.dtype),
            "body_acceleration": (self.body_acceleration, self.dtype),
            "thruster_state": (self.thruster_state, self.dtype),
            "energy_j": (self.energy_j, self.dtype),
            "power_w": (self.power_w, self.dtype),
            "step_counters": (self.step_counters, torch.int64),
            "active_mask": (self.active_mask, torch.bool),
            "rng_state": (self.rng_state, torch.int64),
        }
        validated: dict[str, torch.Tensor] = {}
        for name, (destination, dtype) in specifications.items():
            try:
                value = torch.as_tensor(checkpoint[name], dtype=dtype, device=self.device)
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError(f"checkpoint {name} is missing or invalid") from error
            if value.shape != destination.shape:
                raise ValueError(f"checkpoint {name} has shape {tuple(value.shape)}; expected {tuple(destination.shape)}")
            if dtype.is_floating_point and not torch.isfinite(value).all():
                raise ValueError(f"checkpoint {name} must be finite")
            if name == "step_counters" and torch.any(value < 0):
                raise ValueError("checkpoint step counters must be non-negative")
            if name == "rng_state" and (torch.any(value < 0) or torch.any(value > UINT32_MASK)):
                raise ValueError("checkpoint RNG states must be unsigned 32-bit values")
            validated[name] = value.clone()
        for name, value in validated.items():
            getattr(self, name).copy_(value)

    def set_active(self, mask: torch.Tensor) -> None:
        selected = mask.to(device=self.device, dtype=torch.bool)
        if selected.shape != self.active_mask.shape:
            raise ValueError("active mask has the wrong shape")
        self.active_mask.copy_(selected)

    def _advance_thrusters(self, commands: torch.Tensor) -> torch.Tensor:
        targets = torch.where(commands >= 0, commands * self.config.thruster_max_n, commands * abs(self.config.thruster_min_n))
        alpha = 1.0 - torch.exp(torch.tensor(-self.config.timestep_s / self.config.thruster_tau_s, dtype=self.dtype, device=self.device))
        return self.thruster_state + (targets - self.thruster_state) * alpha

    def _derivative(self, state: torch.Tensor, applied_wrench: torch.Tensor, current_ned_mps: torch.Tensor) -> torch.Tensor:
        yaw = state[:, 2]
        velocity = state[:, 3:]
        cos_yaw, sin_yaw = torch.cos(yaw), torch.sin(yaw)
        current_u = cos_yaw * current_ned_mps[:, 0] + sin_yaw * current_ned_mps[:, 1]
        current_v = -sin_yaw * current_ned_mps[:, 0] + cos_yaw * current_ned_mps[:, 1]
        relative = torch.stack((velocity[:, 0] - current_u, velocity[:, 1] - current_v, velocity[:, 2]), dim=1)

        u, v, r = velocity.unbind(1)
        relative_u, relative_v, relative_r = relative.unbind(1)
        mass, cg_x = self.config.mass_kg, self.config.cg_x_m
        a1 = self.config.xu_dot * relative_u
        a2 = self.config.yv_dot * relative_v
        # -(C_RB(nu) + C_A(nu_r)) nu_r, expanded with the same three-term
        # row reductions as the Node reference.
        c02 = -mass * (cg_x * r + v) + a2
        c12 = mass * u - a1
        c20 = mass * (cg_x * r + v) - a2
        c21 = -mass * u + a1
        coriolis = torch.stack(
            (
                -(c02 * relative_r),
                -(c12 * relative_r),
                -(c20 * relative_u + c21 * relative_v),
            ),
            dim=1,
        )
        damping = -(self.linear_damping * relative + self.quadratic_damping * torch.abs(relative) * relative)
        wrench = applied_wrench + coriolis + damping
        acceleration = torch.stack(
            tuple(sum(self.mass_matrix_inv[row, col] * wrench[:, col] for col in range(3)) for row in range(3)),
            dim=1,
        )
        eta_dot = torch.stack((cos_yaw * u - sin_yaw * v, sin_yaw * u + cos_yaw * v, r), dim=1)
        return torch.cat((eta_dot, acceleration), dim=1)

    def step(self, commands: torch.Tensor, current_ned_mps: torch.Tensor | None = None) -> torch.Tensor:
        command = commands.to(device=self.device, dtype=self.dtype)
        if command.shape != self.thruster_state.shape or not torch.isfinite(command).all():
            raise ValueError("commands must be finite with shape [environments, 2]")
        command = torch.clamp(command, -1.0, 1.0)
        current = torch.zeros_like(self.pose_ned_yaw) if current_ned_mps is None else current_ned_mps.to(device=self.device, dtype=self.dtype)
        if current.shape != self.pose_ned_yaw.shape or not torch.isfinite(current).all():
            raise ValueError("current_ned_mps must be finite with shape [environments, 3]")

        next_thrusters = self._advance_thrusters(command)
        applied_wrench = torch.stack(
            (
                next_thrusters[:, 0] + next_thrusters[:, 1],
                torch.zeros(self.config.environments, dtype=self.dtype, device=self.device),
                -self.thruster_y_m[0] * next_thrusters[:, 0] - self.thruster_y_m[1] * next_thrusters[:, 1],
            ),
            dim=1,
        )
        y0 = self.state
        dt = self.config.timestep_s
        k1 = self._derivative(y0, applied_wrench, current)
        k2 = self._derivative(y0 + k1 * (dt * 0.5), applied_wrench, current)
        k3 = self._derivative(y0 + k2 * (dt * 0.5), applied_wrench, current)
        k4 = self._derivative(y0 + k3 * dt, applied_wrench, current)
        next_state = y0 + (k1 + 2.0 * k2 + 2.0 * k3 + k4) * (dt / 6.0)
        final_derivative = self._derivative(next_state, applied_wrench, current)

        selected = self.active_mask
        self.pose_ned_yaw[selected] = next_state[selected, :3]
        self.body_velocity[selected] = next_state[selected, 3:]
        self.body_acceleration[selected] = final_derivative[selected, 3:]
        self.thruster_state[selected] = next_thrusters[selected]
        # Vehicle A's current actuator preset has no integrated power model;
        # these explicit fields remain zero exactly as in production.
        self.power_w[selected] = 0.0
        self.step_counters[selected] += 1
        return self.state.clone()
