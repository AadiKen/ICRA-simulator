"""Small constrained linear MPC for Vehicle A's planar3/twin-fixed plant.

The prediction state includes delivered port/starboard thrust.  Commands are bounded
at the plant's documented +/-70 N limits and enter through the documented 0.35 s
first-order actuator lag.  There is deliberately no rate constraint: planar3 does
not define one.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from time import perf_counter
import math

import numpy as np
import osqp
from scipy import sparse


# The old 0.02 delivered-thrust state weight made a tens-of-newtons command
# dominate the position objective.  Keep a small regularizer, but put it on
# the same scale as the task's progress reward.
DELIVERED_THRUST_EFFORT_WEIGHT = 2e-5


@dataclass
class MPCSolve:
    command: np.ndarray
    solve_time_s: float
    fallback: bool
    status: str


@dataclass
class LinearPlanarMPC:
    prediction_horizon: int
    position_effort_ratio: float
    dt_s: float = .05
    control_horizon: int = 1
    command_limit_n: float = 70.0
    actuator_tau_s: float = .35
    previous_command: np.ndarray = field(default_factory=lambda: np.zeros(2))
    delivered_thrust: np.ndarray = field(default_factory=lambda: np.zeros(2))
    previous_solution: np.ndarray = field(default_factory=lambda: np.zeros(2))

    def reset(self) -> None:
        self.previous_command[:] = 0
        self.delivered_thrust[:] = 0
        self.previous_solution = np.zeros(2 * self.control_horizon)

    def _step(self, state: np.ndarray, command: np.ndarray) -> np.ndarray:
        """Euler-discrete planar3 local model, including twin-thruster lag."""
        n, e, psi, u, v, r, port, starboard = state
        alpha = math.exp(-self.dt_s / self.actuator_tau_s)
        next_port = alpha * port + (1 - alpha) * command[0]
        next_starboard = alpha * starboard + (1 - alpha) * command[1]
        surge = next_port + next_starboard
        yaw_moment = .395 * (next_starboard - next_port)
        # Vehicle-A Otter pinned mass/inertia and planar damping coefficients.
        u_dot = (surge - 24 * u - 2 * abs(u) * u) / 55 + r * v
        v_dot = (-33 * v - 8 * abs(v) * v) / 55 - r * u
        r_dot = (yaw_moment - 2 * r - abs(r) * r) / 30
        return np.array([
            n + self.dt_s * (math.cos(psi) * u - math.sin(psi) * v),
            e + self.dt_s * (math.sin(psi) * u + math.cos(psi) * v),
            psi + self.dt_s * r,
            u + self.dt_s * u_dot,
            v + self.dt_s * v_dot,
            r + self.dt_s * r_dot,
            next_port,
            next_starboard,
        ], dtype=np.float64)

    def _linearize(self, state: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        eps_x, eps_u = 1e-5, 1e-4
        base = self._step(state, self.previous_command)
        a = np.empty((8, 8), dtype=np.float64)
        b = np.empty((8, 2), dtype=np.float64)
        for i in range(8):
            delta = np.zeros(8); delta[i] = eps_x
            a[:, i] = (self._step(state + delta, self.previous_command) - base) / eps_x
        for i in range(2):
            delta = np.zeros(2); delta[i] = eps_u
            b[:, i] = (self._step(state, self.previous_command + delta) - base) / eps_u
        # x(k+1) = A*x(k) + B*u(k) + c; retain the affine residual.
        c = base - a @ state - b @ self.previous_command
        return a, b, c

    def _reference_state(
        self,
        reference_trajectory: list[tuple[float, float, float, float]],
        step: int,
    ) -> np.ndarray:
        """Build the nominal state used to linearize one prediction step.

        Reference samples provide planar pose and surge speed.  Sway is zero
        for the path reference; yaw rate is the wrapped finite difference of
        adjacent headings.  Delivered thrust remains the measured state because
        the reference does not prescribe a thrust trajectory.
        """
        index = min(step, len(reference_trajectory) - 1)
        n, e, psi, u = reference_trajectory[index]
        if index + 1 < len(reference_trajectory):
            next_psi = reference_trajectory[index + 1][2]
            r = math.atan2(math.sin(next_psi - psi), math.cos(next_psi - psi)) / self.dt_s
        elif index:
            previous_psi = reference_trajectory[index - 1][2]
            r = math.atan2(math.sin(psi - previous_psi), math.cos(psi - previous_psi)) / self.dt_s
        else:
            r = 0.
        return np.array([n, e, psi, u, 0., r, *self.delivered_thrust], dtype=np.float64)

    def solve(self, truth: dict[str, object], reference_trajectory: list[tuple[float, float, float, float]], delivered_thrust: np.ndarray | None = None) -> MPCSolve:
        started = perf_counter()
        n, e = truth["position_ned_m"][:2]  # type: ignore[index]
        psi = truth["attitude_rad"][2]  # type: ignore[index]
        u, v = truth["velocity_body_mps"][:2]  # type: ignore[index]
        r = truth["angular_rate_body_rad_s"][2]  # type: ignore[index]
        if delivered_thrust is not None:
            self.delivered_thrust[:] = delivered_thrust
        state = np.array([n, e, psi, u, v, r, *self.delivered_thrust], dtype=np.float64)
        # Piecewise-constant move blocking: the first Nc moves are independently
        # optimized and the final move is held for the remaining prediction steps.
        nc = self.control_horizon
        if self.previous_solution.shape != (2 * nc,):
            self.previous_solution = np.zeros(2 * nc)
        offset = state.copy()
        bsum = np.zeros((8, 2 * nc))
        qpos = self.position_effort_ratio
        q = np.diag([qpos, qpos, .5, 1., .25, .25,
                     DELIVERED_THRUST_EFFORT_WEIGHT, DELIVERED_THRUST_EFFORT_WEIGHT])
        # The task's -0.05 delta-action penalty maps to this soft command-delta cost.
        hessian = np.zeros((2 * nc, 2 * nc))
        linear = np.zeros(2 * nc)
        for step in range(self.prediction_horizon):
            # This is an LTV prediction: each transition is linearized at the
            # corresponding reference point, rather than freezing A/B/c at the
            # measured state for the entire horizon.
            a, b, c = self._linearize(self._reference_state(reference_trajectory, step))
            bsum = a @ bsum
            move = min(step, nc - 1)
            bsum[:, 2 * move:2 * move + 2] += b
            offset = a @ offset + c
            target = np.array(reference_trajectory[min(step, len(reference_trajectory) - 1)], dtype=np.float64)
            xref = np.array([target[0], target[1], target[2], target[3], 0., 0., 0., 0.])
            hessian += bsum.T @ q @ bsum
            linear += bsum.T @ q @ (offset - xref)
        # Soft command-delta penalty: first move relative to last command, then
        # adjacent moves. It does not create a hard rate constraint.
        delta = np.zeros((2 * nc, 2 * nc)); delta[:2, :2] = np.eye(2)
        for move in range(1, nc):
            delta[2 * move:2 * move + 2, 2 * move:2 * move + 2] = np.eye(2)
            delta[2 * move:2 * move + 2, 2 * (move - 1):2 * move] = -np.eye(2)
        r_delta = .02 * np.eye(2 * nc)
        prior = np.zeros(2 * nc); prior[:2] = self.previous_command
        hessian += delta.T @ r_delta @ delta
        linear += -delta.T @ r_delta @ prior
        p = sparse.csc_matrix((hessian + hessian.T) + 1e-8 * np.eye(2 * nc))
        solver = osqp.OSQP()
        solver.setup(P=p, q=2 * linear, A=sparse.eye(2 * nc, format="csc"),
                     l=np.full(2 * nc, -self.command_limit_n), u=np.full(2 * nc, self.command_limit_n),
                     verbose=False, polishing=False, warm_starting=False, eps_abs=1e-5, eps_rel=1e-5)
        result = solver.solve()
        ok = result.info.status_val in (1, 2) and result.x is not None and np.all(np.isfinite(result.x))
        solution = np.clip(np.asarray(result.x if ok else self.previous_solution, dtype=np.float64), -self.command_limit_n, self.command_limit_n)
        self.previous_solution[:] = solution
        command = solution[:2]
        self.previous_command[:] = command
        return MPCSolve(command=command, solve_time_s=perf_counter() - started, fallback=not ok, status=str(result.info.status))
