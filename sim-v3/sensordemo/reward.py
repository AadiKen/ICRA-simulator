from __future__ import annotations

from dataclasses import dataclass

from .config import SensorDemoConfig


@dataclass(frozen=True)
class RewardTerms:
    progress: float
    collision: float
    proximity: float
    jitter: float
    effort: float
    goal: float

    @property
    def total(self) -> float: return self.progress-self.collision-self.proximity-self.jitter-self.effort+self.goal


def compute_reward(config: SensorDemoConfig, *, previous_goal_distance_m: float, goal_distance_m: float, collision: bool, true_cpa_m: float, bearing_action: float, previous_bearing_action: float, integrated_thrust_ns: float, thrust_normalizer_ns: float, reached_goal: bool) -> RewardTerms:
    proximity = max(0.0, min(1.0, (config.safe_radius_m-true_cpa_m)/config.safe_radius_m))
    return RewardTerms(config.w_progress*(previous_goal_distance_m-goal_distance_m)/config.route_length_m, config.w_collision*float(collision), config.w_proximity*proximity, config.w_jitter*abs(bearing_action-previous_bearing_action)/2.0, config.w_effort*integrated_thrust_ns/max(thrust_normalizer_ns, 1e-9), config.w_goal*float(reached_goal))
