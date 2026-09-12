from __future__ import annotations

from collections import deque
import csv
import json
import math
from pathlib import Path
import statistics
import time
from typing import Callable

from stable_baselines3.common.callbacks import BaseCallback


EPISODE_FIELDS = ["wall_time_s", "timesteps", "episode", "variant", "seed", "return", "length_steps", "elapsed_s", "success", "collision", "outside_route_bounds", "termination_reason", "final_goal_distance_m", "minimum_true_cpa_m", "final_track_count", "reward_progress", "reward_collision", "reward_proximity", "reward_jitter", "reward_effort", "reward_goal"]
EVALUATION_FIELDS = ["wall_time_s", "timesteps", "evaluation", "variant", "episodes", "mean_return", "median_return", "return_std", "success_rate", "collision_rate", "outside_rate", "mean_final_goal_distance_m", "mean_minimum_true_cpa_m", "mean_length_steps"]


class ResearchLogger(BaseCallback):
    """Figure-ready episode/evaluation logs plus checkpoints and best models."""

    def __init__(self, *, run_dir: Path, variant: str, factory: Callable, checkpoint_every: int, evaluate_every: int, evaluation_episodes: int) -> None:
        super().__init__(); self.run_dir, self.variant, self.factory = run_dir, variant, factory
        self.checkpoint_every, self.evaluate_every, self.evaluation_episodes = checkpoint_every, evaluate_every, evaluation_episodes
        self.started = time.time(); self.episode_index = self.evaluation_index = 0; self.next_checkpoint = checkpoint_every; self.next_evaluation = evaluate_every; self.best_score = (-1., -math.inf)
        self.recent_success, self.recent_collision, self.recent_return = deque(maxlen=100), deque(maxlen=100), deque(maxlen=100)

    def _on_training_start(self) -> None:
        self.run_dir.mkdir(parents=True, exist_ok=True); (self.run_dir/"checkpoints").mkdir(exist_ok=True)
        self._initialize_csv("episodes.csv", EPISODE_FIELDS); self._initialize_csv("evaluations.csv", EVALUATION_FIELDS)
        self._write_json("status.json", {"status": "running", "variant": self.variant, "started_unix_s": self.started, "timesteps": 0})

    def _initialize_csv(self, name: str, fields: list[str]) -> None:
        path = self.run_dir/name
        if not path.exists():
            with path.open("w", newline="") as stream: csv.DictWriter(stream, fieldnames=fields).writeheader()

    def _append(self, name: str, fields: list[str], row: dict) -> None:
        with (self.run_dir/name).open("a", newline="") as stream: csv.DictWriter(stream, fieldnames=fields).writerow(row)

    def _write_json(self, name: str, value: dict) -> None:
        path = self.run_dir/name; temporary = path.with_suffix(path.suffix+".tmp"); temporary.write_text(json.dumps(value, indent=2)+"\n"); temporary.replace(path)

    def _on_step(self) -> bool:
        for done, info in zip(self.locals.get("dones", []), self.locals.get("infos", [])):
            if done: self._record_episode(info)
        if self.num_timesteps >= self.next_checkpoint:
            self.model.save(self.run_dir/"checkpoints"/f"model-{self.num_timesteps}"); self.next_checkpoint += self.checkpoint_every
        if self.num_timesteps >= self.next_evaluation:
            self._evaluate(); self.next_evaluation += self.evaluate_every
        return True

    def _record_episode(self, info: dict) -> None:
        self.episode_index += 1; parts = info.get("episode_reward_components", {})
        row = {"wall_time_s": time.time()-self.started, "timesteps": self.num_timesteps, "episode": self.episode_index, "variant": self.variant, "seed": info.get("terminal_seed", ""), "return": info.get("episode_return"), "length_steps": info.get("episode_steps"), "elapsed_s": info.get("elapsed_s"), "success": int(bool(info.get("success"))), "collision": int(bool(info.get("collision"))), "outside_route_bounds": int(bool(info.get("outside_route_bounds"))), "termination_reason": info.get("termination_reason"), "final_goal_distance_m": info.get("goal_distance_m"), "minimum_true_cpa_m": info.get("minimum_true_cpa_m"), "final_track_count": info.get("track_count"), **{f"reward_{name}": parts.get(name) for name in ("progress", "collision", "proximity", "jitter", "effort", "goal")}}
        self._append("episodes.csv", EPISODE_FIELDS, row); self.recent_success.append(row["success"]); self.recent_collision.append(row["collision"]); self.recent_return.append(float(row["return"] or 0.))
        for key, values in (("success_rate_100", self.recent_success), ("collision_rate_100", self.recent_collision), ("mean_return_100", self.recent_return)):
            if values: self.logger.record(f"rollout/{key}", statistics.fmean(values))

    def _evaluate(self) -> None:
        self.evaluation_index += 1; rows = []; env = self.factory(blind=self.variant == "abl_blind")
        try:
            for index in range(self.evaluation_episodes):
                obs, _ = env.reset(seed=900_000+self.evaluation_index*10_000+index); total = 0.
                for length in range(math.ceil(env.config.max_episode_time_s/env.config.waypoint_interval_s)):
                    action, _ = self.model.predict(obs, deterministic=True); obs, reward, terminated, truncated, info = env.step(action); total += float(reward)
                    if terminated or truncated: break
                rows.append({"return": total, "success": bool(info["success"]), "collision": bool(info["collision"]), "outside": bool(info["outside_route_bounds"]), "distance": float(info["goal_distance_m"]), "cpa": float(info["minimum_true_cpa_m"]), "length": length+1})
        finally: env.close()
        finite_cpa = [row["cpa"] for row in rows if math.isfinite(row["cpa"])]
        summary = {"wall_time_s": time.time()-self.started, "timesteps": self.num_timesteps, "evaluation": self.evaluation_index, "variant": self.variant, "episodes": len(rows), "mean_return": statistics.fmean(row["return"] for row in rows), "median_return": statistics.median(row["return"] for row in rows), "return_std": statistics.pstdev(row["return"] for row in rows), "success_rate": statistics.fmean(row["success"] for row in rows), "collision_rate": statistics.fmean(row["collision"] for row in rows), "outside_rate": statistics.fmean(row["outside"] for row in rows), "mean_final_goal_distance_m": statistics.fmean(row["distance"] for row in rows), "mean_minimum_true_cpa_m": statistics.fmean(finite_cpa) if finite_cpa else "", "mean_length_steps": statistics.fmean(row["length"] for row in rows)}
        self._append("evaluations.csv", EVALUATION_FIELDS, summary); (self.run_dir/f"evaluation-{self.evaluation_index:04d}.json").write_text(json.dumps({"summary": summary, "episodes": rows}, indent=2)+"\n")
        for key in EVALUATION_FIELDS[5:]:
            if isinstance(summary.get(key), (int, float)): self.logger.record(f"eval/{key}", summary[key])
        score = (summary["success_rate"], summary["mean_return"])
        if score > self.best_score: self.best_score = score; self.model.save(self.run_dir/"best-model")
        self._write_json("status.json", {"status": "running", "variant": self.variant, "timesteps": self.num_timesteps, "episodes": self.episode_index, "latest_evaluation": summary, "best_score": self.best_score})

    def _on_training_end(self) -> None:
        self._write_json("status.json", {"status": "training_complete", "variant": self.variant, "timesteps": self.num_timesteps, "episodes": self.episode_index, "evaluations": self.evaluation_index, "best_score": self.best_score, "wall_time_s": time.time()-self.started})
