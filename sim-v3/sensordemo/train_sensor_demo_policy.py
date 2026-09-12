from __future__ import annotations
import argparse
from pathlib import Path
import sys
import time


def _duration(seconds: float) -> str:
    if not (seconds >= 0 and seconds < float("inf")):
        return "--:--:--"
    value = int(seconds)
    hours, remainder = divmod(value, 3600)
    minutes, secs = divmod(remainder, 60)
    return f"{hours:02d}:{minutes:02d}:{secs:02d}"


def main() -> None:
    parser = argparse.ArgumentParser(description="Train the sensor-demo PPO and abl_blind variant")
    parser.add_argument("--backend-factory", default="sensordemo.local_backend:create_training_env", help="module:function returning a configured environment")
    parser.add_argument("--variant", choices=("sensor", "abl_blind"), default="sensor")
    parser.add_argument("--steps", type=int, default=50_000)
    parser.add_argument("--output", type=Path, default=Path("artifacts/sensor-demo/policy"))
    parser.add_argument("--checkpoint-every", type=int, default=10_000)
    parser.add_argument("--evaluate-every", type=int, default=10_000)
    parser.add_argument("--evaluation-episodes", type=int, default=5)
    parser.add_argument("--no-progress", action="store_true", help="disable the terminal ETA progress bar")
    args = parser.parse_args()
    module_name, function_name = args.backend_factory.split(":", 1)
    import importlib
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback
    from stable_baselines3.common.callbacks import CallbackList
    from stable_baselines3.common.logger import configure
    from stable_baselines3.common.vec_env import DummyVecEnv
    from .obs_contract import SENSOR_DEMO_OBS_CONTRACT_HASH
    from .training_logging import ResearchLogger

    class EtaProgressBar(BaseCallback):
        """Dependency-free progress bar suitable for local and cluster logs."""

        def __init__(self, total_steps: int, label: str, enabled: bool = True) -> None:
            super().__init__()
            self.total_steps, self.label, self.enabled = total_steps, label, enabled
            self.started = self.last_render = 0.0
            self.last_percent = -1

        def _on_training_start(self) -> None:
            self.started = self.last_render = time.monotonic()
            self._render(force=True)

        def _on_step(self) -> bool:
            now = time.monotonic()
            percent = min(100, int(100*self.num_timesteps/max(1, self.total_steps)))
            if percent != self.last_percent or now-self.last_render >= 2.0:
                self._render()
            return True

        def _on_training_end(self) -> None:
            self._render(force=True, complete=True)
            if self.enabled: print(file=sys.stderr, flush=True)

        def _render(self, force: bool = False, complete: bool = False) -> None:
            if not self.enabled: return
            now = time.monotonic(); elapsed = max(0., now-self.started)
            completed = self.total_steps if complete else min(self.num_timesteps, self.total_steps)
            ratio = completed/max(1, self.total_steps); percent = int(100*ratio)
            if not force and percent == self.last_percent and now-self.last_render < 2.0: return
            rate = completed/elapsed if elapsed > 0 else 0.; eta = (self.total_steps-completed)/rate if rate > 0 else float("inf")
            width = 30; filled = min(width, int(width*ratio)); bar = "█"*filled+"░"*(width-filled)
            print(f"\r{self.label:<12} [{bar}] {percent:3d}%  {completed:,}/{self.total_steps:,}  elapsed {_duration(elapsed)}  ETA {_duration(eta)}  {rate:,.1f} steps/s", end="", file=sys.stderr, flush=True)
            self.last_render, self.last_percent = now, percent

    factory = getattr(importlib.import_module(module_name), function_name)
    run_dir = args.output.parent/f"{args.output.name}-logs"
    run_dir.mkdir(parents=True, exist_ok=True)
    env = DummyVecEnv([lambda: factory(blind=args.variant == "abl_blind")])
    model = PPO("MlpPolicy", env, seed=7319, n_steps=512, batch_size=512, n_epochs=10, learning_rate=3e-4, gamma=.99, gae_lambda=.95, clip_range=.2, policy_kwargs={"net_arch": [128, 128]}, verbose=1)
    model.set_logger(configure(str(run_dir), ["stdout", "csv", "json"]))
    progress = EtaProgressBar(args.steps, args.variant, enabled=not args.no_progress)
    research = ResearchLogger(run_dir=run_dir, variant=args.variant, factory=factory, checkpoint_every=args.checkpoint_every, evaluate_every=args.evaluate_every, evaluation_episodes=args.evaluation_episodes)
    metadata = {"schema_version": 1, "variant": args.variant, "seed": 7319, "total_timesteps": args.steps, "backend_factory": args.backend_factory, "output": str(args.output), "observation_contract_sha256": SENSOR_DEMO_OBS_CONTRACT_HASH, "checkpoint_every": args.checkpoint_every, "evaluate_every": args.evaluate_every, "evaluation_episodes": args.evaluation_episodes, "ppo": {"n_steps": 512, "batch_size": 512, "n_epochs": 10, "learning_rate": 3e-4, "gamma": .99, "gae_lambda": .95, "clip_range": .2, "network": [128, 128]}}
    (run_dir/"run-metadata.json").write_text(__import__("json").dumps(metadata, indent=2)+"\n")
    try:
        model.learn(total_timesteps=args.steps, callback=CallbackList([progress, research]), progress_bar=False)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        model.save(args.output)
    finally:
        env.close()


if __name__ == "__main__": main()
