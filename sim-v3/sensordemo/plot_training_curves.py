from __future__ import annotations
import argparse
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot sensor-demo PPO loss and held-out success curves")
    parser.add_argument("--sensor", type=Path, default=Path("artifacts/sensor-demo/policy-logs"))
    parser.add_argument("--blind", type=Path, default=Path("artifacts/sensor-demo/abl-blind-logs"))
    parser.add_argument("--output", type=Path, default=Path("artifacts/sensor-demo/training-curves.png"))
    args = parser.parse_args(); figure, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    found = False
    for label, directory, color in (("Sensor-aware", args.sensor, "#1677b8"), ("Blind ablation", args.blind, "#d95f02")):
        progress, evaluations = directory/"progress.csv", directory/"evaluations.csv"
        if progress.exists():
            data = pd.read_csv(progress); step = data.get("time/total_timesteps")
            loss = data.get("train/loss")
            if step is not None and loss is not None: axes[0].plot(step, loss, label=label, color=color, alpha=.9); found = True
        if evaluations.exists():
            data = pd.read_csv(evaluations)
            if len(data): axes[1].plot(data["timesteps"], data["success_rate"], marker="o", label=label, color=color); found = True
    if not found: raise SystemExit("No training logs found. Complete at least one logged update/evaluation first.")
    axes[0].set(title="PPO training loss", xlabel="Environment steps", ylabel="Loss"); axes[1].set(title="Held-out navigation success", xlabel="Environment steps", ylabel="Success rate", ylim=(-.03, 1.03))
    for axis in axes: axis.grid(alpha=.25); axis.legend(frameon=False)
    figure.tight_layout(); args.output.parent.mkdir(parents=True, exist_ok=True); figure.savefig(args.output, dpi=220); print(args.output)


if __name__ == "__main__": main()
