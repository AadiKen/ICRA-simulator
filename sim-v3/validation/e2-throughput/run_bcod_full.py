"""Publication-grade fixed-horizon E2 sweep for the Vehicle A planar3 tensor backend."""
from __future__ import annotations

import argparse, csv, json, os, platform, subprocess, sys, time
from pathlib import Path
import torch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "backends/tensor"))
from bcod_tensor import VehicleAPlanar3Config, VehicleAPlanar3TensorBackend


def sync(device: str) -> None:
    if device == "cuda":
        torch.cuda.synchronize()


def memory(device: str) -> int:
    if device == "cuda":
        return int(torch.cuda.max_memory_allocated())
    return 0


def cell(device: str, batch: int, steps: int, warmups: int, repeats: int) -> dict:
    numerics = "float32" if device == "cuda" else "float64"
    backend = VehicleAPlanar3TensorBackend(VehicleAPlanar3Config(
        environments=batch, device=device, numerics_mode=numerics))
    action = torch.tensor([[.4, .3]], dtype=backend.dtype, device=backend.device).repeat(batch, 1)
    for _ in range(warmups):
        backend.reset()
        for _ in range(steps): backend.step(action)
    if device == "cuda": torch.cuda.reset_peak_memory_stats()
    samples = []
    for _ in range(repeats):
        backend.reset(); sync(device); started = time.perf_counter()
        for _ in range(steps): backend.step(action)
        sync(device); samples.append(time.perf_counter() - started)
    total = sum(samples); aggregate = batch * steps * repeats
    return {"device": device, "batch_size": batch, "parallelism_kind": "tensor_batch",
            "rollout_steps": steps, "warmup_rollouts": warmups, "timed_rollouts": repeats,
            "timed_wall_s": total, "steps_per_second": aggregate / total,
            "ms_per_step": 1000 * total / (steps * repeats),
            "ms_per_environment_step": 1000 * total / aggregate,
            "memory_used_bytes": memory(device), "rollout_wall_s": samples,
            "numerics_mode": numerics}


def command(*args: str) -> str:
    return subprocess.run(args, text=True, capture_output=True, check=False).stdout.strip()


def main() -> None:
    p = argparse.ArgumentParser(); p.add_argument("--output", type=Path, required=True)
    p.add_argument("--batch-sizes", type=int, nargs="+", default=[1,2,4,8,16,32,64,128,256,512,1024,2048,4096,8192])
    p.add_argument("--rollout-steps", type=int, default=300); p.add_argument("--warmups", type=int, default=3); p.add_argument("--repeats", type=int, default=10)
    a = p.parse_args(); devices = ["cpu"] + (["cuda"] if torch.cuda.is_available() else [])
    scalar = json.loads(subprocess.check_output(["node", str(Path(__file__).with_name("scalar_probe.mjs")), str(a.rollout_steps*a.repeats), str(a.rollout_steps*a.warmups)], cwd=ROOT, text=True))
    rows = []
    for device in devices:
        previous = None
        for batch in a.batch_sizes:
            try: row = cell(device, batch, a.rollout_steps, a.warmups, a.repeats)
            except torch.cuda.OutOfMemoryError:
                rows.append({"device":device,"batch_size":batch,"stop_reason":"out_of_memory"}); break
            row["speedup_over_scalar"] = row["steps_per_second"] / scalar["steps_per_second"]
            row["growth_fraction"] = None if previous is None else row["steps_per_second"] / previous - 1
            rows.append(row); print(json.dumps(row), flush=True)
            if previous is not None and row["growth_fraction"] < .05:
                row["stop_reason"] = "throughput_growth_below_5_percent"; break
            previous = row["steps_per_second"]
    hardware = {"hostname":platform.node(),"cpu_model":command("bash","-lc","lscpu | sed -n 's/^Model name:[[:space:]]*//p' | head -1"),
                "ram_bytes":os.sysconf("SC_PAGE_SIZE")*os.sysconf("SC_PHYS_PAGES"),"gpu_model":command("nvidia-smi","--query-gpu=name","--format=csv,noheader"),
                "slurm_job_id":os.environ.get("SLURM_JOB_ID"),"slurm_nodelist":os.environ.get("SLURM_JOB_NODELIST")}
    report={"schema_version":1,"artifact_kind":"e2-real-throughput-sweep","status":"COMPLETE","simulator":"bcod-sim",
            "scope":{"vehicle":"vehicle-a-otter","plant":"planar3","coupled6_supported":False,"fixed_horizon":True},
            "protocol":{"rollout_steps":a.rollout_steps,"warmup_rollouts":a.warmups,"timed_rollouts":a.repeats,"early_success_termination":False},
            "hardware":hardware,"scalar_reference":scalar,"rows":rows,
            "retraction_root_cause":"NOT_FOUND; this measurement does not resolve the provenance failure in the retracted report."}
    a.output.parent.mkdir(parents=True,exist_ok=True); a.output.write_text(json.dumps(report,indent=2)+"\n")
    valid=[r for r in rows if "steps_per_second" in r]
    with a.output.with_suffix(".csv").open("w",newline="") as f:
        fields=["device","batch_size","steps_per_second","ms_per_step","ms_per_environment_step","memory_used_bytes","speedup_over_scalar","growth_fraction","stop_reason"]
        w=csv.DictWriter(f,fieldnames=fields,extrasaction="ignore");w.writeheader();w.writerows(valid)

if __name__ == "__main__": main()
