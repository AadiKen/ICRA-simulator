from __future__ import annotations
import argparse,hashlib,json
from pathlib import Path
from stable_baselines3 import PPO
ARMS=("bcod-sim","gazebo","holoocean","stonefish")
def main():
 p=argparse.ArgumentParser();p.add_argument("--config",type=Path,required=True,help="JSON with target_steps and per-arm checkpoint/manifest paths");p.add_argument("--output",type=Path,required=True);a=p.parse_args();cfg=json.loads(a.config.read_text());report={"schema_version":1,"artifact_kind":"checkpoint-step-inventory","target_steps":int(cfg["target_steps"])}
 for arm in ARMS:
  item=cfg[arm];checkpoint=Path(item["checkpoint"]);manifest=json.loads(Path(item["manifest"]).read_text());model=PPO.load(checkpoint,device="cpu");logged=int(manifest["actual_timesteps"])
  if int(model.num_timesteps)!=logged:raise RuntimeError(f"{arm}: checkpoint steps {model.num_timesteps} != manifest steps {logged}")
  report[arm]={"steps":logged,"checkpoint_save_interval":int(manifest["checkpoint_frequency_timesteps"]),"checkpoint_sha256":hashlib.sha256(checkpoint.read_bytes()).hexdigest(),"checkpoint":str(checkpoint),"manifest":item["manifest"]}
 a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+"\n");print(json.dumps(report,indent=2))
if __name__=="__main__":main()
