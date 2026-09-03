import json,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/"packages/python-client"));from bcod_sim import ShardedNodeBridge
def config(i):return{"schema_version":1,"experiment":{"name":f"shard-{i}","seed":i,"timestep_s":.05,"duration_s":20},"backend":{"type":"node"},"vehicle":{"preset":"vehicle-a-otter","plant":"planar3"},"mission":{"type":"hold"}}
batch=64;configs=[config(i) for i in range(batch)];actions=[{"portCommand":.3,"starboardCommand":.4} for _ in configs];rows=[]
for workers in (1,2,4,8):
 with ShardedNodeBridge(ROOT,workers) as bridge:
  bridge.reset(configs)
  for _ in range(10):bridge.step(actions)
  start=time.perf_counter()
  for _ in range(100):bridge.step(actions)
  elapsed=time.perf_counter()-start
 rows.append({"workers":workers,"batch":batch,"environment_steps":6400,"wall_clock_s":elapsed,"steps_per_s":6400/elapsed,"paper_measurement_eligible":False})
report={"schema_version":1,"host_class":"local","purpose":"architecture profiling only","rows":rows,"conclusion":"Persistent process sharding is the authoritative Node scaling path; select worker count empirically per host and never mix these local numbers with paper throughput."};out=ROOT/"artifacts/rl-campaign/sharded-bridge-profile.json";out.write_text(json.dumps(report,indent=2)+"\n");print(json.dumps(report,indent=2))
