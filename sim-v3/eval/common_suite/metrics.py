from __future__ import annotations
from collections import defaultdict
from dataclasses import asdict,dataclass
from math import sqrt
from typing import Any,Iterable
import numpy as np
@dataclass(frozen=True)
class EpisodeMetrics:
 arm:str;condition:str;contract_sha256:str;seed:int;success:bool;cross_track_error_m:float;completion_time_s:float;propulsion_cost_ns:float;safety_violations:int;training_steps:int;training_wall_clock_s:float;domain_gap:float
 def to_dict(self)->dict[str,Any]:return asdict(self)
def wilson_interval(successes:int,count:int,z:float=1.959963984540054)->tuple[float,float]:
 if count<=0:raise ValueError("Wilson interval requires at least one episode")
 p=successes/count;d=1+z*z/count;c=(p+z*z/(2*count))/d;m=z*sqrt((p*(1-p)+z*z/(4*count))/count)/d;return c-m,c+m
def bootstrap_mean_ci(values:Iterable[float],*,seed:int=7319,samples:int=10_000)->tuple[float,float,float]:
 data=np.asarray(list(values),float)
 if not len(data) or not np.isfinite(data).all():raise ValueError("bootstrap values must be non-empty and finite")
 rng=np.random.default_rng(seed);means=rng.choice(data,size=(samples,len(data)),replace=True).mean(axis=1);lo,hi=np.quantile(means,[.025,.975]);return float(data.mean()),float(lo),float(hi)
def aggregate(rows:Iterable[EpisodeMetrics],*,bootstrap_samples:int=10_000)->list[dict[str,Any]]:
 groups=defaultdict(list)
 for row in rows:groups[(row.arm,row.condition,row.contract_sha256,row.domain_gap)].append(row)
 output=[]
 for key,group in sorted(groups.items()):
  successes=sum(row.success for row in group);cell=dict(zip(("arm","condition","contract_sha256","domain_gap"),key,strict=True));cell.update(episode_count=len(group),success_rate=successes/len(group),success_ci95=list(wilson_interval(successes,len(group))))
  for offset,field in enumerate(("cross_track_error_m","completion_time_s","propulsion_cost_ns","safety_violations")):
   mean,lo,hi=bootstrap_mean_ci((getattr(row,field) for row in group),seed=7319+offset,samples=bootstrap_samples);cell[field]={"mean":mean,"ci95":[lo,hi]}
  output.append(cell)
 return output
