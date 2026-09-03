from __future__ import annotations
import json,subprocess,threading
from pathlib import Path
from typing import Any
from concurrent.futures import ThreadPoolExecutor

class PersistentNodeBridge:
    """One persistent Node process owning an authoritative deterministic vector batch."""
    def __init__(self, repository: str | Path, node: str="node") -> None:
        self.repository=Path(repository).resolve();self._next_id=0;self._lock=threading.Lock()
        self._process=subprocess.Popen([node,"--experimental-strip-types","backends/node/src/persistent-batch-service.ts"],cwd=self.repository,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,bufsize=1)
    def _request(self,op:str,**payload:Any)->Any:
        with self._lock:
            if self._process.poll() is not None: raise RuntimeError(f"Node bridge exited: {self._process.stderr.read()}")
            self._next_id+=1;request={"id":str(self._next_id),"op":op,**payload};assert self._process.stdin and self._process.stdout
            self._process.stdin.write(json.dumps(request,separators=(",",":"))+"\n");self._process.stdin.flush();line=self._process.stdout.readline()
            if not line: raise RuntimeError(f"Node bridge closed output: {self._process.stderr.read()}")
            response=json.loads(line)
            if response.get("id")!=request["id"]: raise RuntimeError("Node bridge response ID mismatch")
            if not response.get("ok"): raise RuntimeError(response.get("error","Node bridge request failed"))
            return response["result"]
    def reset(self,configs:list[dict[str,Any]],mask:list[bool]|None=None)->Any:return self._request("reset",configs=configs,**({"mask":mask}if mask is not None else {}))
    def reset_slots(self,configs:list[dict[str,Any]],mask:list[bool])->Any:return self._request("reset_slots",configs=configs,mask=mask)
    def step(self,actions:list[Any],mask:list[bool]|None=None)->Any:return self._request("step",actions=actions,**({"mask":mask}if mask is not None else {}))
    def checkpoint(self)->dict[str,Any]:return self._request("checkpoint")
    def restore(self,checkpoint:dict[str,Any])->None:self._request("restore",checkpoint=checkpoint)
    def ground_truth(self,index:int=0)->Any:return self._request("ground_truth",index=index)
    def ground_truth_all(self)->list[Any]:return self._request("ground_truth_all")
    def metrics(self,index:int=0)->Any:return self._request("metrics",index=index)
    def close(self)->None:
        if self._process.poll() is None:
            try:self._request("dispose")
            finally:
                if self._process.stdin:self._process.stdin.close()
                self._process.wait(timeout=5)
                if self._process.stdout:self._process.stdout.close()
                if self._process.stderr:self._process.stderr.close()
    def __enter__(self):return self
    def __exit__(self,*_:Any):self.close()

class ShardedNodeBridge:
    """Persistent multi-process bridge; each shard owns a contiguous deterministic slot range."""
    def __init__(self,repository:str|Path,workers:int=1,node:str="node") -> None:
        if workers<1:raise ValueError("workers must be positive")
        self.bridges=[PersistentNodeBridge(repository,node) for _ in range(workers)];self.slices:list[tuple[int,int]]=[]
    def _parallel(self,fn):
        with ThreadPoolExecutor(max_workers=len(self.bridges)) as pool:return list(pool.map(fn,range(len(self.bridges))))
    def reset(self,configs:list[dict[str,Any]])->list[Any]:
        n=len(configs);base,extra=divmod(n,len(self.bridges));start=0;self.slices=[]
        for i in range(len(self.bridges)):
            end=start+base+(1 if i<extra else 0);self.slices.append((start,end));start=end
        parts=self._parallel(lambda i:self.bridges[i].reset(configs[self.slices[i][0]:self.slices[i][1]])["observations"] if self.slices[i][0]<self.slices[i][1] else [])
        return [item for part in parts for item in part]
    def step(self,actions:list[Any])->dict[str,Any]:
        if not self.slices:raise RuntimeError("reset required")
        parts=self._parallel(lambda i:self.bridges[i].step(actions[self.slices[i][0]:self.slices[i][1]]) if self.slices[i][0]<self.slices[i][1] else {"observations":[],"rewards":[],"terminated":[],"truncated":[],"infos":[]})
        return {key:[item for part in parts for item in part[key]] for key in ("observations","rewards","terminated","truncated","infos")}
    def reset_slots(self,configs:list[dict[str,Any]],mask:list[bool])->list[Any]:
        if len(configs)!=len(mask) or not self.slices:raise ValueError("reset_slots requires full config and mask vectors")
        parts=self._parallel(lambda i:self.bridges[i].reset_slots(configs[self.slices[i][0]:self.slices[i][1]],mask[self.slices[i][0]:self.slices[i][1]])["observations"] if self.slices[i][0]<self.slices[i][1] else [])
        return [item for part in parts for item in part]
    def checkpoints(self)->list[dict[str,Any]]:return self._parallel(lambda i:self.bridges[i].checkpoint())
    def ground_truth_all(self)->list[Any]:return [item for part in self._parallel(lambda i:self.bridges[i].ground_truth_all()) for item in part]
    def restore(self,checkpoints:list[dict[str,Any]])->None:
        if len(checkpoints)!=len(self.bridges):raise ValueError("checkpoint shard count mismatch")
        self._parallel(lambda i:self.bridges[i].restore(checkpoints[i]))
    def close(self)->None:
        for bridge in self.bridges:bridge.close()
    def __enter__(self):return self
    def __exit__(self,*_:Any):self.close()
