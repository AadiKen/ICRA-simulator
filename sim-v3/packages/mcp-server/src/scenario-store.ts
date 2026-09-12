import {mkdtemp} from "node:fs/promises";
import {tmpdir} from "node:os";
import {join} from "node:path";
import type {ResolvedExperimentV1} from "../../experiment-schema/src/index.ts";
import type {RunManifestV1,RunSummary} from "../../metrics/src/index.ts";

export interface StoredRun {manifest:RunManifestV1;metrics:RunSummary["metrics"];directory:string}
export class ScenarioStore {
  readonly scenarios=new Map<string,ResolvedExperimentV1>();
  readonly runs=new Map<string,StoredRun>();
  #root?:string;
  async artifactRoot():Promise<string>{return this.#root??=await mkdtemp(join(tmpdir(),"bcod-mcp-"));}
}

