export interface BathymetryQuery{position_ned_m:[number,number,number];time_s:number}
export interface BathymetrySample{water_depth_m:number|null;status?:"resolved"|"no_data";provenance?:Record<string,unknown>|null}
export interface BathymetryField{sample(query:BathymetryQuery):BathymetrySample;provenance():Record<string,unknown>}
/** Explicit test double. Production geography uses ResolvedGeography. */
export class ConstantBathymetryField implements BathymetryField{
  readonly #depth:number;readonly #source:Record<string,unknown>;
  constructor(waterDepthM:number,source:Record<string,unknown>={mode:"constant-bathymetry"}){if(!Number.isFinite(waterDepthM)||waterDepthM<=0)throw new Error("Bathymetry depth must be positive and finite.");this.#depth=waterDepthM;this.#source=structuredClone(source);}
  sample(_query:BathymetryQuery):BathymetrySample{return{water_depth_m:this.#depth};}
  provenance():Record<string,unknown>{return structuredClone(this.#source);}
}
export class UnresolvedBathymetryField implements BathymetryField{sample():BathymetrySample{return{water_depth_m:null,status:"no_data",provenance:{mode:"unresolved-no-geographic-region"}};}provenance():Record<string,unknown>{return{mode:"unresolved-no-geographic-region"};}}
