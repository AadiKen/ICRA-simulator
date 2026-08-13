import type {Vec3} from "../runtime.ts";

export interface SkyOcclusionSample{clearFraction:number;blockedFraction:number;rayCount:number;modelVersion:string}
export interface SkyOcclusionService{sample(originNedM:Vec3):SkyOcclusionSample}
export const SKY_OCCLUSION_MODEL_VERSION="hemisphere-golden-angle-v1";
export const SKY_OCCLUSION_RAY_COUNT=32;
export const SKY_OCCLUSION_MAX_RANGE_M=20000;
export function deterministicSkyDirections(count=SKY_OCCLUSION_RAY_COUNT):Vec3[]{if(!Number.isInteger(count)||count<1)throw new Error("Sky ray count must be a positive integer.");const goldenAngle=Math.PI*(3-Math.sqrt(5));return Array.from({length:count},(_,index)=>{const up=(index+.5)/count,radius=Math.sqrt(Math.max(1-up*up,0)),azimuth=index*goldenAngle;return[Math.cos(azimuth)*radius,Math.sin(azimuth)*radius,-up];});}
export class RaycastSkyOcclusionService implements SkyOcclusionService{
  readonly #raycast:(origin:Vec3,direction:Vec3,maxRangeM:number)=>unknown;readonly #directions:Vec3[];
  constructor(raycast:(origin:Vec3,direction:Vec3,maxRangeM:number)=>unknown,rayCount=SKY_OCCLUSION_RAY_COUNT){this.#raycast=raycast;this.#directions=deterministicSkyDirections(rayCount);}
  sample(originNedM:Vec3):SkyOcclusionSample{let blocked=0;for(const direction of this.#directions)if(this.#raycast(originNedM,direction,SKY_OCCLUSION_MAX_RANGE_M))blocked++;const blockedFraction=blocked/this.#directions.length;return{clearFraction:1-blockedFraction,blockedFraction,rayCount:this.#directions.length,modelVersion:SKY_OCCLUSION_MODEL_VERSION};}
}
export class ClearSkyOcclusionService implements SkyOcclusionService{sample(_originNedM:Vec3):SkyOcclusionSample{return{clearFraction:1,blockedFraction:0,rayCount:SKY_OCCLUSION_RAY_COUNT,modelVersion:SKY_OCCLUSION_MODEL_VERSION};}}
