import type {GrantedServices,JSONValue,SensorDomain,Vec3} from "../runtime.ts";

export interface UnitConventions{distance:string;frequency:string;pressure:string;level:string;temperature:string;salinity:string}
export interface NoiseProfile{measurementStd:number;biasStd:number;driftStdPerSqrtS:number;dropoutProbability:number}
export interface DomainModule<P,E,Primitives>{readonly domain:SensorDomain;readonly version:string;readonly units:UnitConventions;queryEnvironment(services:GrantedServices,at:Vec3):E;readonly primitives:Primitives;defaultNoiseProfile(params:P):NoiseProfile}
export type DomainParams=Record<string,JSONValue>;
