import type {AtmosphericSample} from "./atmosphere.ts";
import type {AquaticSample} from "./water.ts";
import type {SurfaceSample,RegularWaveSample} from "./surface.ts";
import type {ElectromagneticSample} from "./electromagnetic.ts";
import type {PositioningEnvironmentSample} from "./positioning.ts";
export * from "./atmosphere.ts";
export * from "./water.ts";
export * from "./surface.ts";
export * from "./electromagnetic.ts";
export * from "./positioning.ts";
export * from "./bathymetry.ts";
export * from "./geography.ts";
export * from "./geography-pipeline.ts";
export * from "./weather.ts";
export * from "./era5.ts";
export * from "./rtofs.ts";
export * from "./gebco.ts";
import {LocalGeographicFrame} from "./geography.ts";

export interface EnvironmentQuery {position_ned_m:[number,number,number];time_s:number}
/** Legacy flat fields remain required until existing consumers migrate. Grouped fields are populated by every package-provided field. */
export interface EnvironmentSample {wind_ned_mps:[number,number,number];current_ned_mps:[number,number,number];air_temperature_c:number;pressure_pa:number;humidity_fraction:number;rain_rate_mm_h:number;visibility_m:number;fog_extinction_per_m:number;illumination_lux:number;wave?:RegularWaveSample;tide_m?:number;atmosphere?:AtmosphericSample;water?:AquaticSample;surface?:SurfaceSample;electromagnetic?:ElectromagneticSample;positioning?:PositioningEnvironmentSample}
export interface GroupedEnvironmentSample extends EnvironmentSample{atmosphere:AtmosphericSample;water:AquaticSample;surface:SurfaceSample;electromagnetic:ElectromagneticSample;positioning:PositioningEnvironmentSample}
export function toGroupedEnvironmentSample(sample:EnvironmentSample):GroupedEnvironmentSample{
  const atmosphere:AtmosphericSample={wind_ned_mps:[...sample.wind_ned_mps],air_temperature_c:sample.air_temperature_c,pressure_pa:sample.pressure_pa,humidity_fraction:sample.humidity_fraction,rain_rate_mm_h:sample.rain_rate_mm_h,visibility_m:sample.visibility_m,fog_extinction_per_m:sample.fog_extinction_per_m,illumination_lux:sample.illumination_lux,...structuredClone(sample.atmosphere??{})};
  const water:AquaticSample={current_ned_mps:[...sample.current_ned_mps],...structuredClone(sample.water??{})};
  const surface:SurfaceSample={...sample.wave?{wave:structuredClone(sample.wave)}:{},...sample.tide_m!==undefined?{tide_m:sample.tide_m}:{},...structuredClone(sample.surface??{})};
  const electromagnetic=structuredClone(sample.electromagnetic??{}),positioning=structuredClone(sample.positioning??{});
  return{...structuredClone(sample),wind_ned_mps:[...atmosphere.wind_ned_mps],current_ned_mps:[...water.current_ned_mps],air_temperature_c:atmosphere.air_temperature_c,pressure_pa:atmosphere.pressure_pa,humidity_fraction:atmosphere.humidity_fraction,rain_rate_mm_h:atmosphere.rain_rate_mm_h,visibility_m:atmosphere.visibility_m,fog_extinction_per_m:atmosphere.fog_extinction_per_m,illumination_lux:atmosphere.illumination_lux,...surface.wave?{wave:structuredClone(surface.wave)}:{},...surface.tide_m!==undefined?{tide_m:surface.tide_m}:{},atmosphere,water,surface,electromagnetic,positioning};
}
export interface EnvironmentalField {sample(query:EnvironmentQuery):EnvironmentSample;provenance():Record<string,unknown>}
export class ConstantEnvironmentalField implements EnvironmentalField {private readonly value:GroupedEnvironmentSample;private readonly source:Record<string,unknown>;constructor(value:EnvironmentSample,source:Record<string,unknown>={mode:"constant"}){this.value=toGroupedEnvironmentSample(value);this.source=structuredClone(source);}sample():GroupedEnvironmentSample{return structuredClone(this.value);}provenance():Record<string,unknown>{return structuredClone(this.source);}}
export function wgs84ToNed(point:{latitude_deg:number;longitude_deg:number;altitude_m?:number},origin:{latitude_deg:number;longitude_deg:number;altitude_m?:number},headingDeg=0):[number,number,number]{return new LocalGeographicFrame(origin,headingDeg).wgs84ToNed(point);}
export function nedToWgs84(ned:[number,number,number],origin:{latitude_deg:number;longitude_deg:number;altitude_m?:number},headingDeg=0):{latitude_deg:number;longitude_deg:number;altitude_m:number}{return new LocalGeographicFrame(origin,headingDeg).nedToWgs84(ned) as {latitude_deg:number;longitude_deg:number;altitude_m:number};}
/** Boundary-only compatibility helper. Runtime and propagation code use NED. */
export function wgs84ToEnu(point:{latitude_deg:number;longitude_deg:number;altitude_m?:number},origin:{latitude_deg:number;longitude_deg:number;altitude_m?:number}):[number,number,number]{const f=new LocalGeographicFrame(origin),ned=f.wgs84ToNed(point);return f.nedToEnu(ned);}
/** Boundary-only compatibility helper. Runtime and propagation code use NED. */
export function enuToWgs84(enu:[number,number,number],origin:{latitude_deg:number;longitude_deg:number;altitude_m?:number}):{latitude_deg:number;longitude_deg:number;altitude_m:number}{const f=new LocalGeographicFrame(origin);return f.nedToWgs84(f.enuToNed(enu)) as {latitude_deg:number;longitude_deg:number;altitude_m:number};}
export interface DataProvenance {provider:"NOAA"|"USGS"|"Natural Earth";product:string;retrieved_at:string;version:string;resolution:string;coverage:string;checksum_sha256:string;crs:string;transformations:string[]}

export interface OfflineEnvironmentFixture {schema_version:1;provenance:DataProvenance;sample:EnvironmentSample}
export function validateOfflineFixture(fixture:OfflineEnvironmentFixture):void{if(fixture.schema_version!==1||!fixture.provenance?.crs||!/^[a-f0-9]{64}$/.test(fixture.provenance.checksum_sha256))throw new Error("Offline environmental fixture requires schema, CRS, and checksum provenance.");for(const value of [...fixture.sample.wind_ned_mps,...fixture.sample.current_ned_mps,fixture.sample.air_temperature_c,fixture.sample.pressure_pa,fixture.sample.visibility_m])if(!Number.isFinite(value))throw new Error("Offline environmental fixture contains non-finite fields.");}
export class OfflineEnvironmentalField implements EnvironmentalField {private readonly fixture:OfflineEnvironmentFixture;constructor(fixture:OfflineEnvironmentFixture){validateOfflineFixture(fixture);this.fixture=structuredClone(fixture);}sample():GroupedEnvironmentSample{return toGroupedEnvironmentSample(this.fixture.sample);}provenance():Record<string,unknown>{return{...structuredClone(this.fixture.provenance),mode:"immutable-offline-fixture",live_provider_status:"blocked-external-infrastructure"};}}
export function naturalEarthFallback():OfflineEnvironmentalField{return new OfflineEnvironmentalField({schema_version:1,provenance:{provider:"Natural Earth",product:"offline-open-water-fallback",retrieved_at:"2026-08-02T00:00:00.000Z",version:"5.1.2-fixture",resolution:"coarse",coverage:"global",checksum_sha256:"0".repeat(64),crs:"EPSG:4326",transformations:["WGS84 to local NED at experiment origin"]},sample:{wind_ned_mps:[0,0,0],current_ned_mps:[0,0,0],air_temperature_c:15,pressure_pa:101325,humidity_fraction:.5,rain_rate_mm_h:0,visibility_m:10000,fog_extinction_per_m:0,illumination_lux:10000,tide_m:0}});}
