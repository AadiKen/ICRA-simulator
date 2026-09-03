import assert from "node:assert/strict";
import {OfflineEnvironmentalField,enuToWgs84,naturalEarthFallback,wgs84ToEnu} from "../src/index.ts";
const origin={latitude_deg:37.8,longitude_deg:-122.4,altitude_m:3},point={latitude_deg:37.801,longitude_deg:-122.399,altitude_m:7},enu=wgs84ToEnu(point,origin),round=enuToWgs84(enu,origin);assert.ok(Math.abs(round.latitude_deg-point.latitude_deg)<1e-10);assert.ok(Math.abs(round.longitude_deg-point.longitude_deg)<1e-10);assert.equal(round.altitude_m,7);
const fixture:any={schema_version:1,provenance:{provider:"NOAA",product:"offline-test",retrieved_at:"2026-08-02T00:00:00Z",version:"1",resolution:"fixture",coverage:"test",checksum_sha256:"a".repeat(64),crs:"EPSG:4326",transformations:["to NED"]},sample:{wind_ned_mps:[1,0,0],current_ned_mps:[0,.2,0],air_temperature_c:15,pressure_pa:101325,humidity_fraction:.5,rain_rate_mm_h:0,visibility_m:1000,fog_extinction_per_m:0,illumination_lux:100,tide_m:0}};const field=new OfflineEnvironmentalField(fixture);assert.deepEqual(field.sample().wind_ned_mps,[1,0,0]);assert.equal(field.provenance().live_provider_status,"blocked-external-infrastructure");assert.equal(naturalEarthFallback().provenance().provider,"Natural Earth");assert.throws(()=>new OfflineEnvironmentalField({...fixture,provenance:{...fixture.provenance,crs:""}}),/CRS/);
console.log("Environment tests passed.");
await import("../../../validation/environment-coverage/run.test.ts");
await import("./rtofs-decoder.test.ts");
await import("./gebco.test.ts");
