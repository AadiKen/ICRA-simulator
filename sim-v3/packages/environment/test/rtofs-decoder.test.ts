import assert from "node:assert/strict";
import h5wasm from "h5wasm";
import {decodeRtofsNetcdf4,NomadsRtofsGridDecoder} from "../src/index.ts";

const {FS}=await h5wasm.ready,path="/rtofs-real-schema-fixture.nc",height=5,width=5,file=new h5wasm.File(path,"w"),grid=(value:(y:number,x:number)=>number)=>Float32Array.from({length:height*width},(_,index)=>value(Math.floor(index/width),index%width)),fill=1.2676506e30;
file.create_dataset({name:"Latitude",data:grid((y)=>37.6+y*.1),shape:[height,width],dtype:"<f4",chunks:[height,width],compression:"gzip",compression_opts:1});
file.create_dataset({name:"Longitude",data:grid((_y,x)=>237.3+x*.1),shape:[height,width],dtype:"<f4",chunks:[height,width],compression:"gzip",compression_opts:1});
for(const[name,value]of [["u_velocity",.125],["v_velocity",-.25]]as const)file.create_dataset({name,data:grid((y,x)=>y===2&&x===2?fill:value),shape:[1,1,height,width],dtype:"<f4",chunks:[1,1,height,width],compression:"gzip",compression_opts:1});
for(const[name,value]of [["sst",17.5],["sss",31.75]]as const)file.create_dataset({name,data:grid((y,x)=>y===2&&x===2?fill:value),shape:[1,height,width],dtype:"<f4",chunks:[1,height,width],compression:"gzip",compression_opts:1});
file.create_dataset({name:"MT",data:new Float64Array([45892]),shape:[1],dtype:"<f8"});file.close();const bytes=FS.readFile(path).buffer.slice(0) as ArrayBuffer;FS.unlink(path);
const decoded=await decodeRtofsNetcdf4(bytes,{latitude_deg:37.8,longitude_deg:-122.5},"2026-08-24T00:00:00Z");
assert.match(decoded.grid_id,/rtofs:Y2,X1@37\.799999,-122\.600006/);assert.equal(decoded.current_east_mps,.125);assert.equal(decoded.current_north_mps,-.25);assert.equal(decoded.temperature_c,17.5);assert.equal(decoded.salinity_psu,31.75);assert.equal(decoded.valid_time,"2026-08-24T00:00:00.000Z");
await assert.rejects(()=>decodeRtofsNetcdf4(new ArrayBuffer(8),{latitude_deg:37.8,longitude_deg:-122.5},"2026-08-24T00:00:00Z"),/not NetCDF4\/HDF5/);
let requested="";const production=new NomadsRtofsGridDecoder("20260823T00Z",undefined,async(url)=>{requested=String(url);return new Response(bytes,{status:200});});const cell=await production.query({latitude_deg:37.8,longitude_deg:-122.5},"2026-08-24T00:00:00Z");assert.match(requested,/rtofs\.20260823\/rtofs_glo_2ds_f024_prog\.nc$/);assert.equal(cell.cycle,"20260823T00Z");assert.equal(cell.current_east_mps,.125);
console.log("RTOFS NetCDF4/HDF5 decoder and production default wiring tests passed.");
