import type {Dataset} from "h5wasm";
import type {Wgs84Position} from "./geography.ts";
import type {RtofsBinaryDecoder,RtofsDecodedCell} from "./weather.ts";

const FILL_LIMIT=1e20,COARSE_STRIDE=32;
let fileSequence=0;
const values=(dataset:Dataset,ranges:number[][])=>{const result=dataset.slice(ranges as any);if(!ArrayBuffer.isView(result))throw new Error(`RTOFS dataset ${dataset.path} did not return a numeric array.`);return result as Float32Array|Float64Array;};
const one=(dataset:Dataset,ranges:number[][])=>Number(values(dataset,ranges)[0]);
const longitudeDelta=(a:number,b:number)=>{const x=((a-b+540)%360)-180;return x;};
const distance2=(lat:number,lon:number,point:Wgs84Position)=>{const dy=lat-point.latitude_deg,dx=longitudeDelta(lon,point.longitude_deg)*Math.cos(point.latitude_deg*Math.PI/180);return dy*dy+dx*dx;};
const finiteOcean=(...xs:number[])=>xs.every(x=>Number.isFinite(x)&&Math.abs(x)<FILL_LIMIT);
const validTime=(mtDays:number)=>{if(!Number.isFinite(mtDays))throw new Error("RTOFS MT time coordinate is not finite.");return new Date(Date.UTC(1900,11,31)+mtDays*86400_000).toISOString();};

/** Decode a complete NOAA RTOFS 2-D surface prognostic NetCDF4/HDF5 payload. */
export const decodeRtofsNetcdf4:RtofsBinaryDecoder=async(bytes,point,_requestedTime):Promise<RtofsDecodedCell>=>{
  const signature=[...new Uint8Array(bytes.slice(0,8))].map(x=>x.toString(16).padStart(2,"0")).join("");
  if(signature!=="894844460d0a1a0a")throw new Error(`RTOFS payload is not NetCDF4/HDF5 (signature ${signature||"empty"}). A byte-range header alone is not decodable; provide the complete file.`);
  const h5wasm=(await import("h5wasm")).default,{FS}=await h5wasm.ready,path=`/rtofs-${++fileSequence}.nc`;
  FS.writeFile(path,new Uint8Array(bytes));
  const file=new h5wasm.File(path,"r");
  try{
    const get=(name:string)=>{const dataset=file.get(name);if(!(dataset instanceof h5wasm.Dataset))throw new Error(`RTOFS NetCDF4 payload is missing dataset ${name}.`);return dataset as Dataset;},latitude=get("Latitude"),longitude=get("Longitude"),u=get("u_velocity"),v=get("v_velocity"),sst=get("sst"),sss=get("sss"),mt=get("MT"),shape=latitude.shape;
    if(!shape||shape.length!==2||longitude.shape?.join(",")!==shape.join(","))throw new Error("RTOFS Latitude/Longitude grid dimensions are inconsistent.");
    const[height,width]=shape,coarseLat=values(latitude,[[0,height,COARSE_STRIDE],[0,width,COARSE_STRIDE]]),coarseLon=values(longitude,[[0,height,COARSE_STRIDE],[0,width,COARSE_STRIDE]]),coarseWidth=Math.ceil(width/COARSE_STRIDE);let coarseIndex=-1,coarseDistance=Infinity;
    for(let index=0;index<coarseLat.length;index++){const d=distance2(Number(coarseLat[index]),Number(coarseLon[index]),point);if(Number.isFinite(d)&&d<coarseDistance){coarseDistance=d;coarseIndex=index;}}
    if(coarseIndex<0)throw new Error("RTOFS coordinate grid contains no finite cells.");
    const centerY=Math.floor(coarseIndex/coarseWidth)*COARSE_STRIDE,centerX=coarseIndex%coarseWidth*COARSE_STRIDE;
    let best:{distance:number;y:number;x:number;lat:number;lon:number;u:number;v:number;sst:number;sss:number}|undefined;
    for(const radius of [COARSE_STRIDE,COARSE_STRIDE*4,COARSE_STRIDE*16]){const y0=Math.max(0,centerY-radius),y1=Math.min(height,centerY+radius+1),x0=Math.max(0,centerX-radius),x1=Math.min(width,centerX+radius+1),windowWidth=x1-x0,lat=values(latitude,[[y0,y1],[x0,x1]]),lon=values(longitude,[[y0,y1],[x0,x1]]),east=values(u,[[0,1],[0,1],[y0,y1],[x0,x1]]),north=values(v,[[0,1],[0,1],[y0,y1],[x0,x1]]),temperature=values(sst,[[0,1],[y0,y1],[x0,x1]]),salinity=values(sss,[[0,1],[y0,y1],[x0,x1]]);
      for(let index=0;index<lat.length;index++){const fields=[Number(east[index]),Number(north[index]),Number(temperature[index]),Number(salinity[index])];if(!finiteOcean(Number(lat[index]),Number(lon[index]),...fields))continue;const d=distance2(Number(lat[index]),Number(lon[index]),point);if(!best||d<best.distance)best={distance:d,y:y0+Math.floor(index/windowWidth),x:x0+index%windowWidth,lat:Number(lat[index]),lon:Number(lon[index]),u:fields[0],v:fields[1],sst:fields[2],sss:fields[3]};}
      if(best)break;
    }
    if(!best)throw new Error("RTOFS contains no valid ocean cell near the requested coordinate; fill values were rejected.");
    const normalizedLongitude=(((best.lon+180)%360+360)%360)-180;
    return{grid_id:`rtofs:Y${best.y},X${best.x}@${best.lat.toFixed(6)},${normalizedLongitude.toFixed(6)}`,valid_time:validTime(one(mt,[[0,1]])),current_north_mps:best.v,current_east_mps:best.u,temperature_c:best.sst,salinity_psu:best.sss};
  }finally{file.close();FS.unlink(path);}
};
