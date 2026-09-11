import {createHash} from "node:crypto";
import {mkdir,writeFile} from "node:fs/promises";
import {dirname,resolve} from "node:path";

const BASE="https://encdirect.noaa.gov/arcgis/rest/services/encdirect/enc_harbour/MapServer";
const SITES:Record<string,[number,number]>= {honolulu:[21.289,-157.865],miami:[25.731,-80.162],boston:[42.354,-70.989]};
const OBSTACLES=[".Obstruction_point",".Buoy_Lateral_point",".Wreck_point",".Pile_point",".Underwater_Awash_Rock_point"];
const QUALITY=".Quality_of_Data_area";
const sha256=(text:string)=>createHash("sha256").update(text).digest("hex");

async function get(url:string){const started=performance.now(),response=await fetch(url),text=await response.text();if(!response.ok)throw new Error(`${response.status} ${url}`);const json=JSON.parse(text);if(json.error)throw new Error(JSON.stringify(json.error));return{json,evidence:{url,status:response.status,bytes:Buffer.byteLength(text),latency_ms:Number((performance.now()-started).toFixed(3)),checksum_sha256:sha256(text)}};}
function layer(layers:any[],suffix:string){const found=layers.filter(x=>x.name?.endsWith(suffix));if(found.length!==1)throw new Error(`Expected one layer ${suffix}, found ${found.length}`);return found[0];}
function inside(point:number[],rings:number[][][]){let value=false;for(const ring of rings){let hit=false;for(let i=0,j=ring.length-1;i<ring.length;j=i++){const [xi,yi]=ring[i],[xj,yj]=ring[j];if((yi>point[1])!==(yj>point[1])&&point[0]<(xj-xi)*(point[1]-yi)/((yj-yi)||Number.EPSILON)+xi)hit=!hit;}value=value!==hit;}return value;}
function contains(point:number[],geometry:any){return geometry?.type==="Polygon"?inside(point,geometry.coordinates):geometry?.type==="MultiPolygon"&&geometry.coordinates.some((p:number[][][])=>inside(point,p));}
async function query(id:number,bbox:string){const url=new URL(`${BASE}/${id}/query`);url.search=new URLSearchParams({geometry:bbox,geometryType:"esriGeometryEnvelope",inSR:"4326",outSR:"4326",spatialRel:"esriSpatialRelIntersects",outFields:"*",returnGeometry:"true",f:"geojson",resultRecordCount:"2000"}).toString();return get(url);}

async function main(){
 const output=resolve(process.argv[2]??"artifacts/environment-coverage/enc-catzoc-remaining-sites.json"),catalogue=await get(`${BASE}/layers?f=pjson`),layers=catalogue.json.layers,requests:any[]=[catalogue.evidence],results:any[]=[];
 for(const [site,[lat,lon]] of Object.entries(SITES)){
  const bbox=[lon-.1,lat-.09,lon+.1,lat+.09].join(","),obstacles:any[]=[];
  for(const suffix of OBSTACLES){const selected=layer(layers,suffix),response=await query(selected.id,bbox);requests.push(response.evidence);obstacles.push(...(response.json.features??[]).map((f:any)=>({...f,_source_layer:selected.name})));}
  const qualityLayer=layer(layers,QUALITY),qualityResponse=await query(qualityLayer.id,bbox);requests.push(qualityResponse.evidence);const quality=qualityResponse.json.features??[];
  const assignments=obstacles.map((feature:any)=>{const candidates=quality.filter((polygon:any)=>contains(feature.geometry.coordinates,polygon.geometry)).sort((a:any,b:any)=>Number(b.properties?.CATZOC??-Infinity)-Number(a.properties?.CATZOC??-Infinity));return{obstacle_ref:`${feature.properties?.DSNM??"unknown"}:${feature.properties?.OBJECTID??"unknown"}`,source_layer:feature._source_layer,catzoc_code:candidates[0]?.properties?.CATZOC??null,candidate_count:candidates.length};});
  const covered=assignments.filter((x:any)=>x.candidate_count>0);results.push({site,bbox:{xmin:lon-.1,ymin:lat-.09,xmax:lon+.1,ymax:lat+.09},obstacle_count:assignments.length,catzoc_covered_obstacle_count:covered.length,catzoc_coverage_fraction:assignments.length?covered.length/assignments.length:null,quality_polygon_count:quality.length,catzoc_counts:Object.fromEntries([...new Set(covered.map((x:any)=>String(x.catzoc_code)))].sort().map(value=>[value,covered.filter((x:any)=>String(x.catzoc_code)===value).length])),assignments});
 }
 const artifact={schema_version:1,artifact_kind:"enc-obstacle-catzoc-remaining-sites",generated_at:new Date().toISOString(),methodology:"Live NOAA ENC Direct harbour-band obstacle queries using the same five point classes and obstacle-weighted CATZOC polygon assignment as the retained San Francisco pass.",service:BASE,results,requests,credential_leak_scan:{pass:requests.every(x=>!/[?&](token|key|password|secret)=/i.test(x.url)),findings:[]}};
 await mkdir(dirname(output),{recursive:true});await writeFile(output,JSON.stringify(artifact,null,2)+"\n");console.log(JSON.stringify({output,results:results.map(({assignments,...x})=>x)},null,2));
}
await main();
