import {readFileSync,existsSync} from "node:fs";
import {dirname,join,resolve} from "node:path";
import {fileURLToPath} from "node:url";

const root=dirname(fileURLToPath(import.meta.url));
export const load=(name)=>JSON.parse(readFileSync(join(root,name),"utf8"));
const finite=(value)=>typeof value==="number"&&Number.isFinite(value);
const parameters=(section)=>Object.values(section);

export function validateVehicle(config){
  if(config.schema_version!==1||config.id!=="kvlcc2-l7"||config.model_scale!==45.7)throw new Error("Invalid KVLCC2 L7 identity or scale.");
  if(JSON.stringify(config.frame)!==JSON.stringify({origin:"midship",x:"forward",y:"starboard",z:"down"}))throw new Error("KVLCC2 convention must be midship-fixed x-forward/y-starboard/z-down.");
  const expected={force:"0.5*rho*Lpp*d*U^2",yaw_moment:"0.5*rho*Lpp^2*d*U^2",mass:"0.5*rho*Lpp^2*d",yaw_inertia:"0.5*rho*Lpp^4*d"};
  if(JSON.stringify(config.nondimensionalization)!==JSON.stringify(expected))throw new Error("KVLCC2 non-dimensionalization convention mismatch.");
  for(const section of [config.principal_particulars,config.hull_derivatives,config.added_mass,config.interaction_coefficients,config.propeller,config.test_conditions])for(const parameter of parameters(section)){if(!parameter.provenance?.source?.includes("Yasukawa & Yoshimura 2015")||!parameter.provenance?.table)throw new Error("Every KVLCC2 parameter requires source and table provenance.");if(typeof parameter.value!=="string"&&!finite(parameter.value))throw new Error("KVLCC2 parameter values must be finite.");}
  for(const term of parameters(config.added_mass))if(term.provenance.method!=="Motora empirical charts"||term.provenance.potential_flow!==false)throw new Error("MMG added mass must remain Motora empirical, not Capytaine potential flow.");
  if(config.capytaine_overwrite_policy.added_mass!=="forbidden")throw new Error("Capytaine must not overwrite MMG added mass.");
  return config;
}

export function validateMetricFixture(fixture){
  if(fixture.schema_version!==1||fixture.metrics?.length!==10||fixture.trajectory_overlays_gating!==false)throw new Error("Invalid KVLCC2 reference-neutral metric fixture.");
  const ids=new Set();for(const metric of fixture.metrics){if(ids.has(metric.id)||!finite(metric.value)||!["port","starboard"].includes(metric.direction)||!["Lpp","deg"].includes(metric.unit)||!["Table 4","Table 5"].includes(metric.source_table))throw new Error(`Invalid maneuver metric ${metric.id}.`);ids.add(metric.id);}
  for(const direction of ["port","starboard"])if(!fixture.metrics.some((metric)=>metric.direction===direction))throw new Error(`Missing ${direction} metrics.`);
  return fixture;
}

export function requireCache(){
  const cache=resolve(process.env.BCOD_KVLCC2_CACHE||join(root,"cache"));
  if(!existsSync(join(cache,"kvlcc2","Yasukawa_Yoshimura_2015_MMG.pdf")))throw new Error("KVLCC2 cache is absent; run fetch_public_assets.sh");
}

if(process.argv[1]===fileURLToPath(import.meta.url)){requireCache();validateVehicle(load("kvlcc2-l7.json"));validateMetricFixture(load("fixtures/kvlcc2-mmg-reference-reproduction.json"));validateMetricFixture(load("fixtures/kvlcc2-experimental-maneuver-indices.json"));console.log("KVLCC2 external-reference package validated.");}
