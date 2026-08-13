import {createHash} from "node:crypto";
import {readFileSync} from "node:fs";
import {VehicleParameters} from "../../../core/vehicleParameters.js";
import {createVehicleBMmgParameters} from "../../../packages/core/src/vehicle-b-mmg.js";
import {VEHICLES} from "../../../packages/vehicle-sdk/src/index.ts";

const load=(url:URL)=>JSON.parse(readFileSync(url,"utf8"));
const stable=(value:any):string=>Array.isArray(value)?`[${value.map(stable).join(",")}]`:value&&typeof value==="object"?`{${Object.entries(value).filter(([key])=>key!=="checksum_sha256").sort(([a],[b])=>a.localeCompare(b)).map(([key,item])=>`${JSON.stringify(key)}:${stable(item)}`).join(",")}}`:JSON.stringify(value);
const checksum=(value:any)=>createHash("sha256").update(stable(value)).digest("hex");
const symmetrize=(matrix:number[][])=>matrix.map((row,i)=>row.map((value,j)=>(value+matrix[j][i])/2));

export function buildVehicleBProductionConfiguration(){
  const definition=VEHICLES["vehicle-b-rudder"],hydrodynamics=load(new URL("../../../artifacts/capytaine/vehicle-b-parametric-resolved.json",import.meta.url)),configuration=load(new URL("../../../packages/vehicle-sdk/config/vehicle-b-usv-bootstrap.json",import.meta.url));
  if(definition.plant!=="coupled6"||hydrodynamics.vehicle_id!==definition.id)throw new Error("Vehicle B production hydrodynamics identity mismatch.");
  if(checksum(configuration)!==configuration.provenance.checksum_sha256)throw new Error("Vehicle B USV MMG configuration checksum mismatch.");
  const runtime=hydrodynamics.runtime_parameters,mmg=createVehicleBMmgParameters(definition,configuration),parameters=new VehicleParameters({
    id:definition.id,vehicleClass:"surface_coupled6",geometry:{...runtime.geometry,height:definition.geometry.draft.value*2},massProps:{...runtime.massProps,cg:{x:configuration.x_g_m,y:0,z:0}},addedMass:{matrix6:symmetrize(runtime.addedMass.matrix6)},damping:{...runtime.damping},hydrodynamics:runtime.hydrodynamics,restoring:{waterDensity:1025,gravity:9.80665,waterplaneArea:definition.geometry.length.value*definition.geometry.width.value,displacementVolume:definition.mass.value/1025,metacentricHeightRoll:definition.hydrostatics!.gm_transverse.value,metacentricHeightPitch:definition.hydrostatics!.gm_longitudinal.value,hydrostaticStiffnessMatrix6:runtime.restoring.hydrostaticStiffnessMatrix6,cob:{x:0,y:0,z:-definition.geometry.draft.value/2}},actuator:{maxThrust:configuration.command_mapping.max_surge_force_n,beam:definition.geometry.width.value,motorTimeConstant:configuration.propeller.time_constant_s},maneuveringModel:{type:"mmg",parameterSetId:configuration.parameter_set_id,replacesPlanarDamping:true},validation:{status:definition.validation.status,claim:definition.validation.claim,limitations:definition.validation.limitations,production_path:"coupled6-mmg-usv-bootstrap",hydrodynamics_status:hydrodynamics.status}
  });
  return{definition,hydrodynamics,configuration,parameters,mmg,manifest:{vehicle_id:definition.id,plant:"coupled6",maneuvering_model:"mmg",parameter_set_id:configuration.parameter_set_id,parameter_checksum_sha256:configuration.provenance.checksum_sha256,hydrodynamics_path:definition.potential_flow!.artifact_path,hydrodynamics_checksum_sha256:hydrodynamics.source_artifact.checksum_sha256,hydrodynamic_evaluation_frequency_rad_s:runtime.hydrodynamics.evaluation_frequency_rad_s,hydrodynamic_selection_method:runtime.hydrodynamics.selection_method,hydrodynamic_approximation:runtime.hydrodynamics.approximation,planar_damping_source:"MMG hull derivatives",out_of_plane_damping_source:"Parametric-hull Capytaine radiation plus empirical viscous matrices; roll uses the tracked zero-speed Ikeda lower bound",roll_viscous_damping_artifact:definition.damping.roll_viscous_decomposition?.artifact_path,validation_status:definition.validation.status,claim_limit:"Production integration only; Vehicle B USV dynamics remain behaviorally unvalidated."}};
}
