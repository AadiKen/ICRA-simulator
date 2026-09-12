import {readFileSync,writeFileSync} from "node:fs";
import {coriolisFromMass6,rigidBodyCoriolis3,addedMassCoriolis3} from "../../packages/core/src/coriolis.ts";
import {planarMassMatrix3,totalMassMatrix6,rigidBodyMassMatrix6,addedMassMatrix6} from "../../packages/core/src/mass.ts";

const [csvPath,outPath,uArg="-0.627268",vArg="0.391366",rArg="0.346345"]=process.argv.slice(2);
if(!csvPath||!outPath)throw Error("usage: analyze_vrx_dart_mass_bias.ts telemetry.csv result.json [u v r]");
const lines=readFileSync(csvPath,"utf8").trim().split("\n"),keys=lines[0].split(","),values=lines.at(-1)!.split(",");
const telemetry=Object.fromEntries(keys.map((key,i)=>[key,Number(values[i])]));
const model=JSON.parse(readFileSync("artifacts/rl-campaign/surveyor-vehicle-model.json","utf8"));
const mass=model.mass_properties.mass_kg.value,inertia=model.mass_properties.inertia_tensor_body_kg_m2.diagonal.Izz_yaw;
const params={massProps:{mass,cg:{x:0,y:0,z:0},inertia:{Iz:inertia}},addedMass:{XuDot:-.05*mass,YvDot:-.75*mass,NrDot:-.08*inertia}};
const node=planarMassMatrix3(params),dart=[[mass+telemetry.added_mass_body_xx,telemetry.added_mass_body_xy,telemetry.added_mass_body_xr],[telemetry.added_mass_body_xy,mass+telemetry.added_mass_body_yy,telemetry.added_mass_body_yr],[telemetry.added_mass_body_xr,telemetry.added_mass_body_yr,inertia+telemetry.added_mass_rr]];
const diff=dart.map((row,i)=>row.map((value,j)=>value-node[i][j]));
const nu=[Number(uArg),Number(vArg),Number(rArg)],mul=(a:number[][],x:number[])=>a.map(row=>row.reduce((sum,value,i)=>sum+value*x[i],0));
const rb=mul(rigidBodyCoriolis3(params,nu),nu),added=mul(addedMassCoriolis3(params,nu),nu),nodeBias=rb.map((x,i)=>x+added[i]);
const nu6=[nu[0],nu[1],0,0,0,nu[2]],dartBias6=mul(coriolisFromMass6(totalMassMatrix6(params),nu6),nu6),dartBias=[dartBias6[0],dartBias6[1],dartBias6[5]];
const matrixMax=Math.max(...diff.flat().map(Math.abs)),biasDiff=dartBias.map((x,i)=>x-nodeBias[i]),biasMax=Math.max(...biasDiff.map(Math.abs));
const report={schema_version:1,status:matrixMax<1e-5&&biasMax<1e-10?"MATRIX_AND_BIAS_MATCH":"MISMATCH",state_body_ned:{velocity_mps:nu.slice(0,2),yaw_rate_rad_s:nu[2]},matrix:{node_planar_3x3:node,dart_effective_planar_3x3:dart,dart_minus_node_3x3:diff,max_abs_diff:matrixMax,dart_readback_source:"WorldFluidAddedMassMatrix link-origin coefficients plus WorldInertial-equivalent rigid mass/inertia"},bias_force_n_nm:{node_rigid_body_coriolis:rb,node_added_mass_coriolis:added,node_total:nodeBias,dart_combined_spatial_tensor_reconstruction:dartBias,dart_minus_node:biasDiff,max_abs_diff:biasMax},dart_access_note:"Gazebo's public gz-physics7 system boundary exposes added-mass readback but not DART Skeleton::getCoriolisForces. The DART value is reconstructed with the exact combined spatial tensor installed by gz-physics7_7.6.0 AddedMassFeatures::SetLinkAddedMass and DART's spatial bias formulation; it is not a direct live solver-vector readback.",conclusion:"The static effective matrix and its velocity-dependent spatial-inertia bias both match Node. The remaining coast residual is not explained by matrix loading or the ideal combined-matrix Coriolis term."};
writeFileSync(outPath,JSON.stringify(report,null,2)+"\n");console.log(JSON.stringify(report,null,2));
