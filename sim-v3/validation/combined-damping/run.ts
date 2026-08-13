import {createHash} from "node:crypto";
import {mkdirSync,readFileSync,writeFileSync} from "node:fs";
import {dirname,resolve} from "node:path";
import {fileURLToPath} from "node:url";
import {validateDamping,type DampingModel,type Matrix,type ModeName} from "../../packages/core/src/hydrodynamics.ts";

type ResolvedMode={hydrodynamics:{evaluation_frequency_rad_s:number;added_mass:Matrix;damping:DampingModel;free_decay_iterations:unknown[];selected_mode_vector:number[]};metrics:{natural_period_s:number}};
type ResolvedArtifact={vehicle_id:string;status:string;source_artifact:{checksum_sha256:string;mesh_checksum_sha256:string};approximation:{kind:string;warning:string};runtime_parameters:{massProps:{mass:number;inertia:{Ix:number;Iy:number;Iz:number}};restoring:{hydrostaticStiffnessMatrix6:Matrix}};damping_provenance:DampingModel["provenance"];free_decay:Record<ModeName,ResolvedMode>};

const MODES:ModeName[]=["heave","roll","pitch"],INDICES=[2,3,4];
const sha=(bytes:string|Buffer)=>createHash("sha256").update(bytes).digest("hex");
const subset=(matrix:Matrix)=>INDICES.map((i)=>INDICES.map((j)=>matrix[i][j]));
const diagonal=(values:number[]):Matrix=>values.map((value,i)=>values.map((_,j)=>i===j?value:0));
const matVec=(matrix:Matrix,vector:number[])=>matrix.map((row)=>row.reduce((sum,value,index)=>sum+value*vector[index],0));
const dot=(a:number[],b:number[])=>a.reduce((sum,value,index)=>sum+value*b[index],0);
const add=(a:Matrix,b:Matrix)=>a.map((row,i)=>row.map((value,j)=>value+b[i][j]));
function inverse(matrix:Matrix):Matrix{const n=matrix.length,a=matrix.map((row,i)=>[...row,...Array.from({length:n},(_,j)=>Number(i===j))]);for(let c=0;c<n;c++){let p=c;for(let r=c+1;r<n;r++)if(Math.abs(a[r][c])>Math.abs(a[p][c]))p=r;if(Math.abs(a[p][c])<1e-14)throw new Error("Singular mass matrix in combined-damping validation.");[a[c],a[p]]=[a[p],a[c]];const s=a[c][c];a[c]=a[c].map((v)=>v/s);for(let r=0;r<n;r++)if(r!==c){const f=a[r][c];a[r]=a[r].map((v,j)=>v-f*a[c][j]);}}return a.map((row)=>row.slice(n));}

export function dampingPower(model:DampingModel,velocity:number[]){
  const square=velocity.map((value)=>Math.abs(value)*value);
  const potential=dot(velocity,matVec(model.potentialRadiationDamping,velocity));
  const linearViscous=dot(velocity,matVec(model.linearViscousDamping,velocity));
  const quadraticViscous=dot(velocity,matVec(model.quadraticViscousDamping,square));
  return{potential,linear_viscous:linearViscous,quadratic_viscous:quadraticViscous,total:potential+linearViscous+quadraticViscous};
}

export function assertDissipative(model:DampingModel):{minimum_total_power:number;sample_count:number}{
  validateDamping(model);
  const vectors:number[][]=[];
  for(const scale of [.01,.1,1])for(let a=-1;a<=1;a++)for(let b=-1;b<=1;b++)for(let c=-1;c<=1;c++)if(a||b||c)vectors.push([a*scale,b*scale,c*scale]);
  const powers=vectors.map((velocity)=>dampingPower(model,velocity));
  const minimum=Math.min(...powers.map((value)=>value.total));
  if(minimum < -1e-9)throw new Error(`Combined damping is not dissipative over the validation sample set: ${minimum}.`);
  if(!powers.some((value)=>value.linear_viscous+value.quadratic_viscous>0))throw new Error("Viscous damping contributes no positive dissipation.");
  return{minimum_total_power:minimum,sample_count:vectors.length};
}

function simulate(mode:ModeName,rigid:Matrix,stiffness:Matrix,resolved:ResolvedMode["hydrodynamics"],includeViscous:boolean){
  const target=MODES.indexOf(mode),massInverse=inverse(add(rigid,resolved.added_mass));
  const linear=includeViscous?add(resolved.damping.potentialRadiationDamping,resolved.damping.linearViscousDamping):resolved.damping.potentialRadiationDamping;
  const quadratic=includeViscous?resolved.damping.quadraticViscousDamping:diagonal([0,0,0]);
  const initial=mode==="heave"?.05:10*Math.PI/180,dt=.002,steps=15000;
  let state=[0,0,0,0,0,0];state[target]=initial;
  const derivative=(s:number[])=>{const x=s.slice(0,3),v=s.slice(3),forces=matVec(stiffness,x).map((value,i)=>value+matVec(linear,v)[i]+matVec(quadratic,v.map((n)=>Math.abs(n)*n))[i]),acceleration=matVec(massInverse,forces.map((value)=>-value));return[...v,...acceleration];};
  const values:number[]=[];
  for(let step=0;step<=steps;step++){values.push(state[target]);if(step===steps)break;const shift=(base:number[],k:number[],factor:number)=>base.map((value,i)=>value+factor*k[i]),k1=derivative(state),k2=derivative(shift(state,k1,dt/2)),k3=derivative(shift(state,k2,dt/2)),k4=derivative(shift(state,k3,dt));state=state.map((value,i)=>value+dt*(k1[i]+2*k2[i]+2*k3[i]+k4[i])/6);if(state.some((value)=>!Number.isFinite(value)))throw new Error(`${mode} free decay produced non-finite state.`);}
  const peaks:number[]=[];for(let i=1;i<values.length-1;i++)if(values[i]>values[i-1]&&values[i]>=values[i+1]&&values[i]>0)peaks.push(values[i]);
  const decrement=peaks.length>=2?Math.log(peaks[0]/peaks.at(-1)!)/(peaks.length-1):null;
  const ratio=decrement===null?null:decrement/Math.sqrt(4*Math.PI*Math.PI+decrement*decrement);
  const threshold=.02*Math.abs(initial);let lastAbove=0;for(let i=0;i<values.length;i++)if(Math.abs(values[i])>threshold)lastAbove=i;
  return{initial_displacement:initial,positive_peak_count:peaks.length,logarithmic_decrement:decrement,damping_ratio:ratio,settling_time_s:lastAbove*dt,equilibrium_error:values.at(-1),trace_checksum_sha256:sha(JSON.stringify(values)),simulation:{method:"rk4",dt_s:dt,duration_s:steps*dt,steps}};
}

export function validateCombinedDamping(sourcePath:string){
  const bytes=readFileSync(sourcePath),source=JSON.parse(bytes.toString()) as ResolvedArtifact;
  for(const [name,provenance] of Object.entries(source.damping_provenance))for(const field of ["source","checksum","uncertainty","operating_range","fit_data"] as const)if(!provenance[field])throw new Error(`${name} damping provenance is missing ${field}.`);
  const rigid=diagonal([source.runtime_parameters.massProps.mass,source.runtime_parameters.massProps.inertia.Ix,source.runtime_parameters.massProps.inertia.Iy]);
  const stiffness=subset(source.runtime_parameters.restoring.hydrostaticStiffnessMatrix6);
  const modes=Object.fromEntries(MODES.map((mode)=>{
    const selected=source.free_decay[mode].hydrodynamics,diagnostic=assertDissipative(selected.damping),combined=simulate(mode,rigid,stiffness,selected,true),radiationOnly=simulate(mode,rigid,stiffness,selected,false);
    if(combined.logarithmic_decrement===null||radiationOnly.logarithmic_decrement===null)throw new Error(`${mode} did not produce enough positive peaks for damping extraction.`);
    if(!(combined.logarithmic_decrement>radiationOnly.logarithmic_decrement))throw new Error(`${mode} viscous damping did not increase logarithmic decrement.`);
    return[mode,{evaluation_frequency_rad_s:selected.evaluation_frequency_rad_s,natural_period_s:2*Math.PI/selected.evaluation_frequency_rad_s,frequency_selection:{method:"free-decay-fixed-point",iteration_count:selected.free_decay_iterations.length,selected_mode_vector:selected.selected_mode_vector},damping:selected.damping,dissipation_sample:diagnostic,radiation_only_diagnostic:radiationOnly,combined,viscous_effect:{logarithmic_decrement_increase:combined.logarithmic_decrement-radiationOnly.logarithmic_decrement,damping_ratio_increase:combined.damping_ratio!-radiationOnly.damping_ratio!,settling_time_change_s:combined.settling_time_s-radiationOnly.settling_time_s}}];
  }));
  return{schema_version:1,artifact_kind:"combined-damping-validation",vehicle_id:source.vehicle_id,status:"software-gate-passed-physical-validation-blocked",evidence_scope:"deterministic composition, dissipativity, and free-decay extraction",is_software_validation_evidence:true,is_physical_validation_evidence:false,source_resolved_artifact:{path:sourcePath,checksum_sha256:sha(bytes),capytaine_artifact_checksum_sha256:source.source_artifact.checksum_sha256,mesh_checksum_sha256:source.source_artifact.mesh_checksum_sha256},approximation:source.approximation,damping_provenance:source.damping_provenance,acceptance:{viscous_required:true,all_components_separately_provenanced:true,combined_decay_must_exceed_radiation_only:true,dissipative_sample_required:true,non_finite_output_allowed:false},modes,limitations:["Capytaine and the current hull meshes are bootstrap solver outputs, not reviewed physical validation evidence.","Viscous coefficients are engineering bootstrap estimates and are not fitted to independent Vehicle B/C free-decay measurements.","Radiation-only traces are diagnostic counterfactuals and are invalid production coupled6 configurations.","The constant-coefficient model omits Cummins radiation memory.","Independent free-decay measurements and reviewed meshes are required before claiming physical damping validation."]};
}

if(process.argv[1]===fileURLToPath(import.meta.url)){
  const input=process.argv[2],output=resolve(process.argv[3]);if(!input||!process.argv[3])throw new Error("Usage: run.ts <resolved-capytaine.json> <output.json>");
  const artifact=validateCombinedDamping(input);mkdirSync(dirname(output),{recursive:true});writeFileSync(output,`${JSON.stringify(artifact,null,2)}\n`);console.log(JSON.stringify({output,status:artifact.status,vehicle_id:artifact.vehicle_id,modes:Object.fromEntries(Object.entries(artifact.modes).map(([mode,value])=>[mode,(value as any).viscous_effect]))},null,2));
}
