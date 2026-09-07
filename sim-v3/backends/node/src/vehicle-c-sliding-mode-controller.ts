import {buildVehicleCProductionConfiguration} from "./vehicle-c-production.ts";
import {coriolisFromMass6,eulerRateMatrix,rotationBodyToNed,totalMassMatrix6} from "../../../core/sixDof.js";
import {matVecMul} from "../../../core/math.js";

export type VehicleCControllerMode="backstepping"|"sliding-mode";
export type VehicleCControllerState={position_ned_m:[number,number,number];attitude_rad:[number,number,number];velocity_body_mps:[number,number,number];angular_rate_body_rad_s:[number,number,number]};
export type VehicleCTarget={north_m:number;east_m:number;heading_rad:number;waypoint_index:number};
export type VehicleCSlidingModeOptions={mode:VehicleCControllerMode;dt_s:number;lambda?:number;kp?:[number,number,number];kd?:[number,number,number];boundary_layer?:[number,number,number];uncertainty_bound?:[number,number,number]};

export const VEHICLE_C_LAMBDA_DERIVATION=(()=>{const gravity_m_s2=9.80665,beam_m=1.98,wave_celerity_m_s=Math.sqrt(gravity_m_s2*beam_m/(2*Math.PI)),resonant_frequency_hz=wave_celerity_m_s/beam_m,sampling_frequency_hz=20,actuator_delay_s=.35,resonant=(2/3)*Math.PI*resonant_frequency_hz,sampling=sampling_frequency_hz/5,delay=1/(3*actuator_delay_s);return{gravity_m_s2,beam_m,wave_celerity_m_s,resonant_frequency_hz,candidates_per_s:{resonant,sampling,actuator_delay:delay},selected_per_s:Math.min(resonant,sampling,delay),selected_by:"slowest-decay-rate",azimuth_slew_note:"30 deg/s is a nonlinear rate limit, not folded into the paper's pure-delay criterion."};})();

const wrap=(x:number)=>Math.atan2(Math.sin(x),Math.cos(x));
const transpose=(m:number[][])=>m[0].map((_,j)=>m.map(row=>row[j]));
const add=(...vectors:number[][])=>vectors[0].map((_,i)=>vectors.reduce((sum,v)=>sum+v[i],0));
const scale=(v:number[],k:number)=>v.map(x=>x*k);
const diagMul=(d:number[],v:number[])=>v.map((x,i)=>d[i]*x);
const embedPlanar=(v:[number,number,number])=>[v[0],v[1],0,0,0,v[2]];
const projectPlanar=(v:number[]):[number,number,number]=>[v[0],v[1],v[5]];
const dampingTimes=(parameters:any,relativeNu:number[],vector:number[])=>{const d=parameters.damping,p=d.potentialRadiationMatrix6??Array.from({length:6},()=>Array(6).fill(0)),l=d.linearViscousMatrix6??Array.from({length:6},()=>Array(6).fill(0)),q=d.quadraticViscousMatrix6??Array.from({length:6},()=>Array(6).fill(0));return p.map((row:number[],i:number)=>row.reduce((sum:number,x:number,j:number)=>sum+(x+l[i][j]+q[i][j]*Math.abs(relativeNu[j]))*vector[j],0));};
const j6=(state:VehicleCControllerState)=>{const [roll,pitch,yaw]=state.attitude_rad,J=Array.from({length:6},()=>Array(6).fill(0)),R=rotationBodyToNed({roll,pitch,yaw}),T=eulerRateMatrix({roll,pitch});for(let i=0;i<3;i++)for(let k=0;k<3;k++){J[i][k]=R[i][k];J[i+3][k+3]=T[i][k];}return J;};

/** Vehicle-C-only Sarda-style controller. It deliberately does not call the legacy Coriolis feedforward. */
export class VehicleCSlidingModeController{
  readonly mode:VehicleCControllerMode;readonly dt:number;readonly lambda:number;readonly kp:[number,number,number];readonly kd:[number,number,number];readonly E:[number,number,number];readonly R:[number,number,number];readonly massMatrix:number[][];readonly parameters:any;
  #integral:[number,number,number]=[0,0,0];#previousJ:number[][]|null=null;#targetKey:number|null=null;#saturated=false;
  constructor(options:VehicleCSlidingModeOptions){this.mode=options.mode;this.dt=options.dt_s;this.lambda=options.lambda??VEHICLE_C_LAMBDA_DERIVATION.selected_per_s;this.kp=options.kp??[485.9950330344873,485.9950330344873,485.9950330344873];this.kd=options.kd??[849.6899964175157,849.6899964175157,849.6899964175157];this.E=options.boundary_layer??[1,1,.1];this.R=options.uncertainty_bound??[100,100,100];const production=buildVehicleCProductionConfiguration();this.parameters=production.parameters;this.massMatrix=totalMassMatrix6(this.parameters);}
  reset(){this.#integral=[0,0,0];this.#previousJ=null;this.#targetKey=null;this.#saturated=false;}
  noteAllocationSaturated(saturated:boolean){this.#saturated=saturated;}
  action(state:VehicleCControllerState,target:VehicleCTarget,currentNedMps:[number,number,number]){
    const switched=this.#targetKey!==target.waypoint_index;if(switched){this.#targetKey=target.waypoint_index;this.#integral=[0,0,0];this.#previousJ=null;}
    const etaError:[number,number,number]=[state.position_ned_m[0]-target.north_m,state.position_ned_m[1]-target.east_m,wrap(state.attitude_rad[2]-target.heading_rad)],J=j6(state),JT=transpose(J),nu=[...state.velocity_body_mps,...state.angular_rate_body_rad_s],etaDot6=matVecMul(J,nu),etaDot=projectPlanar(etaDot6),antiWindup=this.#saturated?.1:1;
    if(this.mode==="sliding-mode")for(let i=0;i<3;i++)this.#integral[i]+=etaError[i]*this.dt*antiWindup;
    const reference=this.mode==="sliding-mode"?etaError.map((e,i)=>-2*this.lambda*e-this.lambda*this.lambda*this.#integral[i]):etaError.map(e=>-this.lambda*e),referenceDot=this.mode==="sliding-mode"?etaDot.map((v,i)=>-2*this.lambda*v-this.lambda*this.lambda*etaError[i]*antiWindup):etaDot.map(v=>-this.lambda*v),surface=this.mode==="sliding-mode"?etaDot.map((v,i)=>v+2*this.lambda*etaError[i]+this.lambda*this.lambda*this.#integral[i]):etaDot.map((v,i)=>v+this.lambda*etaError[i]);
    const reference6=matVecMul(JT,embedPlanar(reference as [number,number,number])),referenceDotEarth6=embedPlanar(referenceDot as [number,number,number]),JdotT=this.#previousJ?transpose(J).map((row,i)=>row.map((x,k)=>(x-transpose(this.#previousJ!)[i][k])/this.dt)):J.map(row=>row.map(()=>0)),bodyReferenceAcceleration=add(matVecMul(JT,referenceDotEarth6),matVecMul(JdotT,embedPlanar(reference as [number,number,number])));this.#previousJ=J.map(row=>[...row]);
    const rotation=rotationBodyToNed({roll:state.attitude_rad[0],pitch:state.attitude_rad[1],yaw:state.attitude_rad[2]}),waterBody=matVecMul(transpose(rotation),currentNedMps),relativeNu=[nu[0]-waterBody[0],nu[1]-waterBody[1],nu[2]-waterBody[2],nu[3],nu[4],nu[5]],coriolis=matVecMul(coriolisFromMass6(this.massMatrix,relativeNu),reference6),damping=dampingTimes(this.parameters,relativeNu,reference6),model=add(matVecMul(this.massMatrix,bodyReferenceAcceleration),coriolis,damping);
    const feedbackEarth=this.mode==="backstepping"?add(scale(diagMul(this.kd,surface),-1),scale(diagMul(this.kp,etaError),-1)):surface.map((s,i)=>-this.R[i]*Math.max(-1,Math.min(1,s/Math.max(this.E[i],1e-9)))),feedbackBody=matVecMul(JT,embedPlanar(feedbackEarth as [number,number,number])),wrench=add(model,feedbackBody) as [number,number,number,number,number,number];
    return{desired_wrench:wrench,eta_error:etaError,eta_dot:etaDot,eta_dot_r:reference,surface,integral_error:[...this.#integral],model_wrench:model,feedback_wrench:feedbackBody,uncommanded_dof_wrench:[wrench[2],wrench[3],wrench[4]],target_switch_transient:switched?"piecewise-constant target changed; controller state reset without differentiating the jump":"none"};
  }
}
