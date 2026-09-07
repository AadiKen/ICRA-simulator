import {buildVehicleCProductionConfiguration} from "./vehicle-c-production.ts";
import {coriolisFromMass6,rotationBodyToNed,totalMassMatrix6} from "../../../core/sixDof.js";
import {matVecMul} from "../../../core/math.js";

export const VEHICLE_C_CORIOLIS_COMPENSATION_SCALE=0.5;

export type VehicleCCoupledState={
  attitude_rad:[number,number,number];
  velocity_body_mps:[number,number,number];
  angular_rate_body_rad_s:[number,number,number];
};

const transposeMul=(matrix:number[][],vector:number[])=>matrix[0].map((_,column)=>matrix.reduce((sum,row,index)=>sum+row[column]*vector[index],0));

/** Vehicle-C-only pre-allocation feedforward using the production plant mass matrix. */
export class VehicleCCoriolisFeedforward {
  readonly massMatrix:number[][];
  readonly scale:number;
  constructor(scale=VEHICLE_C_CORIOLIS_COMPENSATION_SCALE){this.scale=scale;this.massMatrix=totalMassMatrix6(buildVehicleCProductionConfiguration().parameters);}

  compensation(state:VehicleCCoupledState,currentNedMps:[number,number,number]){
    const [roll,pitch,yaw]=state.attitude_rad,rotation=rotationBodyToNed({roll,pitch,yaw}),waterBody=transposeMul(rotation,currentNedMps),nu=[...state.velocity_body_mps,...state.angular_rate_body_rad_s],relativeNu=[nu[0]-waterBody[0],nu[1]-waterBody[1],nu[2]-waterBody[2],nu[3],nu[4],nu[5]],full=matVecMul(coriolisFromMass6(this.massMatrix,relativeNu),relativeNu);
    return {full,relative_velocity_body:relativeNu};
  }

  apply(desiredWrench:[number,number,number,number,number,number],state:VehicleCCoupledState,currentNedMps:[number,number,number]){
    const computed=this.compensation(state,currentNedMps),scaled=computed.full.map(value=>this.scale*value),compensated=[...desiredWrench] as [number,number,number,number,number,number];
    // CoupledSixPlant adds -C(M,nu_r)nu_r, so pre-allocation cancellation is +C(M,nu_r)nu_r.
    compensated[0]+=scaled[0];compensated[5]+=scaled[5];
    return {desired_wrench:compensated,compensation_full:scaled,raw_coriolis_full:computed.full,reported_uncommanded_sway_compensation:scaled[1],relative_velocity_body:computed.relative_velocity_body};
  }
}
