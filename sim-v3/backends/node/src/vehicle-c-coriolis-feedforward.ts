import {buildVehicleCProductionConfiguration} from "./vehicle-c-production.ts";
import {coriolisFromMass6,rotationBodyToNed,totalMassMatrix6} from "../../../core/sixDof.js";
import {matVecMul} from "../../../core/math.js";

export type VehicleCCoupledState={
  attitude_rad:[number,number,number];
  velocity_body_mps:[number,number,number];
  angular_rate_body_rad_s:[number,number,number];
};

const transposeMul=(matrix:number[][],vector:number[])=>matrix[0].map((_,column)=>matrix.reduce((sum,row,index)=>sum+row[column]*vector[index],0));

/** Vehicle-C-only pre-allocation feedforward using the production plant mass matrix. */
export class VehicleCCoriolisFeedforward {
  readonly massMatrix:number[][];
  constructor(){this.massMatrix=totalMassMatrix6(buildVehicleCProductionConfiguration().parameters);}

  compensation(state:VehicleCCoupledState,currentNedMps:[number,number,number]){
    const [roll,pitch,yaw]=state.attitude_rad,rotation=rotationBodyToNed({roll,pitch,yaw}),waterBody=transposeMul(rotation,currentNedMps),nu=[...state.velocity_body_mps,...state.angular_rate_body_rad_s],relativeNu=[nu[0]-waterBody[0],nu[1]-waterBody[1],nu[2]-waterBody[2],nu[3],nu[4],nu[5]],full=matVecMul(coriolisFromMass6(this.massMatrix,relativeNu),relativeNu);
    return {full,relative_velocity_body:relativeNu};
  }

  apply(desiredWrench:[number,number,number,number,number,number],state:VehicleCCoupledState,currentNedMps:[number,number,number]){
    const computed=this.compensation(state,currentNedMps),compensated=[...desiredWrench] as [number,number,number,number,number,number];
    // CoupledSixPlant adds -C(M,nu_r)nu_r, so pre-allocation cancellation is +C(M,nu_r)nu_r.
    compensated[0]+=computed.full[0];compensated[5]+=computed.full[5];
    return {desired_wrench:compensated,compensation_full:computed.full,reported_uncommanded_sway_compensation:computed.full[1],relative_velocity_body:computed.relative_velocity_body};
  }
}
