import {ForceModel} from "./force-model.js";
import {addedMassCoriolis3,rigidBodyCoriolis3} from "./coriolis.ts";
import {dampingWrench3} from "./damping.ts";
function matVecMul(matrix,vector){return matrix.map((row)=>row.reduce((sum,value,idx)=>sum+value*vector[idx],0));}
function assertSkewSymmetric(matrix,tolerance=1e-9){for(let r=0;r<matrix.length;r+=1)for(let c=0;c<matrix.length;c+=1)if(Math.abs(matrix[r][c]+matrix[c][r])>tolerance)return false;return true;}
export class AddedMassCoriolis extends ForceModel{computeWrench(ctx){const cRb=rigidBodyCoriolis3(ctx.params,ctx.velocityVector),cA=addedMassCoriolis3(ctx.params,ctx.relativeVelocityVector),c=cRb.map((row,r)=>row.map((value,col)=>value+cA[r][col]));if(!assertSkewSymmetric(c,1e-8))throw new Error("Coriolis matrix lost skew symmetry.");return matVecMul(c,ctx.relativeVelocityVector).map((value)=>-value);}}
export class HydrodynamicDamping extends ForceModel{computeWrench(ctx){return dampingWrench3(ctx.params,ctx.relativeVelocityVector);}}
