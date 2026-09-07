/**
 * Minimum-thrust-norm allocation for independently steerable planar thrusters.
 *
 * The decision variables are each pod's body-frame (Fx,Fy) components.  The
 * Moore-Penrose solution B^T(BB^T)^-1 tau minimizes sum(Fx_i^2+Fy_i^2), which
 * is also the sum of squared pod thrust magnitudes.  A common scale factor is
 * applied if a pod limit is exceeded so the requested wrench direction is
 * preserved.  Full-circle azimuths make negative thrust unnecessary.
 */

const finite=(value,label)=>{if(!Number.isFinite(value))throw new Error(`${label} must be finite.`);return value};
const wrapAngle=(value)=>Math.atan2(Math.sin(value),Math.cos(value));
const angleDistance=(a,b)=>Math.abs(wrapAngle(a-b));

function solve3(matrix,vector){
  const a=matrix.map((row,index)=>[...row,vector[index]]);
  for(let column=0;column<3;column++){
    let pivot=column;for(let row=column+1;row<3;row++)if(Math.abs(a[row][column])>Math.abs(a[pivot][column]))pivot=row;
    if(Math.abs(a[pivot][column])<1e-12)throw new Error("Azimuth allocation geometry is rank deficient.");
    [a[column],a[pivot]]=[a[pivot],a[column]];
    const divisor=a[column][column];for(let j=column;j<4;j++)a[column][j]/=divisor;
    for(let row=0;row<3;row++)if(row!==column){const factor=a[row][column];for(let j=column;j<4;j++)a[row][j]-=factor*a[column][j];}
  }
  return a.map(row=>row[3]);
}

export function allocatePlanarAzimuthMinimumNorm(wrench,placements,options={}){
  if(!Array.isArray(wrench)||wrench.length!==3)throw new Error("Planar azimuth wrench must be [surge, sway, yaw].");
  if(!Array.isArray(placements)||placements.length<2)throw new Error("At least two azimuth pod placements are required.");
  const target=wrench.map((value,index)=>finite(value,`wrench[${index}]`));
  const columns=placements.flatMap((placement,index)=>{
    const x=finite(placement.pos[0],`placements[${index}].x`),y=finite(placement.pos[1],`placements[${index}].y`);
    return [[1,0,-y],[0,1,x]];
  });
  const gram=Array.from({length:3},(_,row)=>Array.from({length:3},(_,column)=>columns.reduce((sum,item)=>sum+item[row]*item[column],0)));
  const dual=solve3(gram.map((row,index)=>row.map((value,column)=>value+(index===column?(options.regularization??0):0))),target);
  const components=columns.map(column=>column.reduce((sum,value,index)=>sum+value*dual[index],0));
  const raw=placements.map((placement,index)=>({id:placement.id,fx:components[2*index],fy:components[2*index+1]}));
  const maxRatio=raw.reduce((largest,pod,index)=>Math.max(largest,Math.hypot(pod.fx,pod.fy)/finite(placements[index].maxThrust??Infinity,`placements[${index}].maxThrust`)),1);
  const scale=1/maxRatio;
  const continuityWeight=Math.max(0,options.angleContinuityWeight??0);
  const pods=raw.map((pod,index)=>{
    const fx=pod.fx*scale,fy=pod.fy*scale,magnitude=Math.hypot(fx,fy),forwardAzimuth=Math.atan2(fy,fx);
    let thrust=magnitude,azimuth=forwardAzimuth,representation="forward";
    if(continuityWeight>0&&magnitude>1e-9&&Array.isArray(options.currentAzimuths)){
      const current=finite(options.currentAzimuths[index],`currentAzimuths[${index}]`),reverseLimit=finite(placements[index].maxReverseThrust??0,`placements[${index}].maxReverseThrust`);
      const reverseAzimuth=wrapAngle(forwardAzimuth+Math.PI),forwardCost=magnitude*magnitude+continuityWeight*magnitude*magnitude*angleDistance(forwardAzimuth,current)**2;
      const reverseCost=magnitude*magnitude+continuityWeight*magnitude*magnitude*angleDistance(reverseAzimuth,current)**2;
      if(magnitude<=reverseLimit&&reverseCost<forwardCost){thrust=-magnitude;azimuth=reverseAzimuth;representation="reverse";}
    }
    return{id:pod.id,thrust,azimuth,representation,force_body_n:[fx,fy]};
  });
  const achieved=pods.reduce((sum,pod,index)=>{const [x,y]=placements[index].pos,[fx,fy]=pod.force_body_n;return[sum[0]+fx,sum[1]+fy,sum[2]+x*fy-y*fx]},[0,0,0]);
  return{strategy:continuityWeight>0?"minimum-thrust-norm-angle-continuity":"minimum-thrust-norm",requested_wrench:[...target],achieved_wrench:achieved,saturation_scale:scale,pods};
}
