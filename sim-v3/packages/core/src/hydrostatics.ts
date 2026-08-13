import type {MassParameters} from "./mass.ts";

type HydrostaticParameters=MassParameters&{buoyancy?:{rho?:number;g?:number};restoring?:{waterDensity?:number;gravity?:number;waterplaneArea?:number;displacementVolume?:number;metacentricHeightRoll?:number;metacentricHeightPitch?:number;hydrostaticStiffnessMatrix6?:number[][];cob?:{x?:number;y?:number;z?:number}};geometry?:{waterplaneArea?:number;length?:number;beam?:number}};
type Attitude={roll?:number;pitch?:number};

export function linearHydrostaticWrench(params:HydrostaticParameters,state:{position:{D:number};eulerAngles:Attitude},equilibriumD=0):number[]{
  const resolved=params.restoring?.hydrostaticStiffnessMatrix6;if(resolved){if(resolved.length!==6||resolved.some((row)=>row.length!==6||row.some((value)=>!Number.isFinite(value))))throw new Error("Resolved hydrostatic stiffness must be a finite 6x6 matrix.");const displacement=[0,0,state.position.D-equilibriumD,state.eulerAngles.roll||0,state.eulerAngles.pitch||0,0];return resolved.map((row)=>-row.reduce((sum,value,index)=>sum+value*displacement[index],0));}
  const e=state.eulerAngles,rho=params.buoyancy?.rho||params.restoring?.waterDensity||1025,g=params.buoyancy?.g||params.restoring?.gravity||9.81;
  const area=params.restoring?.waterplaneArea??params.geometry?.waterplaneArea??(params.geometry?.length||0)*(params.geometry?.beam||0);
  const volume=params.restoring?.displacementVolume??params.massProps!.mass!/rho;
  const gmRoll=params.restoring?.metacentricHeightRoll??Math.max((params.geometry?.beam||1)*0.12,0.05),gmPitch=params.restoring?.metacentricHeightPitch??Math.max((params.geometry?.length||1)*0.12,0.05);
  return [0,0,-rho*g*area*(state.position.D-equilibriumD),-rho*g*volume*gmRoll*(e.roll||0),-rho*g*volume*gmPitch*(e.pitch||0),0];
}

export function restoringWrench6(params:HydrostaticParameters,attitude:Attitude,submergedState?:{totalVolume?:number;cobBody?:number[]}):number[]{
  const mass=params.massProps?.mass||0,g=params.buoyancy?.g||params.restoring?.gravity||9.81,rho=params.buoyancy?.rho||params.restoring?.waterDensity||1025;
  const volume=submergedState?.totalVolume??params.restoring?.displacementVolume??mass/rho,cg=params.massProps?.cg||{x:0,y:0,z:0},cob=submergedState?.cobBody||[params.restoring?.cob?.x||0,params.restoring?.cob?.y||0,params.restoring?.cob?.z||0];
  const xg=cg.x||0,yg=cg.y||0,zg=cg.z||0,xb=cob[0]||0,yb=cob[1]||0,zb=cob[2]||0,phi=attitude.roll||0,theta=attitude.pitch||0;
  const sPhi=Math.sin(phi),cPhi=Math.cos(phi),sTheta=Math.sin(theta),cTheta=Math.cos(theta),weight=mass*g,buoyancy=rho*g*volume;
  return [(weight-buoyancy)*sTheta,-(weight-buoyancy)*cTheta*sPhi,-(weight-buoyancy)*cTheta*cPhi,-(yg*weight-yb*buoyancy)*cTheta*cPhi+(zg*weight-zb*buoyancy)*cTheta*sPhi,(zg*weight-zb*buoyancy)*sTheta+(xg*weight-xb*buoyancy)*cTheta*cPhi,-(xg*weight-xb*buoyancy)*cTheta*sPhi-(yg*weight-yb*buoyancy)*sTheta];
}
