import {readFileSync} from "node:fs";
import {resolve} from "node:path";

const source=JSON.parse(readFileSync(resolve("artifacts/rl-campaign/surveyor-vehicle-model.json"),"utf8"));
const effectors=source.propulsion.effectors;
const dynamics=source.propulsion.command_and_dynamics;
const [Xu,Yv,Nr]=source.hydrodynamics.linear_planar;
const [Xuu,Yvv,Nrr]=source.hydrodynamics.quadratic_planar;

/** Re-derive the tensor from the declared component geometry animplement
d masses.
 * The source artifact records the same derivation, but the Gazebo port does
 * not trust/copy a bootstrap diagonal. */
function geometryDerivedInertia(){
 let Ix=0,Iy=0,Iz=0;
 const cgZ=-.05;
 for(const component of source.geometry.approximation.components){
  const m=component.mass_kg,[length,beam,height]=component.dimensions_m,[x,y,z]=component.center_body_m;
  const zFromCg=z-cgZ;
  Ix+=m*(beam*beam+height*height)/12+m*(y*y+zFromCg*zFromCg);
  Iy+=m*(length*length+height*height)/12+m*(x*x+zFromCg*zFromCg);
  Iz+=m*(length*length+beam*beam)/12+m*(x*x+y*y);
 }
 if(!(Ix>0&&Iy>0&&Iz>0&&Ix+Iy>=Iz&&Ix+Iz>=Iy&&Iy+Iz>=Ix))throw new Error("Surveyor geometry-derived inertia violates the rigid-body triangle inequality");
 return {Ix,Iy,Iz};
}
const inertia=geometryDerivedInertia();

/** Exact serialization of the Surveyor source artifact for stock Gazebo systems. */
export const surveyorGazeboCoefficients={
 id:"surveyor",
 geometry:{length:source.geometry.published_envelope.length_m,beam:source.geometry.published_envelope.beam_m,draft:source.geometry.published_envelope.draft_m,height:.34},
 // Two generated 1 g thruster links bring the assembled model to 52.3 kg.
 massProps:{mass:source.mass_properties.mass_kg.value-.002,cg:{x:0,y:0,z:-.05},inertia:{Ix:inertia.Ix-2e-6,Iy:inertia.Iy-2e-6,Iz:inertia.Iz-2e-6}},
 addedMass:{XuDot:0,YvDot:0,ZwDot:0,KpDot:0,MqDot:0,NrDot:0},
 // Planar values remain source-authored. Unmeasured out-of-plane terms are
 // explicit stability-fixture estimates near 70% critical damping from the
 // geometry-derived mass/inertia and hydrostatic waterplane stiffness.
 damping:{linear:{Xu:-Xu,Yv:-Yv,Zw:-1200,Kp:-100,Mq:-250,Nr:-Nr},quadratic:{Xuu:-Xuu,Yvv:-Yvv,Zww:-200,Kpp:-20,Mqq:-50,Nrr:-Nrr}},
 restoring:{waterDensity:source.geometry.displacement.water_density_kg_m3,gravity:9.81,displacementVolume:source.geometry.displacement.required_displaced_volume_m3},
 buoyancy:{rho:source.geometry.displacement.water_density_kg_m3,g:9.81,mode:"graded-surface"},
 actuator:{beam:source.geometry.published_envelope.beam_m,maxThrust:dynamics.force_range_n_each[1],motorTimeConstant:dynamics.time_constant_s},
 effectors:effectors.map((item:any)=>({id:item.id,type:"FixedThruster",pos:item.position_body_m,axis:item.axis_body,dynamics:{tau:dynamics.time_constant_s,min:dynamics.force_range_n_each[0],max:dynamics.force_range_n_each[1]},conversion:{type:"linear"}})),
 hullPrimitives:source.geometry.approximation.components.map((item:any)=>({type:"box",dims:{length:item.dimensions_m[0],beam:item.dimensions_m[1],height:item.dimensions_m[2]},offset:{pos:item.center_body_m,rot:[0,0,0]}})),
 provenance:{source:"artifacts/rl-campaign/surveyor-vehicle-model.json",source_status:source.status,inertiaMethod:"geometry-derived component assembly",outOfPlaneDamping:"Gazebo stability fixture; unmeasured engineering estimate"},
};
