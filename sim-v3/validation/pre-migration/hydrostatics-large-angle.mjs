import {RigidBodyState} from "../../core/rigidBodyState.js";
import {VehicleParameters} from "../../core/vehicleParameters.js";
import {linearHydrostaticWrench} from "../../core/coupledSixPlant.js";
import {restoringWrench6} from "../../core/sixDof.js";

const params=VehicleParameters.fromGeometry(2,1,0.2,50,{height:0.4,maxThrust:100,metacentricHeightRoll:0.12,metacentricHeightPitch:0.24});
const roll=25*Math.PI/180,pitch=-20*Math.PI/180;
const state=RigidBodyState.fromEuler({N:0,E:0,D:0.12},roll,pitch,0);
const linear=linearHydrostaticWrench(params,state,0);
const nonlinear=restoringWrench6(params,state.eulerAngles,{totalVolume:params.restoring.displacementVolume,cobBody:[params.restoring.cob.x,params.restoring.cob.y,params.restoring.cob.z]});
console.log(JSON.stringify({schema_version:1,case:"25deg-roll-minus20deg-pitch",model_characterization:{coupled6:"constant linear stiffness using GM_T and GM_L",sampled_geometry_path:"nonlinear displaced-volume and center-of-buoyancy recomputation when submergedState is supplied"},inputs:{roll_rad:roll,pitch_rad:pitch,heave_displacement_m:0.12,gm_transverse_m:params.restoring.metacentricHeightRoll,gm_longitudinal_m:params.restoring.metacentricHeightPitch},outputs:{linear_coupled6_wrench:linear,nonlinear_restoring_wrench:nonlinear}},null,2));
