export interface GuidanceDemand {a:number;w:number;desiredSpeed:number;target?:unknown}
export interface GuidanceActuatorCommand {surgeForce:number;differentialForce:number;desiredSpeed:number;target:unknown;hardware_command?:{mode:"set_thruster_mode";thrust:number;thrust_diff:number}}
export interface GuidanceActuatorMapper {map(guidance:GuidanceDemand):GuidanceActuatorCommand}
export interface SurveyorGuidanceConfig {max_thrust_command?:number;bootstrap_newtons_per_command?:number}

const HARDWARE_LIMIT=70;
function finite(value:number,label:string){if(!Number.isFinite(value))throw new Error(`${label} must be finite.`);return value;}
function clamp(value:number,limit:number){return Math.min(Math.max(value,-limit),limit);}

/**
 * Converts our waypoint follower's acceleration/yaw demand into the exact
 * integer command pair accepted by SeaRobotics set_thruster_mode().  The
 * Newton projection exists only to exercise the simulator and is explicitly
 * uncalibrated until FIU supplies measured command-to-bollard-thrust curves.
 */
export class SurveyorGuidanceMapper implements GuidanceActuatorMapper{
  readonly maxCommand:number;readonly bootstrapNewtonsPerCommand:number;
  constructor(config:SurveyorGuidanceConfig={}){const cap=config.max_thrust_command??HARDWARE_LIMIT;if(!Number.isInteger(cap)||cap<0||cap>HARDWARE_LIMIT)throw new Error("Surveyor max_thrust_command must be an integer in [0,70].");this.maxCommand=cap;this.bootstrapNewtonsPerCommand=finite(config.bootstrap_newtons_per_command??1,"Surveyor bootstrap_newtons_per_command");if(this.bootstrapNewtonsPerCommand<=0)throw new Error("Surveyor bootstrap_newtons_per_command must be positive.");}
  map(guidance:GuidanceDemand):GuidanceActuatorCommand{finite(guidance?.a,"Surveyor guidance acceleration");finite(guidance?.w,"Surveyor guidance yaw demand");finite(guidance?.desiredSpeed,"Surveyor guidance desired speed");const thrust=Math.round(clamp(guidance.a*HARDWARE_LIMIT,this.maxCommand)),thrust_diff=Math.round(clamp(guidance.w*HARDWARE_LIMIT,this.maxCommand));return{surgeForce:2*thrust*this.bootstrapNewtonsPerCommand,differentialForce:thrust_diff*this.bootstrapNewtonsPerCommand,desiredSpeed:guidance.desiredSpeed,target:guidance.target??null,hardware_command:{mode:"set_thruster_mode",thrust,thrust_diff}};}
}

export interface SurveyorWaypoint {lat:number;lon:number}
export interface SurveyorMission {type:"surveyor-waypoint";origin:{lat:number;lon:number;heading_deg?:number};waypoints:SurveyorWaypoint[];erp:SurveyorWaypoint;max_thrust_command?:number}
export function validateSurveyorMission(value:unknown):asserts value is SurveyorMission{const mission=value as SurveyorMission;if(mission?.type!=="surveyor-waypoint"||!mission.origin||!Array.isArray(mission.waypoints)||mission.waypoints.length===0||!mission.erp)throw new Error("Surveyor missions require origin, non-empty raw lat/lon waypoints, and ERP.");for(const [label,point] of [["origin",mission.origin],["ERP",mission.erp],...mission.waypoints.map((point,index)=>[`waypoint[${index}]`,point] as const)] as const){if(!Number.isFinite(point.lat)||point.lat< -90||point.lat>90||!Number.isFinite(point.lon)||point.lon< -180||point.lon>180)throw new Error(`Surveyor ${label} must contain valid finite latitude/longitude.`);}if(mission.max_thrust_command!==undefined&&(!Number.isInteger(mission.max_thrust_command)||mission.max_thrust_command<0||mission.max_thrust_command>70))throw new Error("Surveyor max_thrust_command must be an integer in [0,70].");}

export const SURVEYOR_PUBLIC_SPEC={
  id:"searobotics-surveyor-m1.8",length_m:1.83,beam_m:.91,draft_m:.17,mass_kg:52.3,thruster_count:2,thruster_electrical_power_w_each:1000,battery:{energy_wh:1500,nominal_voltage_v:24},
  provenance:"SeaRobotics SR-Surveyor M1.8 manufacturer specification sheet",
  calibration_status:"integration-only-unvalidated" as const,
  unresolved:["port/starboard thruster coordinates","forward/reverse bollard-thrust curves versus integer command and voltage","command deadband","motor time constant and slew rate","loaded CG/inertia","hydrodynamic damping"]
};
