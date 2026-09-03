import type {FrozenAction} from "./shared-actuators.ts";
import {FrozenActuatorBank,resolveSurveyorActuatorSpec} from "./shared-actuators.ts";

export interface TransportCommand {topic:string;message_type:"gz.msgs.Double"|"std_msgs/msg/Float64";value:number;}

/** One lag/allocation implementation serves both simulator transports. */
export class GazeboThrusterAdapter {
  readonly bank=new FrozenActuatorBank();
  apply(action:FrozenAction,dt_s=.05):TransportCommand[] {this.bank.step(action,dt_s);const [port,starboard]=this.bank.thrustNewtons();return [{topic:"/model/bcod_usv/joint/port_joint/cmd_thrust",message_type:"gz.msgs.Double",value:port},{topic:"/model/bcod_usv/joint/starboard_joint/cmd_thrust",message_type:"gz.msgs.Double",value:starboard}];}
}

export class VrxWamvThrusterAdapter {
  readonly bank=new FrozenActuatorBank(resolveSurveyorActuatorSpec());
  /** VRX accepts Newton Float64 thrust; lag and bounds remain in the shared module. */
  apply(action:FrozenAction,dt_s=.05):TransportCommand[] {this.bank.step(action,dt_s);const [port,starboard]=this.bank.thrustNewtons();return [{topic:"/surveyor/thrusters/port/thrust",message_type:"std_msgs/msg/Float64",value:port},{topic:"/surveyor/thrusters/starboard/thrust",message_type:"std_msgs/msg/Float64",value:starboard}];}
}
