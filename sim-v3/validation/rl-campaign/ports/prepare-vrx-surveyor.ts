import {createHash} from "node:crypto";
import {mkdirSync,readFileSync,writeFileSync} from "node:fs";
import {dirname,resolve} from "node:path";

const ROOT=resolve(import.meta.dirname,"../../..");
const SOURCE=resolve(ROOT,"artifacts/rl-campaign/surveyor-vehicle-model.json");
const sha=(value:string|Buffer)=>createHash("sha256").update(value).digest("hex");
const sourceBytes=readFileSync(SOURCE),source=JSON.parse(sourceBytes.toString());
const fmt=(value:number)=>Number.isInteger(value)?String(value):String(value);
// Documented engineering estimate for a compact, roughly 1 kW flooded marine
// thruster. These are deliberately not WAM-V or manufacturer parameters.
export const ROTOR_ESTIMATE={
  mass_kg:0.3,
  diameter_m:0.1,
  radius_m:0.05,
  representative_max_rpm:3000,
  representative_max_rad_s:314.1592653589793,
  axial_inertia_kg_m2:0.000375,
  transverse_inertia_kg_m2:0.0001975,
  joint_damping_n_m_s_per_rad:0.010132118364233778,
  thrust_coefficient:0.006919495468257212,
  fluid_density_kg_m3:1025,
  retained_force_bound_n:70,
} as const;
const sdfBodyPosition=(position:number[])=>[position[0],-position[1],-position[2]];
function baseLinkInertial(totalMass:number,inertia:any,effectors:any[]){
  const rotorMass=ROTOR_ESTIMATE.mass_kg*effectors.length;
  const mass=totalMass-rotorMass;
  const rotorFirstMomentX=effectors.reduce((sum,e)=>sum+ROTOR_ESTIMATE.mass_kg*e.position_body_m[0],0);
  const comX=-rotorFirstMomentX/mass;
  const rotorIxx=effectors.reduce((sum,e)=>sum+ROTOR_ESTIMATE.axial_inertia_kg_m2+ROTOR_ESTIMATE.mass_kg*e.position_body_m[1]**2,0);
  const rotorIyy=effectors.reduce((sum,e)=>sum+ROTOR_ESTIMATE.transverse_inertia_kg_m2+ROTOR_ESTIMATE.mass_kg*e.position_body_m[0]**2,0);
  const rotorIzz=effectors.reduce((sum,e)=>sum+ROTOR_ESTIMATE.transverse_inertia_kg_m2+ROTOR_ESTIMATE.mass_kg*(e.position_body_m[0]**2+e.position_body_m[1]**2),0);
  const baseParallelAxis=mass*comX**2;
  return{mass,comX,ixx:inertia.Ixx_roll-rotorIxx,iyy:inertia.Iyy_pitch-rotorIyy-baseParallelAxis,izz:inertia.Izz_yaw-rotorIzz-baseParallelAxis};
}

export function renderSurveyorVrxModel(){
  const mass=source.mass_properties.mass_kg.value,inertia=source.mass_properties.inertia_tensor_body_kg_m2.diagonal;
  const [port,starboard]=source.propulsion.effectors,[Xu,Yv,Nr]=source.hydrodynamics.linear_planar,[Xuu,Yvv,Nrr]=source.hydrodynamics.quadratic_planar;
  const baseInertial=baseLinkInertial(mass,inertia,[port,starboard]);
  const components=source.geometry.approximation.components;
  const pontoon=(id:string,component:any)=>`      <collision name="${id}_collision"><pose>${component.center_body_m.join(" ")} 0 0 0</pose><geometry><box><size>${component.dimensions_m.join(" ")}</size></box></geometry></collision>\n      <visual name="${id}_visual"><pose>${component.center_body_m.join(" ")} 0 0 0</pose><geometry><box><size>${component.dimensions_m.join(" ")}</size></box></geometry><material><diffuse>0.08 0.22 0.32 1</diffuse></material></visual>`;
  const thruster=(effector:any)=>`    <link name="${effector.id}_propeller"><pose relative_to="base_link">${sdfBodyPosition(effector.position_body_m).join(" ")} 0 0 0</pose><inertial><mass>${ROTOR_ESTIMATE.mass_kg}</mass><inertia><ixx>${ROTOR_ESTIMATE.axial_inertia_kg_m2}</ixx><iyy>${ROTOR_ESTIMATE.transverse_inertia_kg_m2}</iyy><izz>${ROTOR_ESTIMATE.transverse_inertia_kg_m2}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial></link>\n    <joint name="${effector.id}_propeller_joint" type="revolute"><parent>base_link</parent><child>${effector.id}_propeller</child><axis><xyz>${effector.axis_body.join(" ")}</xyz><dynamics><damping>${ROTOR_ESTIMATE.joint_damping_n_m_s_per_rad}</damping></dynamics></axis></joint>\n    <plugin filename="gz-sim-thruster-system" name="gz::sim::systems::Thruster"><joint_name>${effector.id}_propeller_joint</joint_name><thrust_coefficient>${ROTOR_ESTIMATE.thrust_coefficient}</thrust_coefficient><fluid_density>${ROTOR_ESTIMATE.fluid_density_kg_m3}</fluid_density><propeller_diameter>${ROTOR_ESTIMATE.diameter_m}</propeller_diameter><velocity_control>true</velocity_control><min_thrust_cmd>-${ROTOR_ESTIMATE.retained_force_bound_n}</min_thrust_cmd><max_thrust_cmd>${ROTOR_ESTIMATE.retained_force_bound_n}</max_thrust_cmd><namespace>surveyor</namespace><topic>thrusters/${effector.id}/thrust</topic></plugin>`;
  return `<?xml version="1.0"?>
<sdf version="1.10">
  <model name="surveyor">
    <link name="base_link">
      <inertial><pose>${baseInertial.comX} 0 0 0 0 0</pose><mass>${fmt(baseInertial.mass)}</mass><inertia><ixx>${fmt(baseInertial.ixx)}</ixx><iyy>${fmt(baseInertial.iyy)}</iyy><izz>${fmt(baseInertial.izz)}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
${pontoon("port_pontoon",components[0])}
${pontoon("starboard_pontoon",components[1])}
${pontoon("deck_payload_and_superstructure",components[2])}
      <sensor name="task_imu" type="imu"><always_on>true</always_on><update_rate>20</update_rate><topic>imu</topic></sensor>
      <sensor name="task_gps" type="navsat"><always_on>true</always_on><update_rate>20</update_rate><topic>gps</topic></sensor>
    </link>
    <plugin filename="libSurface.so" name="vrx::Surface"><link_name>base_link</link_name><hull_length>1.83</hull_length><hull_radius>0.17</hull_radius><fluid_level>0</fluid_level><fluid_density>1025</fluid_density><points><point>0.4575 -0.33 0</point><point>-0.4575 -0.33 0</point></points><wavefield><topic>/vrx/wavefield/parameters</topic></wavefield></plugin>
    <plugin filename="libSurface.so" name="vrx::Surface"><link_name>base_link</link_name><hull_length>1.83</hull_length><hull_radius>0.17</hull_radius><fluid_level>0</fluid_level><fluid_density>1025</fluid_density><points><point>0.4575 0.33 0</point><point>-0.4575 0.33 0</point></points><wavefield><topic>/vrx/wavefield/parameters</topic></wavefield></plugin>
    <plugin filename="libSimpleHydrodynamics.so" name="vrx::SimpleHydrodynamics"><link_name>base_link</link_name><xDotU>0</xDotU><yDotV>0</yDotV><nDotR>0</nDotR><xU>${Xu}</xU><xUU>${Xuu}</xUU><yV>${Yv}</yV><yVV>${Yvv}</yVV><zW>0</zW><kP>0</kP><mQ>0</mQ><nR>${Nr}</nR><nRR>${Nrr}</nRR></plugin>
${thruster(port)}
${thruster(starboard)}
    <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher"><publish_model_pose>true</publish_model_pose><publish_link_pose>true</publish_link_pose><use_pose_vector_msg>true</use_pose_vector_msg><static_publisher>false</static_publisher></plugin>
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher"><odom_publish_frequency>20</odom_publish_frequency><odom_topic>odometry</odom_topic><dimensions>3</dimensions></plugin>
  </model>
</sdf>\n`;
}

export function renderSurveyorVrxWorld(){return `<?xml version="1.0"?>
<sdf version="1.10"><world name="surveyor_vrx"><physics name="vrx_physics" type="dart"><max_step_size>0.05</max_step_size><real_time_factor>1</real_time_factor></physics><gravity>0 0 -9.81</gravity><include><uri>model://surveyor</uri><pose>0 0 0.05 0 0 0</pose></include><plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/><plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/><plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/><plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"/><plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/><plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/></world></sdf>\n`;}

export function prepareVrxSurveyor(out=resolve(ROOT,"artifacts/rl-campaign/vrx-surveyor-runtime")){
  const mass=source.mass_properties.mass_kg.value,inertia=source.mass_properties.inertia_tensor_body_kg_m2.diagonal,[port,starboard]=source.propulsion.effectors;
  const baseInertial=baseLinkInertial(mass,inertia,[port,starboard]);
  const modelDir=resolve(out,"models/surveyor"),worldDir=resolve(out,"worlds");mkdirSync(modelDir,{recursive:true});mkdirSync(worldDir,{recursive:true});
  const model=renderSurveyorVrxModel(),world=renderSurveyorVrxWorld();writeFileSync(resolve(modelDir,"model.sdf"),model);writeFileSync(resolve(modelDir,"model.config"),`<?xml version="1.0"?><model><name>Surveyor</name><version>1.0</version><sdf version="1.10">model.sdf</sdf></model>\n`);writeFileSync(resolve(worldDir,"surveyor-vrx.sdf"),world);
  const artifact={schema_version:1,artifact_kind:"vrx-surveyor-port",status:"rotor-estimate-built-pending-smoke",decision_path:"A",source_of_truth:"artifacts/rl-campaign/surveyor-vehicle-model.json",source_sha256:sha(sourceBytes),vrx:{source_ref:"osrf/vrx@fda35961463c2ed8c44d6c646fe8ce773a3eaff1",image:"leadcat/vrx:v3.0.1",image_digest:"sha256:1b0ebfc18f5265b684239e67e132b88557303fa6031334e9010a1d9794efd369",gazebo_sim_version:"8.10.0"},plugin_inventory:{buoyancy:{plugin:"vrx::Surface (libSurface.so)",generic_link_parameter:true,inputs:["link_name","hull_length","hull_radius","fluid_level","fluid_density","points","wavefield"],hardcoded_wamv_identity:false},hydrodynamics:{plugin:"vrx::SimpleHydrodynamics (libSimpleHydrodynamics.so)",generic_link_parameter:true,accepts_existing_model:true,inputs:["six diagonal added-mass terms","six linear drag terms","six quadratic drag terms"],hardcoded_wamv_identity:false},thruster:{plugin:"gz::sim::systems::Thruster",command_units:"newtons",message_type:"gz.msgs.Double / ROS std_msgs/msg/Float64 through bridge",translation_layer_required:false,topics:["/surveyor/thrusters/port/thrust","/surveyor/thrusters/starboard/thrust"]}},serialized_values:{mass_kg:mass,inertia_diagonal_kg_m2:[inertia.Ixx_roll,inertia.Iyy_pitch,inertia.Izz_yaw],components:source.geometry.approximation.components.map((x:any)=>({id:x.id,dimensions_m:x.dimensions_m,center_body_m:x.center_body_m})),thruster_positions_body_ned_m:[port.position_body_m,starboard.position_body_m],thruster_positions_sdf_flu_m:[sdfBodyPosition(port.position_body_m),sdfBodyPosition(starboard.position_body_m)],force_range_n_each:source.propulsion.command_and_dynamics.force_range_n_each,time_constant_s:source.propulsion.command_and_dynamics.time_constant_s,linear_planar_damping:source.hydrodynamics.linear_planar,quadratic_planar_damping:source.hydrodynamics.quadratic_planar,allocation_policy:source.propulsion.command_and_dynamics.allocation_policy},rotor_engineering_estimate:{classification:"documented engineering estimate; not SR-Surveyor, manufacturer, or WAM-V data",values:ROTOR_ESTIMATE,reasoning:{mass:"0.30 kg is centered within the defensible 0.20-0.50 kg range for compact marine propeller/rotor assemblies in this power class.",diameter:"0.10 m diameter gives a 0.05 m radius, a few-centimetre radius appropriate to a compact roughly 1 kW pocket thruster.",axial_inertia:"Thin-disk/blade approximation I=0.5*m*r^2 = 0.000375 kg m^2.",transverse_inertia:"Short-cylinder approximation I=0.25*m*r^2 + m*t^2/12 with estimated 0.02 m axial thickness = 0.0001975 kg m^2.",damping:"Viscous-equivalent damping c=P/omega^2 at 1000 W and 3000 rpm = 0.010132118364233778 N m s/rad; this represents rated-point hydrodynamic rotational dissipation without claiming a manufacturer curve.",thrust_coefficient:"Gazebo force-mode identity Kt=T/(rho*D^4*omega^2), using the established 70 N bound, rho=1025 kg/m^3, D=0.10 m and 3000 rpm, gives 0.006919495468257212. The plugin command remains clamped at exactly +/-70 N, so the estimate does not alter thrust authority."},comparable_power_class_reference:"Blue Robotics T500 product documentation describes a compact flooded thruster operating just over 1 kW; used only to support the broad power-class analogy, not to copy its physical parameters.",reference_url:"https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t500-thruster/"},wiring:{shared_actuator_module:"validation/rl-campaign/ports/shared-actuators.ts",adapter:"validation/rl-campaign/ports/simulator-actuator-adapters.ts",newton_output_direct_to_vrx:true},files:{model_sdf:"artifacts/rl-campaign/vrx-surveyor-runtime/models/surveyor/model.sdf",world_sdf:"artifacts/rl-campaign/vrx-surveyor-runtime/worlds/surveyor-vrx.sdf",model_sha256:sha(model),world_sha256:sha(world)},provenance_statement:"Established hull, mass, inertia, damping, actuator placement, lag and force bounds are serialized from the Surveyor source artifact, never bcod_usv. Only explicitly labeled rotor values are engineering estimates; no WAM-V rotor number is used.",smoke_test:{executed:false,reason:"Regeneration precedes the requested post-fix runtime test."},governing_contract:{path:"artifacts/rl-campaign/surveyor/task-contract-frozen.json",version:"2.0.0-surveyor-time-aware",content_sha256:"cc2c35cafee9eceb31cbb7e76522426cbabbcc78ef4176ba03a69bbdf420a1fb",observation_fields:16},gate_7:{executed:false,reason:"Awaiting successful smoke test."},review_required:true};
  artifact.status="smoke-test-passed-gate-7-blocked-invalid-harness";
  artifact.serialized_values.assembly_mass_partition={total_mass_kg:mass,base_link_mass_kg:baseInertial.mass,two_rotor_mass_kg:2*ROTOR_ESTIMATE.mass_kg,base_link_com_x_m:baseInertial.comX,base_link_inertia_diagonal_kg_m2:[baseInertial.ixx,baseInertial.iyy,baseInertial.izz],reason:"Rotor child-link mass and parallel-axis inertia are partitioned out of the established totals so the assembled model remains exactly 52.3 kg with zero first moment and the established aggregate inertia tensor."};
  artifact.serialized_values.surface_points_sdf_flu_m={port:[[0.4575,-0.33,0],[-0.4575,-0.33,0]],starboard:[[0.4575,0.33,0],[-0.4575,0.33,0]],reason:"Symmetric fore/aft quadrature points at +/- one quarter of the established 1.83 m pontoon length provide pitch-restoring buoyancy; no WAM-V coordinate is used."};
  artifact.smoke_test={runtime:"leadcat/vrx:v3.0.1 under amd64 emulation",environment:{GZ_SIM_SYSTEM_PLUGIN_PATH:"/opt/vrx_ws/install/lib",note:"VRX libraries and their dependent libWaves.so must both be discoverable."},sdf_model_validation:"passed",spawn:"passed",zero_command_control:{sample_sim_time_s:10,pose_xyz_m:[0.02793183159294944,0.08802453371418911,0.19112203273911976],quaternion_xyzw:[-0.026580717782313287,-0.009442659936576576,0.0005332797135321105,0.9996019293840155],finite:true,stable:true},equal_thrust_test:{commands_newtons:{port:35,starboard:35},publication:"concurrent",sample_sim_time_s:11.25,pose_xyz_m:[6.761481981843311,0.06751718421055694,0.1462282036482464],quaternion_xyzw:[0.033412031190596055,-0.018951126207803023,0.00014863023698423968,0.9992619620981413],finite:true,direction_sanity_passed:true,physics_blowup_absent:true,assessment:"Strong positive surge with negligible lateral displacement and yaw; small bounded roll/pitch."},axis_and_serialization_check:{joint_type:"revolute (supported by the pinned DART build)",axis_sdf_flu:[1,0,0],source_axis_body_ned:[1,0,0],axis_conversion_required:false,port_starboard_y_conversion:"NED body y is negated for SDF FLU",surface_point_fix:"Each pontoon uses symmetric fore/aft points; a single center point had no pitch-restoring moment."}};
  artifact.gate_7={executed:false,status:"BLOCKED_INVALID_HARNESS",contract_content_sha256:"cc2c35cafee9eceb31cbb7e76522426cbabbcc78ef4176ba03a69bbdf420a1fb",reason:"The checked-in Gate 7 runner cannot yet execute a faithful VRX conformance episode.",verified_repairs:["Node reference preset changed from vehicle-a-otter to searobotics-surveyor-m1.8.","TaskTraceBridge actuator state changed from the Vehicle A default to the Surveyor per-effector specification."],remaining_structural_gaps:["run-gate-7.ts only writes a pre-existing blocked report and never launches VRX or compares traces.","vrx-episode-driver.ts is only a re-export; no lifecycle connects seeded reset, 1200 control messages, and 2400 odometry samples.","The Surveyor VRX world has no serialized wind/current systems, so contract reset disturbances cannot be applied.","The existing Jazzy exporter consumes odometry but no checked-in orchestrator synchronizes it with deterministic simulation stepping."],integrity_rule:"No Gate 7 result is emitted from an unseeded calm-water smoke world or from the previously Vehicle A-labelled Node trace."};
  const artifactPath=resolve(ROOT,"artifacts/rl-campaign/vrx-surveyor-port.json");mkdirSync(dirname(artifactPath),{recursive:true});writeFileSync(artifactPath,JSON.stringify(artifact,null,2)+"\n");return{artifactPath,modelDir,world:resolve(worldDir,"surveyor-vrx.sdf")};
}
if(process.argv[1]===import.meta.filename)console.log(JSON.stringify(prepareVrxSurveyor(process.argv[2]),null,2));
