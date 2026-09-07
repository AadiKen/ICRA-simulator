import {mkdirSync,writeFileSync} from "node:fs";
import {resolve} from "node:path";
import {fixedActionTrace} from "./frozen-conformance-trace.ts";
import {frozenReset,FULL_STEPS,PHYSICS_DT,prepareEpisode} from "./episode-driver.ts";
import {renderSurveyorVrxModel,renderSurveyorVrxWorld} from "./prepare-vrx-surveyor.ts";
import {SURVEYOR_WIND_ESTIMATE} from "../../../packages/vehicle-sdk/src/surveyor-environment.js";
import {SURVEYOR_PUBLIC_SPEC} from "../../../packages/vehicle-sdk/src/surveyor.ts";
import {loadFrozenTaskContract} from "./frozen-task-contract.ts";

const VRX_CONFIGURATION="vrx:surveyor-patched";
export function prepareVrxEpisode(seed:number,out:string,steps=FULL_STEPS,environmentScale=1,actionScale=1,windScale=environmentScale,currentScale=environmentScale,collisionMode="none",surfaceMode="both"){
  const CONTRACT_HASH=loadFrozenTaskContract().contentSha256;
  const reset=frozenReset(seed),episode=prepareEpisode("VRX",seed,steps,actionScale);
  const [N,E,yawNed]=reset.initial_state;
  const yawEnu=Math.PI/2-yawNed;
  const obstacle=collisionMode==="none"?"":(()=>{const ground=collisionMode==="grounding",forward=ground?1.15:.7,lateral=ground?0:-.33,x=E+forward*Math.cos(yawEnu)-lateral*Math.sin(yawEnu),y=N+forward*Math.sin(yawEnu)+lateral*Math.cos(yawEnu);return `<model name="${ground?"bathymetry_seabed":"test_buoy"}"><static>true</static><pose>${x} ${y} ${ground?-0.24:0.25} 0 0 0</pose><link name="body"><collision name="collision"><geometry><box><size>${ground?"1.5 1.5 0.3":"0.35 0.35 0.8"}</size></box></geometry><surface><contact><collide_bitmask>65535</collide_bitmask></contact></surface></collision><visual name="visual"><geometry><box><size>${ground?"1.5 1.5 0.3":"0.35 0.35 0.8"}</size></box></geometry></visual></link></model>`;})();
  const world=renderSurveyorVrxWorld()
    .replace('<pose>0 0 0.05 0 0 0</pose>',`<pose>${E} ${N} 0.05 0 0 ${yawEnu}</pose>`)
    .replace('</world>',`${obstacle}<spherical_coordinates><surface_model>EARTH_WGS84</surface_model><world_frame_orientation>ENU</world_frame_orientation><latitude_deg>-33.72276876888639</latitude_deg><longitude_deg>150.67399110174387</longitude_deg><elevation>0</elevation><heading_deg>0</heading_deg></spherical_coordinates></world>`);
  const root=resolve(out),worldPath=resolve(root,"world.sdf"),schedulePath=resolve(root,"transport.json"),modelDir=resolve(root,"models/surveyor");
  const currentAngle=reset.disturbance.current_direction_deg*Math.PI/180,currentN=currentScale*reset.disturbance.current_speed_m_s*Math.cos(currentAngle),currentE=currentScale*reset.disturbance.current_speed_m_s*Math.sin(currentAngle);
  const windAngle=reset.disturbance.wind_direction_deg*Math.PI/180,windN=windScale*reset.disturbance.wind_speed_m_s*Math.cos(windAngle),windE=windScale*reset.disturbance.wind_speed_m_s*Math.sin(windAngle);
  const currentPlugin=`<plugin filename="libVrxCurrentRelativeVelocity.so" name="vrx_surveyor::CurrentRelativeVelocity"><link_name>base_link</link_name><current_enu>${currentE} ${currentN} 0</current_enu><xU>6</xU><xUU>18</xUU><yV>18</yV><yVV>60</yVV><xDotU>${-0.05*SURVEYOR_PUBLIC_SPEC.mass_kg}</xDotU><yDotV>${-0.75*SURVEYOR_PUBLIC_SPEC.mass_kg}</yDotV></plugin>`;
  const windPlugin=`<plugin filename="libVrxSurveyorRelativeWind.so" name="vrx_surveyor::SurveyorRelativeWind"><link_name>base_link</link_name><wind_enu>${windE} ${windN} 0</wind_enu><air_density>${SURVEYOR_WIND_ESTIMATE.air_density_kg_m3}</air_density><frontal_area>${SURVEYOR_WIND_ESTIMATE.frontal_area_m2}</frontal_area><side_area>${SURVEYOR_WIND_ESTIMATE.side_area_m2}</side_area><drag_coefficient>${SURVEYOR_WIND_ESTIMATE.drag_coefficient}</drag_coefficient></plugin>`;
  // Generate from the current source instead of copying a possibly stale
  // checked-in artifact (the stale copy still configured GPS at 20 Hz).
  const generatedModel=renderSurveyorVrxModel();
  const surfacePattern=/\s*<plugin filename="libSurface\.so"[\s\S]*?<\/plugin>/g;
  const surfaceBlocks=[...generatedModel.matchAll(surfacePattern)].map(match=>match[0]);
  if(surfaceBlocks.length!==2)throw new Error(`expected two Surface plugins, found ${surfaceBlocks.length}`);
  let surfaceIndex=0;
  const baseModel=generatedModel.replace(surfacePattern,block=>{
    const keep=surfaceMode==="both"||(surfaceMode==="port"&&surfaceIndex===0)||(surfaceMode==="starboard"&&surfaceIndex===1);
    surfaceIndex+=1;return keep?block:"";
  });
  mkdirSync(modelDir,{recursive:true});writeFileSync(resolve(modelDir,"model.sdf"),baseModel.replace("</model>",`${currentPlugin}${windPlugin}</model>`));
  writeFileSync(resolve(modelDir,"model.config"),'<?xml version="1.0"?><model><name>Surveyor</name><version>1.0</version><sdf version="1.10">model.sdf</sdf></model>\n');writeFileSync(worldPath,world);
  writeFileSync(schedulePath,JSON.stringify({schema_version:1,vrx_configuration:VRX_CONFIGURATION,purpose:"gate-7-conformance-diagnostics-only",policy_training_allowed:false,baseline_comparison_allowed:false,contract_content_sha256:CONTRACT_HASH,seed,environment_scale:environmentScale,wind_scale:windScale,current_scale:currentScale,action_scale:actionScale,physics_dt_s:PHYSICS_DT,control_dt_s:.1,physics_samples:steps,control_actions:steps/2,hold_samples_per_action:2,reset,action_trace:fixedActionTrace(steps),transport:episode.transport},null,2)+"\n");
  return{worldPath,schedulePath,reset};
}
if(process.argv[1]===import.meta.filename){const[seed,out,environmentScale="1",actionScale="1",windScale=environmentScale,currentScale=environmentScale,collisionMode="none",surfaceMode="both"]=process.argv.slice(2);if(!seed||!out)throw new Error("usage: prepare-vrx-episode.ts seed output-directory [environment-scale] [action-scale] [wind-scale] [current-scale] [collision-mode] [surface-mode]");console.log(JSON.stringify(prepareVrxEpisode(Number(seed),out,FULL_STEPS,Number(environmentScale),Number(actionScale),Number(windScale),Number(currentScale),collisionMode,surfaceMode),null,2));}
