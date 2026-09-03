import {mkdirSync,readFileSync,writeFileSync} from "node:fs";
import {resolve} from "node:path";
import {fixedActionTrace} from "./frozen-conformance-trace.ts";
import {frozenReset,FULL_STEPS,PHYSICS_DT,prepareEpisode} from "./episode-driver.ts";
import {renderSurveyorVrxWorld} from "./prepare-vrx-surveyor.ts";
import {SURVEYOR_WIND_ESTIMATE} from "../../../packages/vehicle-sdk/src/surveyor-environment.js";

const CONTRACT_HASH="cc2c35cafee9eceb31cbb7e76522426cbabbcc78ef4176ba03a69bbdf420a1fb";
const VRX_CONFIGURATION="vrx:surveyor-patched";
export function prepareVrxEpisode(seed:number,out:string,steps=FULL_STEPS,environmentScale=1,actionScale=1,windScale=environmentScale,currentScale=environmentScale){
  const reset=frozenReset(seed),episode=prepareEpisode("VRX",seed,steps,actionScale);
  const [N,E,yawNed]=reset.initial_state;
  const yawEnu=Math.PI/2-yawNed;
  const world=renderSurveyorVrxWorld()
    .replace('<pose>0 0 0.05 0 0 0</pose>',`<pose>${E} ${N} 0.05 0 0 ${yawEnu}</pose>`);
  const root=resolve(out),worldPath=resolve(root,"world.sdf"),schedulePath=resolve(root,"transport.json"),modelDir=resolve(root,"models/surveyor");
  const currentAngle=reset.disturbance.current_direction_deg*Math.PI/180,currentN=currentScale*reset.disturbance.current_speed_m_s*Math.cos(currentAngle),currentE=currentScale*reset.disturbance.current_speed_m_s*Math.sin(currentAngle);
  const windAngle=reset.disturbance.wind_direction_deg*Math.PI/180,windN=windScale*reset.disturbance.wind_speed_m_s*Math.cos(windAngle),windE=windScale*reset.disturbance.wind_speed_m_s*Math.sin(windAngle);
  const currentPlugin=`<plugin filename="libVrxCurrentRelativeVelocity.so" name="vrx_surveyor::CurrentRelativeVelocity"><link_name>base_link</link_name><current_enu>${currentE} ${currentN} 0</current_enu><xU>6</xU><xUU>18</xUU><yV>18</yV><yVV>60</yVV></plugin>`;
  const windPlugin=`<plugin filename="libVrxSurveyorRelativeWind.so" name="vrx_surveyor::SurveyorRelativeWind"><link_name>base_link</link_name><wind_enu>${windE} ${windN} 0</wind_enu><air_density>${SURVEYOR_WIND_ESTIMATE.air_density_kg_m3}</air_density><frontal_area>${SURVEYOR_WIND_ESTIMATE.frontal_area_m2}</frontal_area><side_area>${SURVEYOR_WIND_ESTIMATE.side_area_m2}</side_area><drag_coefficient>${SURVEYOR_WIND_ESTIMATE.drag_coefficient}</drag_coefficient></plugin>`;
  const baseModel=readFileSync(resolve("artifacts/rl-campaign/vrx-surveyor-runtime/models/surveyor/model.sdf"),"utf8");
  mkdirSync(modelDir,{recursive:true});writeFileSync(resolve(modelDir,"model.sdf"),baseModel.replace("</model>",`${currentPlugin}${windPlugin}</model>`));
  writeFileSync(resolve(modelDir,"model.config"),'<?xml version="1.0"?><model><name>Surveyor</name><version>1.0</version><sdf version="1.10">model.sdf</sdf></model>\n');writeFileSync(worldPath,world);
  writeFileSync(schedulePath,JSON.stringify({schema_version:1,vrx_configuration:VRX_CONFIGURATION,purpose:"gate-7-conformance-diagnostics-only",policy_training_allowed:false,baseline_comparison_allowed:false,contract_content_sha256:CONTRACT_HASH,seed,environment_scale:environmentScale,wind_scale:windScale,current_scale:currentScale,action_scale:actionScale,physics_dt_s:PHYSICS_DT,control_dt_s:.1,physics_samples:steps,control_actions:steps/2,hold_samples_per_action:2,reset,action_trace:fixedActionTrace(steps),transport:episode.transport},null,2)+"\n");
  return{worldPath,schedulePath,reset};
}
if(process.argv[1]===import.meta.filename){const[seed,out,environmentScale="1",actionScale="1",windScale=environmentScale,currentScale=environmentScale]=process.argv.slice(2);if(!seed||!out)throw new Error("usage: prepare-vrx-episode.ts seed output-directory [environment-scale] [action-scale] [wind-scale] [current-scale]");console.log(JSON.stringify(prepareVrxEpisode(Number(seed),out,FULL_STEPS,Number(environmentScale),Number(actionScale),Number(windScale),Number(currentScale)),null,2));}
