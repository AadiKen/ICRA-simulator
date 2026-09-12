import {readFileSync,writeFileSync} from "node:fs";
import {resolve} from "node:path";
import {prepareVrxEpisode} from "./prepare-vrx-episode.ts";

const [outArg,stepArg,zWArg="0",kPArg="0",mQArg="0",samplesArg="1200"] = process.argv.slice(2);
if(!outArg||!stepArg)throw new Error("usage: prepare-vrx-damping-diagnostic.ts out max_step_size [zW kP mQ samples]");
const out=resolve(outArg),maxStep=Number(stepArg),zW=Number(zWArg),kP=Number(kPArg),mQ=Number(mQArg),samples=Number(samplesArg);
if(![maxStep,zW,kP,mQ,samples].every(Number.isFinite)||maxStep<=0||samples<=0)throw new Error("invalid numeric diagnostic argument");

// Diagnostic-only generation: retain Gate 7's 20 Hz observation/control
// schedule, zero thrust, real seed-20000 disturbances, and production model
// generation path. Only the physics substep and explicitly requested
// out-of-plane damping coefficients differ.
prepareVrxEpisode(20000,out,samples,1,0);
const worldPath=resolve(out,"world.sdf"),modelPath=resolve(out,"models/surveyor/model.sdf");
const world=readFileSync(worldPath,"utf8").replace(/<max_step_size>[^<]+<\/max_step_size>/,`<max_step_size>${maxStep}</max_step_size>`);
let model=readFileSync(modelPath,"utf8");
model=model.replace("<zW>0</zW>",`<zW>${zW}</zW>`).replace("<kP>0</kP>",`<kP>${kP}</kP>`).replace("<mQ>0</mQ>",`<mQ>${mQ}</mQ>`);
if(!world.includes(`<max_step_size>${maxStep}</max_step_size>`)||!model.includes(`<zW>${zW}</zW>`)||!model.includes(`<kP>${kP}</kP>`)||!model.includes(`<mQ>${mQ}</mQ>`))throw new Error("diagnostic transform failed");
writeFileSync(worldPath,world);writeFileSync(modelPath,model);
writeFileSync(resolve(out,"diagnostic-config.json"),JSON.stringify({schema_version:1,artifact_kind:"vrx-out-of-plane-damping-diagnostic",production_change:false,seed:20000,environment_scale:1,action_scale:0,observation_period_s:.05,max_step_size_s:maxStep,damping:{zW,kP,mQ},requested_observations:samples},null,2)+"\n");
