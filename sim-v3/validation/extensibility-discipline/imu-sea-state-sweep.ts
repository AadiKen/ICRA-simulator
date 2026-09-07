import {createHash} from "node:crypto";
import {readFileSync,renameSync,writeFileSync} from "node:fs";
import {dirname,relative,resolve} from "node:path";
import {fileURLToPath} from "node:url";
import {HullMotionPlatformStateService,ImuPlugin,SnapshotPlatformStateService} from "../../packages/sensor-sdk/src/index.ts";

const root=resolve(dirname(fileURLToPath(import.meta.url)),"../..");
const generator=resolve(fileURLToPath(import.meta.url));
const sensorSources=[
  "packages/sensor-sdk/src/plugins/imu.ts",
  "packages/sensor-sdk/src/services/platform-state.ts",
  "packages/sensor-sdk/src/runtime.ts",
];
const protocol={
  id:"imu-sea-state-fixed-truth-v1",seed:271828,samples_per_cell:512,sample_rate_hz:50,
  truth:{acceleration_body_mps2:[.1,.2,.3],angular_rate_body_rad_s:[.01,.02,.03],attitude_rad:[0,0,.4]},
  motion:{acceleration_body_mps2:[.1,.2,.3],angular_rate_body_rad_s:[.01,.02,.03]},
  sea_states:[
    {beaufort:0,significant_wave_height_m:0},{beaufort:2,significant_wave_height_m:.3},
    {beaufort:4,significant_wave_height_m:1.5},{beaufort:6,significant_wave_height_m:4},
    {beaufort:8,significant_wave_height_m:8},{beaufort:10,significant_wave_height_m:12},
    {beaufort:12,significant_wave_height_m:16},
  ],
};
const sha=(bytes:string|Buffer)=>createHash("sha256").update(bytes).digest("hex");
function rng(seed:number){let state=seed>>>0;return()=>{state=(state+0x6D2B79F5)>>>0;let v=state;v=Math.imul(v^v>>>15,v|1);v^=v+Math.imul(v^v>>>7,v|61);return((v^v>>>14)>>>0)/4294967296;};}
const rms=(values:number[])=>Math.sqrt(values.reduce((sum,value)=>sum+value*value,0)/values.length);

export function generate(){
  const cells=protocol.sea_states.map(sea=>{
    const base=new SnapshotPlatformStateService(()=>({actuator_states:[],actuator_energy_j:0,propulsion_energy_j:0,actuator_power_w:0}));
    const platform=new HullMotionPlatformStateService(base,()=>({surface:{sea_state_beaufort:sea.beaufort,significant_wave_height_m:sea.significant_wave_height_m}}),()=>protocol.motion);
    const plugin=new ImuPlugin(),random=rng(protocol.seed),truth=()=>protocol.truth;
    plugin.init({rate_hz:protocol.sample_rate_hz,latency_s:0,accel_bias_std_mps2:0,gyro_bias_std_rad_s:0,accel_drift_std_per_sqrt_s:0,gyro_drift_std_per_sqrt_s:0},{seededRng:random,groundTruth:truth,platformState:platform});
    const accel:number[]=[],gyro:number[]=[];let declared:any=null;
    for(let i=0;i<protocol.samples_per_cell;i++){
      const sample=plugin.sample({stepIndex:i,simTimeS:i/protocol.sample_rate_hz,lifecycleState:"ACTIVE"});
      if(!sample)throw new Error(`Missing IMU sample at sea state ${sea.beaufort}, step ${i}`);
      declared=sample.payload;
      for(let axis=0;axis<3;axis++){
        accel.push(sample.payload.acceleration_body_mps2[axis]-protocol.truth.acceleration_body_mps2[axis]);
        gyro.push(sample.payload.angular_rate_body_rad_s[axis]-protocol.truth.angular_rate_body_rad_s[axis]);
      }
    }
    plugin.dispose();
    return{...sea,vibration_rms_mps2:declared.vibration_rms_mps2,declared_accel_noise_std_mps2:declared.accel_noise_std_mps2,declared_gyro_noise_std_rad_s:declared.gyro_noise_std_rad_s,observed_accel_error_rms_mps2:rms(accel),observed_gyro_error_rms_rad_s:rms(gyro)};
  });
  return{schema_version:1,artifact_kind:"imu-sea-state-response-sweep",status:"complete-offline-deterministic",protocol,implementation:{plugin:"ImuPlugin",platform_service:"HullMotionPlatformStateService",sensor_source_sha256:Object.fromEntries(sensorSources.map(path=>[path,sha(readFileSync(resolve(root,path)))])),generator_path:relative(root,generator),generator_sha256:sha(readFileSync(generator))},cells,claim_limit:"Fixed-truth sensor responsiveness test; it validates configured IMU noise conditioning, not physical vessel motion or a calibrated sea-state model."};
}

if(import.meta.url===`file://${process.argv[1]}`){
  const output=resolve(process.argv[2]??resolve(root,"artifacts/extensibility-discipline/imu-sea-state-sweep.json"));
  const temporary=`${output}.tmp`,text=JSON.stringify(generate(),null,2)+"\n";writeFileSync(temporary,text);renameSync(temporary,output);console.log(output);
}
