import {createHash} from "node:crypto";
import {mkdirSync,readFileSync,writeFileSync} from "node:fs";
import {existsSync} from "node:fs";
import {dirname,resolve} from "node:path";
import {fileURLToPath} from "node:url";
import {resolveExperiment} from "../../packages/experiment-schema/src/index.ts";

const root=resolve(fileURLToPath(new URL("../../",import.meta.url)));
const read=(path:string)=>readFileSync(resolve(root,path),"utf8");
const lines=(text:string)=>text.split(/\r?\n/).filter((line)=>line.trim()&&!line.trim().startsWith("#")).length;
const sha=(text:string)=>createHash("sha256").update(text).digest("hex");

export function run(){
  const bcodPath="validation/vrx-comparison/bcod-stationkeeping.json",vrxWorldPath=".cache/vrx/v3.1.2/vrx_gz/worlds/stationkeeping_task.sdf",vrxLaunchPath=".cache/vrx/v3.1.2/vrx_gz/launch/competition.launch.py";
  if(!existsSync(resolve(root,vrxWorldPath)))throw new Error("VRX v3.1.2 cache is absent. Run `npm run fetch:vrx` before regenerating the comparison artifact.");
  const bcodText=read(bcodPath),world=read(vrxWorldPath),launch=read(vrxLaunchPath),original=JSON.parse(bcodText),resolvedExperiment=resolveExperiment(original);
  const fuelAssets=[...world.matchAll(/<uri>https:\/\/fuel\.gazebosim\.org\/[^<]+<\/uri>/g)].map((match)=>match[0]).filter((value,index,all)=>all.indexOf(value)===index);
  return{schema_version:1,artifact_kind:"vrx-scenario-construction-comparison",status:"runtime-cross-check-executed-human-authoring-time-unmeasured",comparison_role:"usability-and-setup-baseline-not-physics-validation",versions:{vrx:"3.1.2",vrx_commit:"260fa40976e87cb33f417d61762b9a0cec17cafb",ros2:"Jazzy",gazebo_harmonic:"8.14.0",docker_engine:"29.6.1-linux-arm64"},equivalent_scope:"Station keeping with WAM-V/Vehicle C, wind/current, GPS, IMU, camera, LiDAR, and mission scoring.",construction:{bcod:{scenario_specific_files:1,configuration_nonblank_lines:lines(bcodText),required_external_assets:0,setup_commands:1,config_checksum_sha256:sha(bcodText),resolved_checksum_sha256:resolvedExperiment.resolution.checksum_sha256},vrx:{scenario_specific_world_files:1,world_nonblank_lines:lines(world),generic_launch_nonblank_lines:lines(launch),required_fuel_assets:fuelAssets.length,required_container_build_stages:2,world_checksum_sha256:sha(world),launch_checksum_sha256:sha(launch)}},measured_setup:{initial_compose_builder_result:"failed-missing-local-vrx-base-image",base_image_build_seconds:277.6,builder_image_build_seconds:99.0,first_cold_launch_seconds:30,first_cold_launch_result:"world-load-incomplete-during-fuel-download",persistent_cache_launch_bound_seconds:120,persistent_cache_launch_result:"wamv-spawned-sensors-initialized-scoring-plugin-running-then-intentional-timeout",runtime_exit_code:124,exit_code_interpretation:"GNU timeout boundary, not simulator failure",source_checkout_size_mib:280},human_authoring_time:{bcod:null,vrx:null,status:"not-measured",reason:"No controlled human construction study was conducted; machine generation/build timing is not a substitute."},reproduction:{source:"https://github.com/osrf/vrx",checkout:"git clone --branch v3.1.2 --depth 1 https://github.com/osrf/vrx.git .cache/vrx/v3.1.2",build:["docker compose build base","docker compose build builder"],run:"docker run --rm -v bcod-vrx-fuel:/root/.gz/fuel vrx-builder:latest bash -lc 'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && timeout 120s ros2 launch vrx_gz competition.launch.py world:=stationkeeping_task headless:=true sim_mode:=sim competition_mode:=true'"},claim_limit:"This executed comparison measures configuration surface and local setup/runtime overhead. It does not validate BCOD physics, and without a controlled human study it does not establish scenario-authoring speed."};
}

if(process.argv[1]===fileURLToPath(import.meta.url)){const artifact=run(),output=resolve(process.argv[2]??"artifacts/vrx/construction-comparison.json");mkdirSync(dirname(output),{recursive:true});writeFileSync(output,`${JSON.stringify(artifact,null,2)}\n`);console.log(JSON.stringify({output,status:artifact.status,construction:artifact.construction,measured_setup:artifact.measured_setup},null,2));}
