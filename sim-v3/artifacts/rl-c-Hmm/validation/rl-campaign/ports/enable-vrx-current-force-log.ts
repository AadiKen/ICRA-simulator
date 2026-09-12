import {readFileSync,writeFileSync} from "node:fs";

const path=process.argv[2], output=process.argv[3]??"/episode/current-force.csv",
  initialBodyVelocity=process.argv[4], driveBodyForce=process.argv[5],
  driveDuration=process.argv[6];
if(!path)throw new Error("usage: enable-vrx-current-force-log.ts model.sdf [container-output-path]");
const source=readFileSync(path,"utf8");
const marker="</plugin>";
const pluginStart=source.indexOf('<plugin filename="libVrxCurrentRelativeVelocity.so"');
if(pluginStart<0)throw new Error("current-relative-velocity plugin not found");
const pluginEnd=source.indexOf(marker,pluginStart);
if(pluginEnd<0)throw new Error("current plugin closing tag not found");
const initial=initialBodyVelocity?
  `<diagnostic_initial_body_velocity>${initialBodyVelocity}</diagnostic_initial_body_velocity>`:"";
const drive=driveBodyForce?
  `<diagnostic_drive_body_force>${driveBodyForce}</diagnostic_drive_body_force><diagnostic_drive_duration_s>${driveDuration??1}</diagnostic_drive_duration_s>`:"";
const updated=source.slice(0,pluginEnd)+`<diagnostic_output>${output}</diagnostic_output>${initial}${drive}`+source.slice(pluginEnd);
writeFileSync(path,updated);
