import assert from "node:assert/strict";
import {mkdtempSync,readFileSync,rmSync} from "node:fs";
import {tmpdir} from "node:os";
import {join} from "node:path";
import {prepareGazeboEpisode} from "./prepare-gazebo-episode.ts";

const root=mkdtempSync(join(tmpdir(),"gazebo-current-test-"));
try{
 const output=prepareGazeboEpisode(42000,root,20,true,[.2,-.1,0]);
 const model=readFileSync(join(root,"models","surveyor","model.sdf"),"utf8");
 assert.match(model,/filename="libVrxCurrentRelativeVelocity\.so"/);
 assert.match(model,/<current_enu>-0\.1 0\.2 0<\/current_enu>/);
 const manifest=JSON.parse(readFileSync(output.manifestPath,"utf8"));
 assert.deepEqual(manifest.reset.disturbance.current_ned_mps,[.2,-.1,0]);
 console.log("Gazebo current injection serialization passed.");
}finally{rmSync(root,{recursive:true,force:true});}
