import {readFileSync,writeFileSync} from "node:fs";

const path=process.argv[2];
if(!path)throw new Error("usage: apply-vrx-merged-body-diagnostic.ts model.sdf");
let source=readFileSync(path,"utf8");

// Diagnostic topology variant only. The articulated propeller masses and their
// parallel-axis contributions are folded back into base_link using the exact
// aggregate values from surveyor-vehicle-model.json. The coast diagnostic uses
// zero propeller thrust, so removing the rotor joints changes topology without
// changing the applied post-release wrench.
const aggregateInertial='<inertial><pose>0 0 0 0 0 0</pose><mass>52.3</mass><inertia><ixx>5.375132500000001</ixx><iyy>13.347308666666668</iyy><izz>17.891219833333338</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia><fluid_added_mass><xx>2.615</xx><yy>39.224999999999994</yy><rr>1.431297586666667</rr></fluid_added_mass></inertial>';
source=source.replace(/<inertial><pose>[^<]+<\/pose><mass>[^<]+<\/mass><inertia>[\s\S]*?<\/inertia><fluid_added_mass>[\s\S]*?<\/fluid_added_mass><\/inertial>/,aggregateInertial);
for(const id of ["port","starboard"]){
  source=source.replace(new RegExp(`\\s*<link name="${id}_propeller">[\\s\\S]*?<\\/link>`),"");
  source=source.replace(new RegExp(`\\s*<joint name="${id}_propeller_joint"[\\s\\S]*?<\\/joint>`),"");
  source=source.replace(new RegExp(`\\s*<plugin filename="gz-sim-thruster-system"[\\s\\S]*?<joint_name>${id}_propeller_joint<\\/joint_name>[\\s\\S]*?<\\/plugin>`),"");
}
if(source.includes("_propeller"))throw new Error("propeller articulation remained after merge");
writeFileSync(path,source);
