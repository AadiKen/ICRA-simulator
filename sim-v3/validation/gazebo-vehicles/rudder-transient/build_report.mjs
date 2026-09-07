import {createHash} from "node:crypto";
import {mkdir,readFile,writeFile} from "node:fs/promises";

const root="artifacts/gazebo/vehicle-b/rudder-transient";
const read=async path=>{const bytes=await readFile(path);return{path,sha256:createHash("sha256").update(bytes).digest("hex"),value:JSON.parse(bytes)}};
const task1Protocol=await read(`${root}/task1-protocol.json`),task1=await read(`${root}/task1-results.json`),task2Protocol=await read(`${root}/task2-bound-sensitivity-protocol.json`),task2=await read(`${root}/task2-bound-sensitivity-report.json`),candidateProtocol=await read(`${root}/candidate-separation-protocol.json`),candidates12=await read(`${root}/candidate-1-2-report.json`),candidate3=await read(`${root}/candidate-3-multistart-report.json`);
const artifact={
  schema_version:1,
  artifact_kind:"vehicle-b-rudder-transient-diagnostic-campaign",
  date:"2026-09-05",
  status:"working-hypothesis-not-supported-stop-before-task3",
  priority:"background; does not block Vehicle C, figure machinery, or PPO work",
  immutable_prior_work:{modified:false,paths:["artifacts/gazebo/vehicle-b-cross-implementation-protocol.json","validation/external-references/kvlcc2-marin/artifacts/identification/identification-report.json","artifacts/gazebo/vehicle-b/scoped-negative-result.json"]},
  task1:{
    name:"leave-one-out refit",
    protocol:{path:task1Protocol.path,sha256:task1Protocol.sha256,registered_before_results:task1Protocol.value.status==="precommitted-before-results"},
    prediction:task1Protocol.value.prediction,
    result:task1.value,
    conclusion:"The strict predicted separation is false: maximum turning-circle held-out normalized MSE (1.923593) exceeds minimum zig-zag held-out normalized MSE (0.778946). The coverage-gap hypothesis is not supported by Task 1.",
    prediction_confirmed:false
  },
  task2:{
    name:"bound sensitivity",
    protocol:{path:task2Protocol.path,sha256:task2Protocol.sha256,registered_before_results:task2Protocol.value.status==="registered-before-task2-results"},
    predictions:task2Protocol.value.predictions,
    premise_correction:"The immutable source fit has two exact bound hits, Y_R and X_vr, not three. N_rrr is interior. Only the two reproducible hits were relaxed.",
    result:task2.value,
    conclusion:"Fivefold wider bounds produced stable interior solutions just beyond the old bounds, but all four held-out zig-zags worsened and the preregistered substantial-improvement criterion failed. Tight original bounds are not supported as the explanation.",
    prediction_confirmed:false
  },
  task3:{name:"KVLCC2 full-scale structural check",status:"not-run-by-preregistered-decision-rule",reason:"Task 1 did not support the coverage-gap hypothesis; instructions require stopping for review rather than proceeding on the current assumption."},
  task4:{name:"source new transient-maneuver data",status:"not-authorized-not-started"},
  candidate_separation:{protocol:{path:candidateProtocol.path,sha256:candidateProtocol.sha256,registered_before_results:true},sign_or_frame:candidates12.value.sign_or_frame_assessment,measured_mirror_pairs:candidates12.value.mirror_pairs,asymmetry_representation:candidates12.value.asymmetry_representation,multistart:candidate3.value},
  overall_interpretation:"The tested data-coverage and tight-bound explanations do not account for the transient failures. No simple ingestion sign/frame error is indicated. Real port/starboard asymmetry is present in the measurements and only partly representable by the model. The preregistered multi-start non-uniqueness rule was not met, although one higher-training-cost solution had much better held-out errors, pointing to possible objective mismatch rather than demonstrated equal-cost local minima. Model-form mismatch and identifiability limitations remain open.",
  claim_limit:"Separate diagnostic campaign only. It does not relax, supersede, or reinterpret the original preregistered Vehicle B result, and it does not validate Vehicle B USV coefficients."
};
await mkdir(root,{recursive:true});await writeFile(`${root}/report.json`,JSON.stringify(artifact,null,2)+"\n");console.log(JSON.stringify({path:`${root}/report.json`,status:artifact.status,task1_confirmed:artifact.task1.prediction_confirmed,task2_confirmed:artifact.task2.prediction_confirmed,task3:artifact.task3.status},null,2));
