export interface ThrusterLocation {x_m: number; y_m: number; max_thrust_n: number}
export interface AllocationDiagnostics {
  rank: number;
  singular_values: number[];
  condition_number: number;
  achieved_wrench: [number, number, number];
  residual_wrench: [number, number, number];
  active_constraints: string[];
  degradation: "none" | "ill-conditioned" | "rank-deficient";
  reachable_wrench: [number, number, number];
  stuck_actuator_bias: [number, number, number];
  infeasible: boolean;
}

function transpose(a: number[][]): number[][] { return a[0].map((_, j) => a.map((row) => row[j])); }
function multiply(a: number[][], b: number[][]): number[][] { return a.map((row) => b[0].map((_, j) => row.reduce((s, v, k) => s + v * b[k][j], 0))); }
function matVec(a: number[][], x: number[]): number[] { return a.map((row) => row.reduce((s, v, i) => s + v * x[i], 0)); }

function inverse3(a: number[][]): number[][] {
  const [m00,m01,m02] = a[0], [m10,m11,m12] = a[1], [m20,m21,m22] = a[2];
  const det = m00*(m11*m22-m12*m21)-m01*(m10*m22-m12*m20)+m02*(m10*m21-m11*m20);
  if (Math.abs(det) < 1e-18) throw new Error("Regularized allocation matrix unexpectedly singular.");
  return [[m11*m22-m12*m21,m02*m21-m01*m22,m01*m12-m02*m11],[m12*m20-m10*m22,m00*m22-m02*m20,m02*m10-m00*m12],[m10*m21-m11*m20,m01*m20-m00*m21,m00*m11-m01*m10]].map((r)=>r.map((v)=>v/det));
}

function symmetricEigenvalues3(a: number[][]): number[] {
  const m = a.map((r) => [...r]);
  for (let sweep=0;sweep<30;sweep++) {
    let p=0,q=1;
    for (let i=0;i<3;i++) for (let j=i+1;j<3;j++) if (Math.abs(m[i][j])>Math.abs(m[p][q])) [p,q]=[i,j];
    if (Math.abs(m[p][q])<1e-12) break;
    const phi=0.5*Math.atan2(2*m[p][q],m[q][q]-m[p][p]), c=Math.cos(phi), s=Math.sin(phi);
    const app=c*c*m[p][p]-2*s*c*m[p][q]+s*s*m[q][q], aqq=s*s*m[p][p]+2*s*c*m[p][q]+c*c*m[q][q];
    for (let k=0;k<3;k++) if(k!==p&&k!==q){const mkp=m[k][p],mkq=m[k][q];m[k][p]=m[p][k]=c*mkp-s*mkq;m[k][q]=m[q][k]=s*mkp+c*mkq;}
    m[p][p]=app;m[q][q]=aqq;m[p][q]=m[q][p]=0;
  }
  return [m[0][0],m[1][1],m[2][2]].sort((a,b)=>b-a);
}

export function allocateDualAzimuth(desired: [number, number, number], thrusters: [ThrusterLocation, ThrusterLocation], options: {conditionLimit?: number; damping?: number; weights?:[number,number,number]; unavailable?:boolean[]; stuck?:Array<{fx_n:number;fy_n:number}|null>; infeasibilityTolerance?:number} = {}): {commands: [{fx_n:number;fy_n:number},{fx_n:number;fy_n:number}]; diagnostics: AllocationDiagnostics} {
  const [a,b]=thrusters;
  const nominal=[[1,0,1,0],[0,1,0,1],[-a.y_m,a.x_m,-b.y_m,b.x_m]];
  const stuck=options.stuck??[null,null],unavailable=options.unavailable??[false,false];
  const stuckComponents=[stuck[0]?.fx_n??0,stuck[0]?.fy_n??0,stuck[1]?.fx_n??0,stuck[1]?.fy_n??0];
  const bias=matVec(nominal,stuckComponents) as [number,number,number];
  const target=desired.map((value,index)=>value-bias[index]) as [number,number,number];
  const weights=options.weights??[1,1,1];
  const matrix=nominal.map((row,rowIndex)=>row.map((value,column)=>value*weights[rowIndex]*(unavailable[Math.floor(column/2)]||stuck[Math.floor(column/2)]?0:1)));
  const weightedTarget=target.map((value,index)=>value*weights[index]);
  const gram=multiply(matrix,transpose(matrix));
  const eigen=symmetricEigenvalues3(gram).map((v)=>Math.max(v,0));
  const singular=eigen.map(Math.sqrt);
  const tolerance=Math.max(singular[0]*1e-9,1e-10), rank=singular.filter((v)=>v>tolerance).length;
  const condition=singular.at(-1)!>tolerance?singular[0]/singular.at(-1)!:Infinity;
  const limit=options.conditionLimit??1e5;
  const degradation=rank<3?"rank-deficient":condition>limit?"ill-conditioned":"none";
  const lambda=degradation==="none"?(options.damping??1e-8):Math.max(options.damping??1e-3,singular[0]*1e-4);
  const regularized=gram.map((row,i)=>row.map((v,j)=>v+(i===j?lambda*lambda:0)));
  let components=matVec(transpose(matrix),matVec(inverse3(regularized),weightedTarget));
  const active:string[]=[];
  for(let i=0;i<2;i++){
    const mag=Math.hypot(components[i*2],components[i*2+1]), max=thrusters[i].max_thrust_n;
    if(mag>max){components[i*2]*=max/mag;components[i*2+1]*=max/mag;active.push(`thruster-${i+1}-magnitude`);}
  }
  if(degradation!=="none") active.push(degradation);
  const commandedAchieved=matVec(nominal,components) as [number,number,number];
  const achieved=commandedAchieved.map((value,index)=>value+bias[index]) as [number,number,number];
  const residual=desired.map((v,i)=>v-achieved[i]) as [number,number,number];
  if([...components,...achieved,...residual].some((v)=>!Number.isFinite(v))) throw new Error("Allocation produced a non-finite command.");
  const residualNorm=Math.hypot(...residual),desiredNorm=Math.max(Math.hypot(...desired),1);
  const infeasible=residualNorm>(options.infeasibilityTolerance??1e-3)*desiredNorm;
  if(infeasible)active.push("unachievable-wrench");
  return {commands:[{fx_n:components[0],fy_n:components[1]},{fx_n:components[2],fy_n:components[3]}],diagnostics:{rank,singular_values:singular,condition_number:condition,achieved_wrench:achieved,residual_wrench:residual,active_constraints:active,degradation,reachable_wrench:achieved,stuck_actuator_bias:bias,infeasible}};
}
