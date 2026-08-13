export type Matrix = number[][];
export type ModeName = "heave" | "roll" | "pitch";

export interface FrequencyCoefficients {
  omega_rad_s: number;
  added_mass: Matrix;
  radiation_damping: Matrix;
}
export type ComplexPair=[number,number];
export interface WaveExcitationCoefficients {omega_rad_s:number;heading_rad:number;complex_force:[ComplexPair,ComplexPair,ComplexPair,ComplexPair,ComplexPair,ComplexPair]}

export interface DampingProvenance {
  method: "capytaine" | "ikeda" | "empirical" | "free-decay-fit";
  source: string;
  checksum?: string;
  uncertainty?: string;
  operating_range?: string;
  fit_data?: string;
  limitations?: string[];
}

export interface DampingModel {
  potentialRadiationDamping: Matrix;
  linearViscousDamping: Matrix;
  quadraticViscousDamping: Matrix;
  provenance: {
    potential: DampingProvenance;
    linearViscous: DampingProvenance;
    quadraticViscous: DampingProvenance;
  };
}

export interface ResolvedHydrodynamics {
  approximation: "constant-coefficient-no-radiation-memory";
  evaluation_frequency_rad_s: number;
  selection_method: "wave-encounter" | "free-decay-fixed-point";
  intrinsic_wave_frequency_rad_s?: number;
  reference_speed_mps?: number;
  encounter_angle_rad?: number;
  added_mass: Matrix;
  damping: DampingModel;
  free_decay_iterations?: FreeDecayIteration[];
  selected_mode_vector?: number[];
  wave_excitation?: WaveExcitationCoefficients;
  interpolation_bracket_rad_s?: [number,number];
  warnings: string[];
}

const EPS = 1e-12;
const clone = (m: Matrix): Matrix => m.map((row) => [...row]);
const zeros = (n: number): Matrix => Array.from({length: n}, () => Array(n).fill(0));
const add = (a: Matrix, b: Matrix): Matrix => a.map((row, i) => row.map((v, j) => v + b[i][j]));
const dot = (a: number[], b: number[]): number => a.reduce((s, v, i) => s + v * b[i], 0);
const matVec = (a: Matrix, x: number[]): number[] => a.map((row) => dot(row, x));
const transpose = (a: Matrix): Matrix => a[0].map((_, j) => a.map((row) => row[j]));
const multiply = (a: Matrix, b: Matrix): Matrix => a.map((row) => transpose(b).map((column) => dot(row, column)));

function assertSquareFinite(name: string, matrix: Matrix, n?: number): void {
  const size = n ?? matrix.length;
  if (matrix.length !== size || matrix.some((row) => row.length !== size || row.some((v) => !Number.isFinite(v)))) {
    throw new Error(`${name} must be a finite ${size}x${size} matrix.`);
  }
}

export function validateDamping(model: DampingModel): void {
  const n = model.potentialRadiationDamping.length;
  assertSquareFinite("potential radiation damping", model.potentialRadiationDamping, n);
  assertSquareFinite("linear viscous damping", model.linearViscousDamping, n);
  assertSquareFinite("quadratic viscous damping", model.quadraticViscousDamping, n);
  const viscousMagnitude = [...model.linearViscousDamping.flat(), ...model.quadraticViscousDamping.flat()].reduce((s, v) => s + Math.abs(v), 0);
  if (viscousMagnitude <= EPS) throw new Error("coupled6 requires a non-zero viscous damping component; Capytaine radiation damping is not total damping.");
  if (model.provenance.potential.method !== "capytaine") throw new Error("Potential radiation damping provenance must identify Capytaine.");
  if (model.provenance.linearViscous.method === "capytaine" || model.provenance.quadraticViscous.method === "capytaine") {
    throw new Error("Capytaine cannot be the provenance source for viscous damping.");
  }
}

export function interpolateCoefficients(table: FrequencyCoefficients[], omega: number): FrequencyCoefficients {
  if (table.length < 2) throw new Error("At least two Capytaine frequency samples are required.");
  const sorted = [...table].sort((a, b) => a.omega_rad_s - b.omega_rad_s);
  if (omega < sorted[0].omega_rad_s || omega > sorted.at(-1)!.omega_rad_s) throw new Error(`Hydrodynamic frequency ${omega} rad/s is outside the Capytaine grid.`);
  let hi = sorted.findIndex((entry) => entry.omega_rad_s >= omega);
  if (hi === 0) return structuredClone(sorted[0]);
  const lo = hi - 1;
  const alpha = (omega - sorted[lo].omega_rad_s) / (sorted[hi].omega_rad_s - sorted[lo].omega_rad_s);
  const mix = (a: Matrix, b: Matrix): Matrix => a.map((row, i) => row.map((v, j) => v + alpha * (b[i][j] - v)));
  return {omega_rad_s: omega, added_mass: mix(sorted[lo].added_mass, sorted[hi].added_mass), radiation_damping: mix(sorted[lo].radiation_damping, sorted[hi].radiation_damping)};
}
function interpolationBracket(table:FrequencyCoefficients[],omega:number):[number,number]{const sorted=[...table].sort((a,b)=>a.omega_rad_s-b.omega_rad_s);const hi=sorted.findIndex((entry)=>entry.omega_rad_s>=omega);if(hi<0)throw new Error(`Hydrodynamic frequency ${omega} rad/s is outside the Capytaine grid.`);return hi===0?[sorted[0].omega_rad_s,sorted[0].omega_rad_s]:[sorted[hi-1].omega_rad_s,sorted[hi].omega_rad_s];}

export function interpolateWaveExcitation(table:WaveExcitationCoefficients[],omega:number,heading:number):WaveExcitationCoefficients{
  if(!table.length)throw new Error("Capytaine wave-excitation table is empty.");const normalized=Math.atan2(Math.sin(heading),Math.cos(heading)),headings=[...new Set(table.map((row)=>row.heading_rad))].sort((a,b)=>a-b),selected=headings.reduce((best,value)=>Math.abs(Math.atan2(Math.sin(value-normalized),Math.cos(value-normalized)))<Math.abs(Math.atan2(Math.sin(best-normalized),Math.cos(best-normalized)))?value:best,headings[0]),rows=table.filter((row)=>row.heading_rad===selected).sort((a,b)=>a.omega_rad_s-b.omega_rad_s);if(omega<rows[0].omega_rad_s||omega>rows.at(-1)!.omega_rad_s)throw new Error(`Wave-excitation frequency ${omega} rad/s is outside the Capytaine grid.`);const hi=rows.findIndex((row)=>row.omega_rad_s>=omega);if(hi===0)return structuredClone(rows[0]);const lo=hi-1,alpha=(omega-rows[lo].omega_rad_s)/(rows[hi].omega_rad_s-rows[lo].omega_rad_s),complex_force=rows[lo].complex_force.map((pair,index)=>[pair[0]+alpha*(rows[hi].complex_force[index][0]-pair[0]),pair[1]+alpha*(rows[hi].complex_force[index][1]-pair[1])] as ComplexPair) as WaveExcitationCoefficients["complex_force"];return{omega_rad_s:omega,heading_rad:selected,complex_force};
}

export function resolveEncounterFrequency(periodS: number, directionRad: number, vesselHeadingRad: number, referenceSpeedMps: number, depthM?: number): {intrinsic: number; encounter: number; waveNumber: number; angle: number} {
  if (!(periodS > 0)) throw new Error("Wave period must be positive.");
  const intrinsic = 2 * Math.PI / periodS;
  const g = 9.80665;
  let waveNumber = intrinsic * intrinsic / g;
  if (depthM !== undefined) {
    if (!(depthM > 0)) throw new Error("Water depth must be positive.");
    for (let i = 0; i < 30; i++) {
      const kh = waveNumber * depthM;
      const tanh = Math.tanh(kh);
      const f = g * waveNumber * tanh - intrinsic * intrinsic;
      const df = g * (tanh + kh * (1 - tanh * tanh));
      waveNumber = Math.max(waveNumber - f / df, EPS);
    }
  }
  const angle = directionRad - vesselHeadingRad;
  return {intrinsic, encounter: Math.abs(intrinsic - waveNumber * referenceSpeedMps * Math.cos(angle)), waveNumber, angle};
}

function inverse(matrix: Matrix): Matrix {
  const n = matrix.length;
  const a = matrix.map((row, i) => [...row, ...Array.from({length: n}, (_, j) => Number(i === j))]);
  for (let col = 0; col < n; col++) {
    let pivot = col;
    for (let row = col + 1; row < n; row++) if (Math.abs(a[row][col]) > Math.abs(a[pivot][col])) pivot = row;
    if (Math.abs(a[pivot][col]) < EPS) throw new Error("Singular mass matrix in modal solve.");
    [a[col], a[pivot]] = [a[pivot], a[col]];
    const scale = a[col][col];
    a[col] = a[col].map((v) => v / scale);
    for (let row = 0; row < n; row++) if (row !== col) {
      const factor = a[row][col];
      a[row] = a[row].map((v, j) => v - factor * a[col][j]);
    }
  }
  return a.map((row) => row.slice(n));
}

function cholesky(matrix: Matrix): Matrix {
  const n=matrix.length,L=zeros(n);
  for(let i=0;i<n;i++)for(let j=0;j<=i;j++){let sum=matrix[i][j];for(let k=0;k<j;k++)sum-=L[i][k]*L[j][k];if(i===j){if(!(sum>EPS))throw new Error("Mass matrix must be symmetric positive definite in modal solve.");L[i][j]=Math.sqrt(sum);}else L[i][j]=sum/L[j][j];}
  return L;
}

function symmetricEigen(matrix: Matrix): {values:number[];vectors:Matrix} {
  const n=matrix.length,a=clone(matrix),v=zeros(n);for(let i=0;i<n;i++)v[i][i]=1;
  for(let iteration=0;iteration<100;iteration++){let p=0,q=1,max=0;for(let i=0;i<n;i++)for(let j=i+1;j<n;j++)if(Math.abs(a[i][j])>max){max=Math.abs(a[i][j]);p=i;q=j;}if(max<1e-13)break;const angle=.5*Math.atan2(2*a[p][q],a[q][q]-a[p][p]),c=Math.cos(angle),s=Math.sin(angle);for(let k=0;k<n;k++)if(k!==p&&k!==q){const akp=a[k][p],akq=a[k][q];a[k][p]=a[p][k]=c*akp-s*akq;a[k][q]=a[q][k]=s*akp+c*akq;}const app=a[p][p],aqq=a[q][q],apq=a[p][q];a[p][p]=c*c*app-2*s*c*apq+s*s*aqq;a[q][q]=s*s*app+2*s*c*apq+c*c*aqq;a[p][q]=a[q][p]=0;for(let k=0;k<n;k++){const vkp=v[k][p],vkq=v[k][q];v[k][p]=c*vkp-s*vkq;v[k][q]=s*vkp+c*vkq;}}
  return{values:a.map((row,i)=>row[i]),vectors:v};
}

function modalAssurance(a:number[],b:number[]):number{return Math.pow(dot(a,b),2)/Math.max(dot(a,a)*dot(b,b),EPS);}

function dominantMode(mass: Matrix, stiffness: Matrix, target: number, previous?: number[]): {omega: number; vector: number[]} {
  const lower=cholesky(mass),lowerInverse=inverse(lower),standard=multiply(multiply(lowerInverse,stiffness),transpose(lowerInverse)),eigen=symmetricEigen(standard),backTransform=transpose(lowerInverse),candidates=eigen.values.map((lambda,index)=>{let vector=matVec(backTransform,eigen.vectors.map((row)=>row[index])),norm=Math.hypot(...vector);if(!(norm>EPS)||!(lambda>0)||!Number.isFinite(lambda))return null;vector=vector.map((x)=>x/norm);if(vector[target]<0)vector=vector.map((x)=>-x);return{lambda,vector,score:previous?modalAssurance(previous,vector):Math.abs(vector[target])};}).filter((entry):entry is {lambda:number;vector:number[];score:number}=>entry!==null).sort((a,b)=>b.score-a.score);
  if(!candidates.length||candidates[0].score<EPS)throw new Error("Mode lost during free-decay frequency solve.");
  return{omega:Math.sqrt(candidates[0].lambda),vector:candidates[0].vector};
}

export interface FreeDecayIteration {iteration: number; input_frequency_rad_s: number; modal_frequency_rad_s: number; relaxed_frequency_rad_s: number; modal_assurance: number; mode_vector: number[];added_mass:Matrix;radiation_damping:Matrix}

export function solveFreeDecayFrequency(mode: ModeName, rigidBodyMass: Matrix, hydrostaticStiffness: Matrix, table: FrequencyCoefficients[], options: {tolerance?: number; maxIterations?: number; relaxation?: number} = {}): {frequency_rad_s: number; coefficients: FrequencyCoefficients; iterations: FreeDecayIteration[]} {
  const names: ModeName[] = ["heave", "roll", "pitch"];
  const target = names.indexOf(mode);
  if (target < 0) throw new Error(`Unsupported free-decay mode: ${mode}`);
  assertSquareFinite("rigid-body mass", rigidBodyMass, 3);
  assertSquareFinite("hydrostatic stiffness", hydrostaticStiffness, 3);
  const sorted = [...table].sort((a, b) => a.omega_rad_s - b.omega_rad_s);
  let frequency = dominantMode(add(rigidBodyMass, sorted[0].added_mass), hydrostaticStiffness, target).omega;
  let previous: number[] | undefined;
  const iterations: FreeDecayIteration[] = [];
  const tolerance = options.tolerance ?? 1e-4;
  const maxIterations = options.maxIterations ?? 50;
  const relaxation = options.relaxation ?? 0.5;
  for (let iteration = 0; iteration < maxIterations; iteration++) {
    const coefficients = interpolateCoefficients(sorted, frequency);
    const modal = dominantMode(add(rigidBodyMass, coefficients.added_mass), hydrostaticStiffness, target, previous);
    const assurance = previous ? modalAssurance(previous, modal.vector) : 1;
    if (previous && assurance < 0.5) throw new Error("Mode lost during free-decay frequency solve.");
    const relaxed = frequency + relaxation * (modal.omega - frequency);
    iterations.push({iteration, input_frequency_rad_s: frequency, modal_frequency_rad_s: modal.omega, relaxed_frequency_rad_s: relaxed, modal_assurance: assurance, mode_vector: [...modal.vector],added_mass:clone(coefficients.added_mass),radiation_damping:clone(coefficients.radiation_damping)});
    if (Math.abs(relaxed - frequency) / Math.max(Math.abs(frequency), EPS) < tolerance) return {frequency_rad_s: relaxed, coefficients: interpolateCoefficients(sorted, relaxed), iterations};
    previous = modal.vector;
    frequency = relaxed;
  }
  throw new Error(`Free-decay frequency solve did not converge within ${maxIterations} iterations.`);
}

export function resolveWaveHydrodynamics(args: {table: FrequencyCoefficients[]; excitation_table?:WaveExcitationCoefficients[]; damping: Omit<DampingModel, "potentialRadiationDamping">; period_s: number; direction_rad: number; heading_rad: number; reference_speed_mps: number; water_depth_m?: number}): ResolvedHydrodynamics {
  const encounter = resolveEncounterFrequency(args.period_s, args.direction_rad, args.heading_rad, args.reference_speed_mps, args.water_depth_m);
  const coefficients = interpolateCoefficients(args.table, encounter.encounter);
  const damping: DampingModel = {...args.damping, potentialRadiationDamping: clone(coefficients.radiation_damping)};
  validateDamping(damping);
  return {
    approximation: "constant-coefficient-no-radiation-memory",
    evaluation_frequency_rad_s: encounter.encounter,
    selection_method: "wave-encounter",
    intrinsic_wave_frequency_rad_s: encounter.intrinsic,
    reference_speed_mps: args.reference_speed_mps,
    encounter_angle_rad: encounter.angle,
    added_mass: clone(coefficients.added_mass),
    damping,
    ...(args.excitation_table?{wave_excitation:interpolateWaveExcitation(args.excitation_table,encounter.encounter,encounter.angle)}:{}),
    interpolation_bracket_rad_s:interpolationBracket(args.table,encounter.encounter),
    warnings: ["Coefficients are fixed for the episode; Cummins radiation-memory convolution and time-varying encounter-frequency effects are omitted."]
  };
}

export function resolveFreeDecayHydrodynamics(args:{mode:ModeName;rigid_body_mass:Matrix;hydrostatic_stiffness:Matrix;table:FrequencyCoefficients[];damping:Omit<DampingModel,"potentialRadiationDamping">}):ResolvedHydrodynamics{
  const solved=solveFreeDecayFrequency(args.mode,args.rigid_body_mass,args.hydrostatic_stiffness,args.table),damping:DampingModel={...args.damping,potentialRadiationDamping:clone(solved.coefficients.radiation_damping)};validateDamping(damping);return{approximation:"constant-coefficient-no-radiation-memory",evaluation_frequency_rad_s:solved.frequency_rad_s,selection_method:"free-decay-fixed-point",added_mass:clone(solved.coefficients.added_mass),damping,free_decay_iterations:solved.iterations,selected_mode_vector:[...solved.iterations.at(-1)!.mode_vector],interpolation_bracket_rad_s:interpolationBracket(args.table,solved.frequency_rad_s),warnings:["Free-decay uses constant added mass and radiation damping at the converged modal frequency; Cummins radiation memory is omitted."]};
}

export function applyResolvedHydrodynamicsToCoupled6<T extends Record<string,any>>(params:T,resolved:ResolvedHydrodynamics,hydrostaticStiffness:Matrix):T{
  assertSquareFinite("resolved added mass",resolved.added_mass,6);assertSquareFinite("resolved hydrostatic stiffness",hydrostaticStiffness,6);validateDamping(resolved.damping);return{...params,addedMass:{...(params.addedMass??{}),matrix6:clone(resolved.added_mass)},damping:{...(params.damping??{}),potentialRadiationMatrix6:clone(resolved.damping.potentialRadiationDamping),linearViscousMatrix6:clone(resolved.damping.linearViscousDamping),quadraticViscousMatrix6:clone(resolved.damping.quadraticViscousDamping)},restoring:{...(params.restoring??{}),hydrostaticStiffnessMatrix6:clone(hydrostaticStiffness)},hydrodynamics:{...(params.hydrodynamics??{}),approximation:resolved.approximation,evaluation_frequency_rad_s:resolved.evaluation_frequency_rad_s,selection_method:resolved.selection_method,interpolation_bracket_rad_s:resolved.interpolation_bracket_rad_s,wave_excitation:resolved.wave_excitation,warnings:[...resolved.warnings]}};
}

export const matrix = {zeros, add};
