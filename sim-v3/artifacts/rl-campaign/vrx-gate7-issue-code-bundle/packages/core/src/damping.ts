export interface LegacyDampingParameters {
  damping?: {
    linear?: {Xu?: number; Yv?: number; Zw?: number; Kp?: number; Mq?: number; Nr?: number};
    quadratic?: {Xuu?: number; Yvv?: number; Zww?: number; Kpp?: number; Mqq?: number; Nrr?: number};
    linearMatrix?: number[][]; quadraticMatrix?: number[][]; linear6?: number[]; quadratic6?: number[];
    potentialRadiationMatrix6?: number[][]; linearViscousMatrix6?: number[][]; quadraticViscousMatrix6?: number[][];
  };
}

const matVec = (matrix: number[][], vector: number[]): number[] => matrix.map((row) => row.reduce((sum, value, col) => sum + value * vector[col], 0));
const add = (a: number[], b: number[]): number[] => a.map((value, index) => value + b[index]);

export function dampingWrench3(params: LegacyDampingParameters, nu: number[]): number[] {
  const [u, v, r] = nu, damping = params.damping!, linear = damping.linear!, quadratic = damping.quadratic!;
  if (damping.linearMatrix || damping.quadraticMatrix) {
    const linearWrench = damping.linearMatrix ? matVec(damping.linearMatrix, nu) : [0, 0, 0];
    const absNu = nu.map((value) => Math.abs(value) * value);
    const quadraticWrench = damping.quadraticMatrix ? matVec(damping.quadraticMatrix, absNu) : [0, 0, 0];
    return linearWrench.map((value, idx) => -(value + quadraticWrench[idx]));
  }
  return [-((linear.Xu || 0) * u + (quadratic.Xuu || 0) * Math.abs(u) * u), -((linear.Yv || 0) * v + (quadratic.Yvv || 0) * Math.abs(v) * v), -((linear.Nr || 0) * r + (quadratic.Nrr || 0) * Math.abs(r) * r)];
}

export function dampingWrench6(params: LegacyDampingParameters, relativeNu: number[]): number[] {
  const damping = params.damping || {};
  const linear6 = damping.linear6 || [damping.linear?.Xu || 0, damping.linear?.Yv || 0, damping.linear?.Zw || 0, damping.linear?.Kp || 0, damping.linear?.Mq || 0, damping.linear?.Nr || 0];
  const quadratic6 = damping.quadratic6 || [damping.quadratic?.Xuu || 0, damping.quadratic?.Yvv || 0, damping.quadratic?.Zww || 0, damping.quadratic?.Kpp || 0, damping.quadratic?.Mqq || 0, damping.quadratic?.Nrr || 0];
  if (damping.potentialRadiationMatrix6 || damping.linearViscousMatrix6 || damping.quadraticViscousMatrix6) {
    const velocitySquared = relativeNu.map((value) => Math.abs(value) * value);
    const potential = damping.potentialRadiationMatrix6 ? matVec(damping.potentialRadiationMatrix6, relativeNu) : Array(6).fill(0);
    const linearViscous = damping.linearViscousMatrix6 ? matVec(damping.linearViscousMatrix6, relativeNu) : Array(6).fill(0);
    const quadraticViscous = damping.quadraticViscousMatrix6 ? matVec(damping.quadraticViscousMatrix6, velocitySquared) : Array(6).fill(0);
    return add(add(potential, linearViscous), quadraticViscous).map((value) => -value);
  }
  return relativeNu.map((value, i) => -((linear6[i] || 0) * value + (quadratic6[i] || 0) * Math.abs(value) * value));
}
