import type {MassParameters} from "./mass.ts";

function skew([x, y, z]: number[]): number[][] {
  return [[0, -z, y], [z, 0, -x], [-y, x, 0]];
}

export function rigidBodyCoriolis3(params: MassParameters, velocity: number[]): number[][] {
  const m = params.massProps!.mass!;
  const xg = params.massProps!.cg?.x || 0;
  const [u, v, r] = velocity;
  return [[0, 0, -m * (xg * r + v)], [0, 0, m * u], [m * (xg * r + v), -m * u, 0]];
}

export function addedMassCoriolis3(params: MassParameters, relativeVelocity: number[]): number[][] {
  const [u, v, r] = relativeVelocity;
  const Xu = params.addedMass!.XuDot || 0;
  const Yv = params.addedMass!.YvDot || 0;
  const Nr = params.addedMass!.NrDot || 0;
  const a1 = Xu * u, a2 = Yv * v, a3 = Nr * r;
  return [[0, 0, a2], [0, 0, -a1], [-a2, a1, 0]].map((row, rowIdx) => row.map((value, colIdx) => {
    if (rowIdx === 2 && colIdx === 0) return -a2;
    if (rowIdx === 2 && colIdx === 1) return a1;
    return value + (a3 * 0);
  }));
}

function matVecAdd(a: number[][], x: number[], b: number[][], y: number[]): number[] {
  return a.map((row, idx) => {
    const left = row.reduce((sum, value, col) => sum + value * x[col], 0);
    const right = b[idx].reduce((sum, value, col) => sum + value * y[col], 0);
    return left + right;
  });
}

export function coriolisFromMass6(massMatrix: number[][], nu: number[]): number[][] {
  const nu1 = nu.slice(0, 3), nu2 = nu.slice(3, 6);
  const m11 = massMatrix.slice(0, 3).map((row) => row.slice(0, 3));
  const m12 = massMatrix.slice(0, 3).map((row) => row.slice(3, 6));
  const m21 = massMatrix.slice(3, 6).map((row) => row.slice(0, 3));
  const m22 = massMatrix.slice(3, 6).map((row) => row.slice(3, 6));
  const sa = skew(matVecAdd(m11, nu1, m12, nu2));
  const sb = skew(matVecAdd(m21, nu1, m22, nu2));
  const c = Array.from({length: 6}, () => Array(6).fill(0));
  for (let row = 0; row < 3; row += 1) for (let col = 0; col < 3; col += 1) {
    c[row][col + 3] = -sa[row][col];
    c[row + 3][col] = -sa[row][col];
    c[row + 3][col + 3] = -sb[row][col];
  }
  return c;
}
