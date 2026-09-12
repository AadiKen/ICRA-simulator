export interface MassParameters {
  massProps?: {mass?: number; cg?: {x?: number; y?: number; z?: number}; inertia?: {Ix?: number; Iy?: number; Iz?: number; x?: number; y?: number; z?: number; Ixy?: number; Ixz?: number; Iyz?: number}};
  addedMass?: {matrix6?: number[][]; fluidAddedMass?: number[][]; XuDot?: number; YvDot?: number; ZwDot?: number; KpDot?: number; MqDot?: number; NrDot?: number; YrDot?: number; NvDot?: number; ZqDot?: number; MwDot?: number};
}

function skew([x, y, z]: number[]): number[][] {
  return [[0, -z, y], [z, 0, -x], [-y, x, 0]];
}

export function rigidBodyMassMatrix6(params: MassParameters): number[][] {
  const m = params.massProps?.mass || 0;
  const cg = params.massProps?.cg || {x: 0, y: 0, z: 0};
  const s = skew([cg.x || 0, cg.y || 0, cg.z || 0]);
  const inertia = params.massProps?.inertia || {};
  const ig = [[inertia.Ix || inertia.x || 0, inertia.Ixy || 0, inertia.Ixz || 0], [inertia.Ixy || 0, inertia.Iy || inertia.y || 0, inertia.Iyz || 0], [inertia.Ixz || 0, inertia.Iyz || 0, inertia.Iz || inertia.z || 0]];
  const matrix = Array.from({length: 6}, () => Array(6).fill(0));
  for (let i = 0; i < 3; i += 1) {
    matrix[i][i] = m;
    for (let j = 0; j < 3; j += 1) {
      matrix[i][j + 3] = -m * s[i][j];
      matrix[i + 3][j] = m * s[i][j];
      matrix[i + 3][j + 3] = ig[i][j];
    }
  }
  return matrix;
}

export function addedMassMatrix6(params: MassParameters): number[][] {
  const source = params.addedMass?.matrix6 || params.addedMass?.fluidAddedMass;
  if (source) return source.map((row) => [...row]);
  const a = params.addedMass || {};
  return [[-(a.XuDot || 0), 0, 0, 0, 0, 0], [0, -(a.YvDot || 0), 0, 0, 0, -(a.YrDot || 0)], [0, 0, -(a.ZwDot || 0), 0, -(a.ZqDot || 0), 0], [0, 0, 0, -(a.KpDot || 0), 0, 0], [0, 0, -(a.MwDot || 0), 0, -(a.MqDot || 0), 0], [0, -(a.NvDot || 0), 0, 0, 0, -(a.NrDot || 0)]];
}

export function addMassMatrices(a: number[][], b: number[][]): number[][] {
  return a.map((row, r) => row.map((value, c) => value + b[r][c]));
}

export function totalMassMatrix6(params: MassParameters): number[][] {
  return addMassMatrices(rigidBodyMassMatrix6(params), addedMassMatrix6(params));
}

export function planarMassMatrix3(params: MassParameters): number[][] {
  const m = params.massProps?.mass || 0;
  const xg = params.massProps?.cg?.x || 0;
  const inertia = params.massProps?.inertia || {};
  const iz = inertia.Iz || inertia.z || 1;
  const a = params.addedMass || {};
  return [[m - (a.XuDot || 0), 0, 0], [0, m - (a.YvDot || 0), m * xg - (a.YrDot || 0)], [0, m * xg - (a.NvDot || 0), iz - (a.NrDot || 0)]];
}
