import assert from "node:assert/strict";
import {addedMassCoriolis3, coriolisFromMass6, rigidBodyCoriolis3} from "../src/coriolis.ts";
import {totalMassMatrix6} from "../src/mass.ts";

const params = {massProps: {mass: 10, cg: {x: 0.2, y: 0.1, z: -0.05}, inertia: {Ix: 4, Iy: 5, Iz: 6}}, addedMass: {XuDot: -1, YvDot: -2, ZwDot: -3, KpDot: -0.4, MqDot: -0.5, NrDot: -0.6}};
const nu3 = [1.2, -0.3, 0.4];
assert.deepEqual(rigidBodyCoriolis3(params, nu3), [[0, 0, 2.1999999999999997], [0, 0, 12], [-2.1999999999999997, -12, 0]]);
assert.deepEqual(addedMassCoriolis3(params, nu3), [[0, 0, 0.6], [0, 0, 1.2], [-0.6, -1.2, 0]]);
const nu6 = [1.2, -0.3, 0.1, 0.02, -0.04, 0.4];
const c6 = coriolisFromMass6(totalMassMatrix6(params), nu6);
for (let row = 0; row < 6; row += 1) for (let col = 0; col < 6; col += 1) assert(Math.abs(c6[row][col] + c6[col][row]) === 0);
const power = nu6.reduce((sum, value, row) => sum + value * c6[row].reduce((inner, coefficient, col) => inner + coefficient * nu6[col], 0), 0);
assert(Math.abs(power) < 1e-12);
console.log("Coriolis construction tests passed.");
