import assert from "node:assert/strict";
import {addedMassMatrix6, planarMassMatrix3, rigidBodyMassMatrix6, totalMassMatrix6} from "../src/mass.ts";

const params = {
  massProps: {mass: 10, cg: {x: 1, y: 2, z: 3}, inertia: {Ix: 4, Iy: 5, Iz: 6, Ixy: 0.1, Ixz: 0.2, Iyz: 0.3}},
  addedMass: {XuDot: -1, YvDot: -2, ZwDot: -3, KpDot: -0.4, MqDot: -0.5, NrDot: -0.6, YrDot: -0.7, NvDot: -0.7}
};
const rigid = rigidBodyMassMatrix6(params);
assert.deepEqual(rigid[0], [10, 0, 0, -0, 30, -20]);
assert.deepEqual(rigid[3], [0, -30, 20, 4, 0.1, 0.2]);
assert.deepEqual(addedMassMatrix6(params)[1], [0, 2, 0, 0, 0, 0.7]);
assert.deepEqual(totalMassMatrix6(params)[1], [0, 12, 0, -30, 0, 10.7]);
assert.deepEqual(planarMassMatrix3(params), [[11, 0, 0], [0, 12, 10.7], [0, 10.7, 6.6]]);
const supplied = Array.from({length: 6}, (_, row) => Array.from({length: 6}, (_, col) => row === col ? row + 1 : 0));
const cloned = addedMassMatrix6({addedMass: {matrix6: supplied}});
assert.deepEqual(cloned, supplied);
assert.notEqual(cloned, supplied);
assert.notEqual(cloned[0], supplied[0]);
console.log("Mass/inertia construction tests passed.");
