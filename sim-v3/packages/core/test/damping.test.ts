import assert from "node:assert/strict";
import {dampingWrench3, dampingWrench6} from "../src/damping.ts";

const params = {damping: {linear: {Xu: 2, Yv: 3, Nr: 4}, quadratic: {Xuu: 5, Yvv: 6, Nrr: 7}, linear6: [2, 3, 0, 0, 0, 4], quadratic6: [5, 6, 0, 0, 0, 7]}};
assert.deepEqual(dampingWrench3(params, [1, -2, 0.5]), [-7, 30, -3.75]);
assert.deepEqual(dampingWrench6(params, [1, -2, 0, 0, 0, 0.5]), [-7, 30, -0, -0, -0, -3.75]);
const diagonal = (values: number[]) => values.map((value, row) => values.map((_, col) => row === col ? value : 0));
const split = {damping: {potentialRadiationMatrix6: diagonal([1, 1, 1, 1, 1, 1]), linearViscousMatrix6: diagonal([2, 2, 2, 2, 2, 2]), quadraticViscousMatrix6: diagonal([3, 3, 3, 3, 3, 3])}};
assert.deepEqual(dampingWrench6(split, [1, -1, 2, -2, 0.5, -0.5]), [-6, 6, -18, 18, -2.25, 2.25]);
console.log("Damping decomposition tests passed.");
