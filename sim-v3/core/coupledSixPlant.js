import {MarinePlant, PHYSICS_MODES, legacyWrenchToSix, validateStep} from "./marinePlant.js";
import {invertMatrix, matVecMul} from "./math.js";
import {
    coriolisFromMass6,
    normalizeQuaternion,
    quaternionDerivative,
    rotationBodyToNed,
    totalMassMatrix6
} from "./sixDof.js";
import {dampingWrench6} from "../packages/core/src/damping.ts";
import {linearHydrostaticWrench} from "../packages/core/src/hydrostatics.ts";
export {linearHydrostaticWrench} from "../packages/core/src/hydrostatics.ts";

const add = (a, b) => a.map((value, i) => value + b[i]);
const scale = (a, s) => a.map((value) => value * s);
const transposeMul = (m, v) => m[0].map((_, c) => m.reduce((sum, row, r) => sum + row[c] * v[r], 0));

function cloneState(state) {
    return state.clone();
}

function vectorFromState(state) {
    return [
        state.position.N, state.position.E, state.position.D,
        state.quaternion.w, state.quaternion.x, state.quaternion.y, state.quaternion.z,
        state.velocity.u, state.velocity.v, state.velocity.w,
        state.angularRate.p, state.angularRate.q, state.angularRate.r
    ];
}

function stateFromVector(state, y) {
    state.position = {N: y[0], E: y[1], D: y[2]};
    state.quaternion = normalizeQuaternion({w: y[3], x: y[4], y: y[5], z: y[6]});
    state.velocity = {u: y[7], v: y[8], w: y[9]};
    state.angularRate = {p: y[10], q: y[11], r: y[12]};
    return state;
}

export function analyticAxisAlignedBoxSubmersion(primitive, centerD, waterD = 0) {
    if (primitive.type !== "box") throw new Error("Analytic submersion currently supports box primitives only.");
    const dims = primitive.dims || {};
    const length = dims.length || dims.x || 0;
    const beam = dims.beam || dims.y || 0;
    const height = dims.height || dims.z || 0;
    const topD = centerD - height / 2;
    const bottomD = centerD + height / 2;
    const submergedTopD = Math.max(topD, waterD);
    const submergedHeight = Math.max(0, bottomD - submergedTopD);
    return {
        volume: length * beam * submergedHeight,
        centroidD: submergedHeight > 0 ? submergedTopD + submergedHeight / 2 : bottomD,
        fraction: height > 0 ? submergedHeight / height : 0
    };
}

export class CoupledSixPlant extends MarinePlant {
    constructor(params, forceModels = [], integrator = "rk4") {
        super(params, PHYSICS_MODES.COUPLED6);
        this.forceModels = forceModels;
        this.integrator = integrator;
        this.massMatrix = totalMassMatrix6(params);
        this.massMatrixInv = invertMatrix(this.massMatrix);
        this.equilibriumD = null;
    }

    prepareStep(state, env, command, dt, t) {
        if (this.equilibriumD === null) this.equilibriumD = state.position.D;
        const ctx = {state, env, command, dt, t, params: this.params};
        this.forceModels.forEach((model) => model.prepareStep?.(ctx));
    }

    derivative(state, env = {}, command = {}, t = 0) {
        const euler = state.eulerAngles;
        const rotation = rotationBodyToNed(euler);
        const nu = [
            state.velocity.u, state.velocity.v, state.velocity.w,
            state.angularRate.p, state.angularRate.q, state.angularRate.r
        ];
        const waterNed = [env.waterV?.z || 0, env.waterV?.x || 0, -(env.waterV?.y || 0)];
        const waterBody = transposeMul(rotation, waterNed);
        const relativeNu = [nu[0] - waterBody[0], nu[1] - waterBody[1], nu[2] - waterBody[2], nu[3], nu[4], nu[5]];
        this.forceBreakdown = {};
        let tau = Array(6).fill(0);
        this.forceModels.forEach((model, index) => {
            const raw = model.computeWrench({
                state, env, command, t, dt: 0, params: this.params,
                velocityVector: [nu[0], nu[1], nu[5]],
                relativeVelocityVector: [relativeNu[0], relativeNu[1], relativeNu[5]]
            });
            const wrench = model.lastFullWrench?.length === 6 ? [...model.lastFullWrench] : legacyWrenchToSix(raw);
            this.forceBreakdown[model.constructor?.name || `force${index}`] = wrench;
            tau = add(tau, wrench);
        });
        const damping = dampingWrench6(this.params, relativeNu);
        if (this.params.maneuveringModel?.replacesPlanarDamping) for (const index of [0, 1, 5]) damping[index] = 0;
        const restoring = linearHydrostaticWrench(this.params, state, this.equilibriumD);
        const coriolis = scale(matVecMul(coriolisFromMass6(this.massMatrix, relativeNu), relativeNu), -1);
        this.forceBreakdown.HydrodynamicDamping6 = damping;
        this.forceBreakdown.Hydrostatics6 = restoring;
        this.forceBreakdown.Coriolis6 = coriolis;
        tau = add(add(add(tau, damping), restoring), coriolis);
        this.lastWrench = tau;
        const nuDot = matVecMul(this.massMatrixInv, tau);
        const posDot = matVecMul(rotation, nu.slice(0, 3));
        const qDot = quaternionDerivative(state.quaternion, state.angularRate);
        return [
            ...posDot, qDot.w, qDot.x, qDot.y, qDot.z, ...nuDot
        ];
    }

    step(state, env, command, dt, t = 0) {
        validateStep(dt, state);
        this.prepareStep(state, env, command, dt, t);
        const y0 = vectorFromState(state);
        let next;
        if (this.integrator === "semiImplicitEuler") {
            const d = this.derivative(state, env, command, t);
            next = add(y0, scale(d, dt));
        }
        else {
            const k1 = this.derivative(cloneState(state), env, command, t);
            const k2 = this.derivative(stateFromVector(cloneState(state), add(y0, scale(k1, dt / 2))), env, command, t + dt / 2);
            const k3 = this.derivative(stateFromVector(cloneState(state), add(y0, scale(k2, dt / 2))), env, command, t + dt / 2);
            const k4 = this.derivative(stateFromVector(cloneState(state), add(y0, scale(k3, dt))), env, command, t + dt);
            next = y0.map((value, i) => value + dt * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6);
        }
        stateFromVector(state, next);
        const d = this.derivative(state, env, command, t + dt);
        state.acceleration = {uDot: d[7], vDot: d[8], wDot: d[9]};
        state.angularAccel = {pDot: d[10], qDot: d[11], rDot: d[12]};
        return state;
    }
}
