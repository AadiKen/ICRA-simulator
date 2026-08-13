import {normalizeQuaternion, yawToQuaternion} from "./frames.js";
import {addVec, scaleVec} from "./math.js";

function stateToVector(state) {
    const euler = state.eulerAngles;
    return [
        state.position.N,
        state.position.E,
        euler.yaw,
        state.velocity.u,
        state.velocity.v,
        state.angularRate.r
    ];
}

function vectorToState(state, vector) {
    state.position.N = vector[0];
    state.position.E = vector[1];
    state.quaternion = normalizeQuaternion(yawToQuaternion(vector[2]));
    state.velocity.u = vector[3];
    state.velocity.v = vector[4];
    state.angularRate.r = vector[5];
    return state;
}

function derivativeAt(core, state, env, command, t) {
    return core.derivative(state, env, command, t);
}

export function stepRK4(core, state, env, command, dt, t = 0) {
    const y0 = stateToVector(state);
    const s1 = vectorToState(state.clone(), y0);
    const k1 = derivativeAt(core, s1, env, command, t);

    const s2 = vectorToState(state.clone(), addVec(y0, scaleVec(k1, dt * 0.5)));
    const k2 = derivativeAt(core, s2, env, command, t + dt * 0.5);

    const s3 = vectorToState(state.clone(), addVec(y0, scaleVec(k2, dt * 0.5)));
    const k3 = derivativeAt(core, s3, env, command, t + dt * 0.5);

    const s4 = vectorToState(state.clone(), addVec(y0, scaleVec(k3, dt)));
    const k4 = derivativeAt(core, s4, env, command, t + dt);

    const next = y0.map((value, idx) => {
        return value + (dt / 6) * (k1[idx] + 2 * k2[idx] + 2 * k3[idx] + k4[idx]);
    });

    return vectorToState(state, next);
}

export function stepSemiImplicitEuler(core, state, env, command, dt, t = 0) {
    const d = derivativeAt(core, state, env, command, t);
    state.velocity.u += d[3] * dt;
    state.velocity.v += d[4] * dt;
    state.angularRate.r += d[5] * dt;
    state.position.N += d[0] * dt;
    state.position.E += d[1] * dt;
    state.quaternion = yawToQuaternion(state.eulerAngles.yaw + d[2] * dt);
    return state;
}
