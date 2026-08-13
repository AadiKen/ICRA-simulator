import {RigidBodyState} from "../core/rigidBodyState.js";
import {VehicleParameters} from "../core/vehicleParameters.js";
import {
    CoupledSixPlant,
    analyticAxisAlignedBoxSubmersion,
    linearHydrostaticWrench
} from "../core/coupledSixPlant.js";
import {coriolisFromMass6, totalMassMatrix6} from "../core/sixDof.js";
import {assertSkewSymmetric, isPositiveDefinite, isSymmetric, matVecMul} from "../core/math.js";

function assert(condition, message) {
    if (!condition) throw new Error(message);
}

function approx(actual, expected, tolerance, message) {
    assert(Math.abs(actual - expected) <= tolerance, `${message}: expected ${expected}, got ${actual}`);
}

function params(options = {}) {
    return VehicleParameters.fromGeometry(2, 1, 0.2, 50, {
        height: 0.4,
        maxThrust: 100,
        ...options
    });
}

function calm() {
    return {waterV: {x: 0, y: 0, z: 0}, waveCoupling: "none"};
}

function testMassMatrixAndCoriolisInvariants() {
    const p = params({xg: 0.1, zg: 0.03});
    const mass = totalMassMatrix6(p);
    assert(isSymmetric(mass), "Total 6-DoF mass matrix must be symmetric.");
    assert(isPositiveDefinite(mass), "Total 6-DoF mass matrix must be positive definite.");
    const nu = [1.2, -0.4, 0.3, 0.1, -0.2, 0.25];
    const c = coriolisFromMass6(mass, nu);
    assert(assertSkewSymmetric(c, 1e-9), "Coriolis matrix must be skew symmetric.");
    const power = nu.reduce((sum, value, i) => sum + value * matVecMul(c, nu)[i], 0);
    approx(power, 0, 1e-9, "Coriolis force must do no work.");
    return {mass, coriolisPower: power, symmetric: true, positiveDefinite: true};
}

function testAnalyticBoxSubmersion() {
    const box = {type: "box", dims: {length: 2, beam: 1, height: 0.4}};
    const half = analyticAxisAlignedBoxSubmersion(box, 0, 0);
    approx(half.volume, 0.4, 1e-12, "Half-submerged box volume.");
    approx(half.centroidD, 0.1, 1e-12, "Half-submerged box centroid.");
    const full = analyticAxisAlignedBoxSubmersion(box, 0.3, 0);
    approx(full.volume, 0.8, 1e-12, "Fully submerged box volume.");
    return {halfVolume: half.volume, halfCentroidD: half.centroidD, fullVolume: full.volume};
}

function testHydrostaticDirections() {
    const p = params();
    const state = RigidBodyState.fromEuler({N: 0, E: 0, D: 0.1}, 0.1, -0.08, 0);
    const wrench = linearHydrostaticWrench(p, state, 0);
    assert(wrench[2] < 0, "Positive down displacement must create upward restoring force.");
    assert(wrench[3] < 0, "Positive roll must create negative roll restoring moment.");
    assert(wrench[4] > 0, "Negative pitch must create positive pitch restoring moment.");
    return {wrench};
}

function testStillWaterEquilibrium() {
    const plant = new CoupledSixPlant(params(), []);
    const state = RigidBodyState.fromYaw({N: 0, E: 0, D: 0}, 0);
    for (let i = 0; i < 600; i += 1) plant.step(state, calm(), {}, 0.1, i * 0.1);
    approx(state.velocity.w, 0, 1e-12, "Equilibrium heave velocity.");
    approx(state.position.D, 0, 1e-12, "Equilibrium draft.");
    approx(Math.hypot(state.quaternion.w, state.quaternion.x, state.quaternion.y, state.quaternion.z), 1, 1e-10, "Quaternion normalization.");
    return {positionD: state.position.D, velocityW: state.velocity.w, quaternion: state.quaternion};
}

function testHeaveRollPitchFreeDecay() {
    const plant = new CoupledSixPlant(params(), []);
    const state = RigidBodyState.fromEuler({N: 0, E: 0, D: 0.05}, 0.08, -0.06, 0);
    plant.equilibriumD = 0;
    const initial = Math.abs(state.position.D) + Math.abs(state.eulerAngles.roll) + Math.abs(state.eulerAngles.pitch);
    for (let i = 0; i < 300; i += 1) plant.step(state, calm(), {}, 0.02, i * 0.02);
    const final = Math.abs(state.position.D) + Math.abs(state.eulerAngles.roll) + Math.abs(state.eulerAngles.pitch);
    assert(final < initial, `Damped free decay must reduce displacement: initial=${initial}, final=${final}.`);
    assert([state.velocity.w, state.angularRate.p, state.angularRate.q].every(Number.isFinite), "Free decay must remain finite.");
    return {initial, final, positionD: state.position.D, roll: state.eulerAngles.roll, pitch: state.eulerAngles.pitch, velocityW: state.velocity.w, rollRate: state.angularRate.p, pitchRate: state.angularRate.q};
}

function testQuaternionConstantRate() {
    const p = params({Zw: 0, Kp: 0, Mq: 0, Nr: 0});
    p.damping.linear6 = Array(6).fill(0);
    p.damping.quadratic6 = Array(6).fill(0);
    p.restoring.waterplaneArea = 0;
    p.restoring.displacementVolume = 0;
    p.restoring.metacentricHeightRoll = 0;
    p.restoring.metacentricHeightPitch = 0;
    const plant = new CoupledSixPlant(p, []);
    const state = RigidBodyState.fromYaw({N: 0, E: 0, D: 0}, 0);
    state.angularRate.p = 0.2;
    for (let i = 0; i < 100; i += 1) plant.step(state, calm(), {}, 0.01, i * 0.01);
    approx(state.eulerAngles.roll, 0.2, 2e-4, "Constant roll-rate integration.");
    approx(Math.hypot(state.quaternion.w, state.quaternion.x, state.quaternion.y, state.quaternion.z), 1, 1e-10, "Quaternion norm.");
    return {roll: state.eulerAngles.roll, quaternion: state.quaternion};
}

function runDecay(dt, duration = 4) {
    const plant = new CoupledSixPlant(params(), []);
    const state = RigidBodyState.fromEuler({N: 0, E: 0, D: 0.04}, 0.06, -0.04, 0.1);
    plant.equilibriumD = 0;
    const steps = Math.round(duration / dt);
    for (let i = 0; i < steps; i += 1) plant.step(state, calm(), {}, dt, i * dt);
    return [state.position.D, state.eulerAngles.roll, state.eulerAngles.pitch, state.velocity.w, state.angularRate.p, state.angularRate.q];
}

function testTimestepConvergenceAndDeterminism() {
    const coarse = runDecay(0.04);
    const mediumA = runDecay(0.02);
    const mediumB = runDecay(0.02);
    const fine = runDecay(0.01);
    assert(mediumA.every((value, i) => value === mediumB[i]), "Repeated 6-DoF runs must be deterministic.");
    const coarseError = Math.hypot(...coarse.map((value, i) => value - fine[i]));
    const mediumError = Math.hypot(...mediumA.map((value, i) => value - fine[i]));
    assert(mediumError < coarseError, `Halving timestep must improve convergence: coarse=${coarseError}, medium=${mediumError}.`);
    return {coarse, medium: mediumA, fine, coarseError, mediumError};
}

function testLongEquilibriumRunRemainsFinite() {
    const plant = new CoupledSixPlant(params(), []);
    const state = RigidBodyState.fromYaw({N: 0, E: 0, D: 0}, 0);
    for (let i = 0; i < 3600; i += 1) plant.step(state, calm(), {}, 0.5, i * 0.5);
    const values = [
        state.position.N, state.position.E, state.position.D,
        state.velocity.u, state.velocity.v, state.velocity.w,
        state.angularRate.p, state.angularRate.q, state.angularRate.r
    ];
    assert(values.every(Number.isFinite), "Thirty-minute equilibrium run must remain finite.");
    assert(Math.abs(state.velocity.w) < 1e-5, "Thirty-minute equilibrium residual heave velocity.");
    return {values};
}

function testInvalidInputsAreRejected() {
    let rejectedMass = false;
    try {
        VehicleParameters.fromGeometry(2, 1, 0.2, 0);
    }
    catch {
        rejectedMass = true;
    }
    assert(rejectedMass, "Non-positive mass must be rejected.");
    const plant = new CoupledSixPlant(params(), []);
    const state = RigidBodyState.fromYaw({N: 0, E: 0, D: 0}, 0);
    let rejectedDt = false;
    try {
        plant.step(state, calm(), {}, 0, 0);
    }
    catch {
        rejectedDt = true;
    }
    assert(rejectedDt, "Non-positive timestep must be rejected.");
    return {rejectedMass, rejectedDt};
}

const tests = [
    testMassMatrixAndCoriolisInvariants,
    testAnalyticBoxSubmersion,
    testHydrostaticDirections,
    testStillWaterEquilibrium,
    testHeaveRollPitchFreeDecay,
    testQuaternionConstantRate,
    testTimestepConvergenceAndDeterminism,
    testLongEquilibriumRunRemainsFinite,
    testInvalidInputsAreRejected
];

try {
    const results = tests.map((test) => {
        const metrics = test();
        return {name: test.name, metrics};
    });
    console.log("Six-DoF plant tests passed.");
    console.log(JSON.stringify({tests: results}, null, 2));
}
catch (error) {
    console.error("Six-DoF plant tests failed.");
    console.error(error.stack || error.message);
    process.exitCode = 1;
}
