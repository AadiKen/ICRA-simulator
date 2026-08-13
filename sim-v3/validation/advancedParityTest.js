import {
    ActuationModel,
    ControlSurface,
    FixedThruster,
    Rotor,
    allocateLeastSquares,
    allocationMatrix
} from "../core/forces/actuatorModel.js";
import {WindLoad} from "../core/forces/windLoad.js";
import {
    generateSamples,
    primitiveSolidVolume
} from "../core/forces/submergedGeometry.js";
import {
    coriolisFromMass6,
    normalizeQuaternion,
    quaternionDerivative,
    restoringWrench6,
    totalMassMatrix6
} from "../core/sixDof.js";
import {VehicleParameters} from "../core/vehicleParameters.js";
import {RigidBodyState} from "../core/rigidBodyState.js";

function assert(condition, message) {
    if (!condition) {
        throw new Error(message);
    }
}

function approx(value, expected, tolerance, message) {
    assert(Math.abs(value - expected) <= tolerance, `${message}: expected ${expected}, got ${value}`);
}

function approxVec(actual, expected, tolerance, message) {
    actual.forEach((value, idx) => approx(value, expected[idx], tolerance, `${message}[${idx}]`));
}

function simpleParams(overrides = {}) {
    return VehicleParameters.fromGeometry(2, 2, 0.2, 20, {
        maxThrust: 10,
        motorTimeConstant: 0.1,
        ...overrides
    });
}

function testAnalyticPrimitiveVolumesAndSamples() {
    const cylinder = {type: "cylinder", dims: {length: 2, radius: 0.5}, offset: {pos: [0, 0, 0]}};
    const ellipsoid = {type: "ellipsoid", dims: {length: 2, beam: 1, height: 0.5}, offset: {pos: [0, 0, 0]}};
    const capsule = {type: "capsule", dims: {length: 3, radius: 0.4}, offset: {pos: [0, 0, 0]}};
    const cone = {type: "cone", dims: {length: 2, radius: 0.5}, offset: {pos: [0, 0, 0]}};
    approx(primitiveSolidVolume(cylinder), Math.PI * 0.25 * 2, 1e-12, "Cylinder analytic volume");
    approx(primitiveSolidVolume(ellipsoid), (4 / 3) * Math.PI * 1 * 0.5 * 0.25, 1e-12, "Ellipsoid analytic volume");
    assert(primitiveSolidVolume(capsule) > Math.PI * 0.4 * 0.4 * (3 - 0.8), "Capsule should include endcap volume.");
    assert(primitiveSolidVolume(cone) < primitiveSolidVolume(cylinder), "Cone volume should be one third of same-radius cylinder.");
    assert(JSON.stringify(generateSamples(cylinder, 32, 5)) === JSON.stringify(generateSamples(cylinder, 32, 5)), "Non-box samples should be deterministic.");
}

function testControlSurfaceV2LawAndStall() {
    const params = simpleParams();
    const rudder = new ControlSurface({
        id: "rudder",
        type: "ControlSurface",
        pos: [-0.8, 0, 0],
        forceAxis: [0, 1, 0],
        dynamics: {tau: 0.1, min: -0.5, max: 0.5},
        foil: {A: 0.2, C_Lalpha: 4, C_D0: 0.01, k: 0.02, alphaStall: 0.3}
    }, params);
    rudder.advance(0, {deflection: 0.1});
    const slow = rudder.wrench(params, new RigidBodyState({velocity: {u: 1, v: 0, w: 0}}));
    const fast = rudder.wrench(params, new RigidBodyState({velocity: {u: 2, v: 0, w: 0}}));
    const stopped = rudder.wrench(params, new RigidBodyState({velocity: {u: 0, v: 0, w: 0}}));
    approx(fast[1] / slow[1], 4, 1e-9, "Control-surface lift should scale with V^2.");
    approxVec(stopped, [0, 0, 0, 0, 0, 0], 1e-12, "Control surface has no authority at zero inflow.");
    assert(Math.abs(rudder.liftCoefficient(0.3)) > Math.abs(rudder.liftCoefficient(0.9)), "Lift coefficient should decay after stall.");
}

function testAllocationRoundTripAndSaturation() {
    const params = simpleParams({
        effectors: [
            {id: "port", type: "FixedThruster", pos: [0, -1, 0], axis: [1, 0, 0], dynamics: {tau: 0.1, min: -10, max: 10}},
            {id: "starboard", type: "FixedThruster", pos: [0, 1, 0], axis: [1, 0, 0], dynamics: {tau: 0.1, min: -10, max: 10}}
        ],
        controlledDOF: ["surge", "yaw"],
        allocator: {saturation: "scale"}
    });
    const effectors = [
        new FixedThruster(params.effectors[0], params),
        new FixedThruster(params.effectors[1], params)
    ];
    const desired = [10, 0, 0, 0, 0, 4];
    const commands = allocateLeastSquares(effectors, params, desired, {controlledDOF: ["surge", "yaw"], saturation: "scale"});
    const b = allocationMatrix(effectors, params, ["surge", "yaw"]);
    const achieved = b.map((row) => row.reduce((sum, value, idx) => sum + value * commands[idx], 0));
    approxVec(achieved, [10, 4], 1e-6, "Allocator should round-trip reachable surge/yaw demand.");
    const saturated = allocateLeastSquares(effectors, params, [1000, 0, 0, 0, 0, 0], {controlledDOF: ["surge", "yaw"], saturation: "scale"});
    assert(saturated.every((value) => Math.abs(value) <= 1), "Scaled saturation should keep commands inside [-1,1].");
}

function testDifferentialVsRudderLowSpeedAuthority() {
    const diff = new ActuationModel(simpleParams());
    const yaw = diff.commandWrench({tauDes: [0, 0, 6]}, 0);
    approx(yaw[2], 6, 1e-12, "Differential thrusters retain yaw authority at zero speed.");
    const params = simpleParams({
        effectors: [{
            id: "rudder",
            type: "ControlSurface",
            pos: [-0.8, 0, 0],
            forceAxis: [0, 1, 0],
            dynamics: {tau: 0.1, min: -0.5, max: 0.5},
            foil: {A: 0.2, C_Lalpha: 4, C_D0: 0.01, k: 0.02, alphaStall: 0.3}
        }],
        controlledDOF: ["yaw"]
    });
    const rudder = new ActuationModel(params);
    const rudderYaw = rudder.commandWrench({effectors: {rudder: {deflection: 0.3}}}, 0, new RigidBodyState());
    approx(rudderYaw[2], 0, 1e-12, "Rudder yaw authority should vanish at zero inflow.");
}

function testRotorReactionTorque() {
    const params = simpleParams();
    const rotor = new Rotor({
        id: "rotor",
        type: "Rotor",
        pos: [1, 0, 0],
        axis: [0, 0, -1],
        dynamics: {tau: 0.1, min: 0, max: 100},
        conversion: {k_T: 0.01, k_Q: 0.002},
        spinDirection: 1
    }, params);
    rotor.advance(0, {speed: 10});
    const wrench = rotor.wrench(params);
    assert(wrench[2] < 0, "Rotor thrust should follow configured axis.");
    assert(wrench[5] > 0, "Offset rotor should create yaw/pitch/roll moment through r cross f.");
    assert(Math.abs(wrench[2]) > Math.abs(wrench[5]) * 0.1, "Rotor force and reaction torque should both be finite.");
}

function testSixDofHelpers() {
    const params = {
        massProps: {
            mass: 20,
            cg: {x: 0, y: 0, z: 0},
            inertia: {Ix: 2, Iy: 3, Iz: 4}
        },
        addedMass: {XuDot: -1, YvDot: -2, ZwDot: -3, KpDot: -0.2, MqDot: -0.3, NrDot: -0.4},
        buoyancy: {rho: 1000, g: 10},
        restoring: {displacementVolume: 20 / 1000}
    };
    const mass = totalMassMatrix6(params);
    assert(mass.length === 6 && mass[0].length === 6, "6-DOF mass matrix should be 6x6.");
    const c = coriolisFromMass6(mass, [1, 2, 3, 0.1, 0.2, 0.3]);
    for (let r = 0; r < 6; r += 1) {
        for (let col = 0; col < 6; col += 1) {
            approx(c[r][col] + c[col][r], 0, 1e-9, "6-DOF Coriolis should be skew-symmetric.");
        }
    }
    approxVec(restoringWrench6(params, {roll: 0, pitch: 0}, {totalVolume: 20 / 1000, cobBody: [0, 0, 0]}), [0, 0, 0, 0, 0, 0], 1e-9, "Neutral upright restoring wrench should be zero.");
    const qDot = quaternionDerivative({w: 1, x: 0, y: 0, z: 0}, {p: 0, q: 0, r: 2});
    approx(qDot.z, 1, 1e-12, "Quaternion derivative should map yaw rate into z component at identity.");
    approx(Math.hypot(...Object.values(normalizeQuaternion({w: 2, x: 0, y: 0, z: 0}))), 1, 1e-12, "Quaternion normalization should produce unit length.");
}

function testWindLoad() {
    const params = simpleParams();
    params.wind = {enabled: true, rhoAir: 1.2, frontalArea: 2, lateralArea: 4, C_X: 1, C_Y: 0.5, C_N: 0.1};
    const model = new WindLoad();
    const wrench = model.computeWrench({
        params,
        state: RigidBodyState.fromYaw({N: 0, E: 0, D: 0}, 0),
        env: {wind: {u: 2, v: 1}}
    });
    assert(wrench[0] > 0 && wrench[1] > 0 && wrench[2] > 0, "Wind load should produce configured surge, sway, and yaw.");
}

const tests = [
    testAnalyticPrimitiveVolumesAndSamples,
    testControlSurfaceV2LawAndStall,
    testAllocationRoundTripAndSaturation,
    testDifferentialVsRudderLowSpeedAuthority,
    testRotorReactionTorque,
    testSixDofHelpers,
    testWindLoad
];

try {
    const results = tests.map((test) => {
        test();
        return test.name;
    });
    console.log("Advanced parity tests passed.");
    console.log(JSON.stringify({tests: results}, null, 2));
} catch (error) {
    console.error("Advanced parity tests failed.");
    console.error(error.stack || error.message);
    process.exitCode = 1;
}
