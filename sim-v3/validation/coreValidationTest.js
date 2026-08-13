import {RigidBodyState} from "../core/rigidBodyState.js";
import {VehicleParameters} from "../core/vehicleParameters.js";
import {DynamicsCore} from "../core/dynamicsCore.js";
import {AddedMassCoriolis, addedMassCoriolis3, rigidBodyCoriolis3} from "../core/forces/addedMassCoriolis.js";
import {HydrodynamicDamping} from "../core/forces/hydrodynamicDamping.js";
import {ActuatorModel} from "../core/forces/actuatorModel.js";
import {createOtterParameters} from "../core/vehicles/otter.js";
import {assertSkewSymmetric} from "../core/math.js";
import {constantThrustManeuver, runManeuver, turningCircleManeuver, zigZagManeuver} from "./maneuvers.js";

function assert(condition, message) {
    if (!condition) {
        throw new Error(message);
    }
}

function buildCore(params) {
    const actuator = new ActuatorModel(params);
    return new DynamicsCore(params, [
        actuator,
        new AddedMassCoriolis(),
        new HydrodynamicDamping()
    ]);
}

function testCoriolisSkewSymmetry() {
    const params = createOtterParameters();
    const velocity = [1.2, -0.35, 0.18];
    const cRb = rigidBodyCoriolis3(params, velocity);
    const cA = addedMassCoriolis3(params, velocity);
    const combined = cRb.map((row, r) => row.map((value, c) => value + cA[r][c]));

    assert(assertSkewSymmetric(cRb), "Rigid-body Coriolis matrix must be skew-symmetric.");
    assert(assertSkewSymmetric(cA), "Added-mass Coriolis matrix must be skew-symmetric.");
    assert(assertSkewSymmetric(combined), "Combined Coriolis matrix must be skew-symmetric.");
}

function testOtterStraightLineDeterminism() {
    const params = createOtterParameters();
    const env = {waterV: {x: 0, y: 0, z: 0}, hullWaterSamples: []};
    const dt = 0.05;
    const steps = 120;
    const a = runManeuver(buildCore(params), RigidBodyState.fromYaw({N: 0, E: 0, D: 0}, 0), env, constantThrustManeuver(60, 0), dt, steps);
    const b = runManeuver(buildCore(params), RigidBodyState.fromYaw({N: 0, E: 0, D: 0}, 0), env, constantThrustManeuver(60, 0), dt, steps);
    const lastA = a[a.length - 1];
    const lastB = b[b.length - 1];

    assert(Math.abs(lastA.N - lastB.N) < 1e-12, "Otter straight-line maneuver should be deterministic.");
    assert(lastA.N > 3, "Otter straight-line maneuver should advance north under surge thrust.");
    assert(Math.abs(lastA.E) < 0.05, "Otter straight-line maneuver should not drift laterally in still water.");
}

function testOtterTurningCircle() {
    const params = createOtterParameters();
    const env = {waterV: {x: 0, y: 0, z: 0}, hullWaterSamples: []};
    const samples = runManeuver(
        buildCore(params),
        RigidBodyState.fromYaw({N: 0, E: 0, D: 0}, 0),
        env,
        turningCircleManeuver(65, 35),
        0.05,
        160
    );
    const last = samples[samples.length - 1];

    assert(Math.abs(last.r) > 0.05, "Turning-circle maneuver should develop yaw rate.");
    assert(Math.abs(last.E) > 0.2, "Turning-circle maneuver should move east/west under differential thrust.");
}

function testCurrentCouplingChangesTrack() {
    const params = createOtterParameters();
    const still = {waterV: {x: 0, y: 0, z: 0}, hullWaterSamples: []};
    const current = {waterV: {x: 0.3, y: 0, z: 0}, hullWaterSamples: []};
    const command = constantThrustManeuver(55, 0);
    const stillSamples = runManeuver(buildCore(params), RigidBodyState.fromYaw({N: 0, E: 0, D: 0}, 0), still, command, 0.05, 120);
    const currentSamples = runManeuver(buildCore(params), RigidBodyState.fromYaw({N: 0, E: 0, D: 0}, 0), current, command, 0.05, 120);
    const stillLast = stillSamples[stillSamples.length - 1];
    const currentLast = currentSamples[currentSamples.length - 1];

    assert(
        Math.hypot(currentLast.E - stillLast.E, currentLast.N - stillLast.N) > 0.25,
        "Water current should alter the world track through relative-velocity coupling."
    );
}

function testZigZagRemainsFinite() {
    const params = createOtterParameters();
    const env = {waterV: {x: 0, y: 0, z: 0}, hullWaterSamples: []};
    const samples = runManeuver(
        buildCore(params),
        RigidBodyState.fromYaw({N: 0, E: 0, D: 0}, 0),
        env,
        zigZagManeuver(60, 28, 2),
        0.05,
        240
    );

    assert(samples.every((s) => Number.isFinite(s.N + s.E + s.yaw + s.u + s.v + s.r)), "Zig-zag samples must remain finite.");
}

const tests = [
    testCoriolisSkewSymmetry,
    testOtterStraightLineDeterminism,
    testOtterTurningCircle,
    testCurrentCouplingChangesTrack,
    testZigZagRemainsFinite
];

try {
    const results = tests.map((test) => {
        test();
        return test.name;
    });
    console.log("Core validation tests passed.");
    console.log(JSON.stringify({tests: results}, null, 2));
} catch (error) {
    console.error("Core validation tests failed.");
    console.error(error.stack || error.message);
    process.exitCode = 1;
}
