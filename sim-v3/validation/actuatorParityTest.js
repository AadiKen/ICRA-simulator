import {ActuationModel, FixedThruster} from "../core/forces/actuatorModel.js";
import {VehicleParameters} from "../core/vehicleParameters.js";
import {createOtterParameters} from "../core/vehicles/otter.js";
import {DynamicsCore} from "../core/dynamicsCore.js";
import {RigidBodyState} from "../core/rigidBodyState.js";

function assert(condition, message) {
    if (!condition) {
        throw new Error(message);
    }
}

function approx(value, expected, tolerance, message) {
    assert(Math.abs(value - expected) <= tolerance, `${message}: expected ${expected}, got ${value}`);
}

function testFixedThrusterLinearCurveAndLag() {
    const params = createOtterParameters();
    const thruster = new FixedThruster({
        id: "test",
        type: "FixedThruster",
        pos: [0, 0, 0],
        axis: [1, 0, 0],
        dynamics: {tau: 0.5, min: -100, max: 100},
        conversion: {type: "linear"}
    }, params);

    thruster.advance(0.5, {command: 1});
    approx(thruster.thrust, 100 * (1 - Math.exp(-1)), 1e-9, "Thruster should reach 63% of final thrust at one time constant.");
    thruster.advance(0, {command: -1});
    approx(thruster.thrust, -100, 1e-12, "Zero-dt direct evaluations should produce steady thrust for open-loop plant tests.");
}

function testAppliedWrenchOverrideWins() {
    const model = new ActuationModel(createOtterParameters());
    const wrench = model.commandWrench({appliedWrench: [4, 5, 6]}, 0.1);
    assert(wrench[0] === 4 && wrench[1] === 5 && wrench[2] === 6, "appliedWrench should bypass effector allocation.");
}

function testPointThrusterWrenchFrame() {
    const params = VehicleParameters.fromGeometry(2, 2, 0.2, 20, {maxThrust: 100});
    const thruster = new FixedThruster({
        id: "starboard",
        type: "FixedThruster",
        pos: [0, 1, 0],
        axis: [1, 0, 0],
        dynamics: {tau: 0.1, min: -100, max: 100}
    }, params);
    thruster.advance(0, {thrust: 10});
    const wrench = thruster.wrench(params);
    approx(wrench[0], 10, 1e-12, "Point thruster should contribute body surge force.");
    approx(wrench[5], -10, 1e-12, "Moment should be r x f in FRD body coordinates.");
}

function testLegacySurgeDifferentialCompatibility() {
    const params = VehicleParameters.fromGeometry(2, 2, 0.2, 20, {maxThrust: 100, motorTimeConstant: 0.2});
    const model = new ActuationModel(params);
    const wrench = model.commandWrench({surgeForce: 80, differentialForce: 20}, 0);
    approx(wrench[0], 80, 1e-12, "Legacy surgeForce should map to total surge wrench.");
    approx(wrench[2], 20, 1e-12, "Legacy differentialForce should map to desired yaw moment through beam/2.");
}

function testEqualCommandsCreateSurgeNoYaw() {
    const model = new ActuationModel(createOtterParameters());
    const wrench = model.commandWrench({portCommand: 0.5, starboardCommand: 0.5}, 0);
    assert(wrench[0] > 0, "Equal positive thruster commands should create positive surge.");
    approx(wrench[2], 0, 1e-12, "Equal thruster commands should create near-zero yaw.");
}

function testDifferentialYawAuthorityAtZeroSpeed() {
    const model = new ActuationModel(createOtterParameters());
    const wrench = model.commandWrench({tauDes: [0, 0, 12]}, 0);
    approx(wrench[0], 0, 1e-12, "Pure yaw allocation should not require surge.");
    approx(wrench[2], 12, 1e-12, "Differential fixed thrusters should retain yaw authority at zero speed.");
    assert(model.lastEffectorCommands.port.thrust > model.lastEffectorCommands.starboard.thrust, "Positive yaw should allocate more port thrust than starboard thrust.");
}

function testDeterministicAllocation() {
    const params = createOtterParameters();
    const a = new ActuationModel(params);
    const b = new ActuationModel(params);
    const wa = a.commandWrench({tauDes: [40, 0, 8]}, 0.1);
    const wb = b.commandWrench({tauDes: [40, 0, 8]}, 0.1);
    approx(wa[0], wb[0], 1e-12, "Actuation allocation should be deterministic in surge.");
    approx(wa[2], wb[2], 1e-12, "Actuation allocation should be deterministic in yaw.");
}

function testRk4AdvancesActuatorOncePerOuterStep() {
    const params = VehicleParameters.fromGeometry(2, 2, 0.2, 20, {
        maxThrust: 100,
        motorTimeConstant: 0.5
    });
    const actuator = new ActuationModel(params);
    const core = new DynamicsCore(params, [actuator]);
    const state = RigidBodyState.fromYaw({N: 0, E: 0, D: 0}, 0);

    core.step(state, {waterV: {x: 0, y: 0, z: 0}}, {
        portCommand: 1,
        starboardCommand: 1
    }, 0.5, 0);

    const expected = 100 * (1 - Math.exp(-1));
    actuator.effectors.forEach((effector) => {
        approx(effector.thrust, expected, 1e-9, "RK4 must advance each actuator by exactly one outer timestep.");
    });
}

const tests = [
    testFixedThrusterLinearCurveAndLag,
    testAppliedWrenchOverrideWins,
    testPointThrusterWrenchFrame,
    testLegacySurgeDifferentialCompatibility,
    testEqualCommandsCreateSurgeNoYaw,
    testDifferentialYawAuthorityAtZeroSpeed,
    testDeterministicAllocation,
    testRk4AdvancesActuatorOncePerOuterStep
];

try {
    const results = tests.map((test) => {
        test();
        return test.name;
    });
    console.log("Actuator parity tests passed.");
    console.log(JSON.stringify({tests: results}, null, 2));
} catch (error) {
    console.error("Actuator parity tests failed.");
    console.error(error.stack || error.message);
    process.exitCode = 1;
}
