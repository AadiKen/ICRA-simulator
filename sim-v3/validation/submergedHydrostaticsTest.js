import {HydrodynamicDamping} from "../core/forces/hydrodynamicDamping.js";
import {HydrostaticsAndWaves} from "../core/forces/hydrostaticsAndWaves.js";
import {
    computeSubmergedState,
    generateSamples,
    prepareHullPrimitive
} from "../core/forces/submergedGeometry.js";
import {positiveYawMomentDirection} from "../core/frameAdapters.js";
import {
    attitudeFluEnuToFrdNed,
    attitudeFrdNedToFluEnu,
    fluToFrd,
    frdToFlu
} from "../core/frameAdapters.js";
import {RigidBodyState} from "../core/rigidBodyState.js";
import {VehicleParameters, deriveDampingFromGeometry} from "../core/vehicleParameters.js";
import {
    scenarioConfig,
    simConfig,
    boatConfig,
    sensorConfig,
    envConfig,
    visibility,
    waterFieldConfig,
    goalConfig,
    controlConfig,
    simulator,
    vec3
} from "../schema.js";

function assert(condition, message) {
    if (!condition) {
        throw new Error(message);
    }
}

function approx(value, expected, tolerance, message) {
    assert(Math.abs(value - expected) <= tolerance, `${message}: expected ${expected}, got ${value}`);
}

function flatWaterField(height = 0, normal = {x: 0, y: 1, z: 0}) {
    return {
        sampleAt(pos) {
            return {
                pos,
                surfaceHeight: height,
                velocity: {x: 0, y: 0, z: 0},
                acceleration: {x: 0, y: 0, z: 0},
                normal,
                depth: height - pos.y,
                submerged: height > pos.y
            };
        }
    };
}

function boxPrimitive(sampleCount = 512, sampleSeed = 123) {
    return prepareHullPrimitive({
        type: "box",
        dims: {length: 4, beam: 2, height: 1},
        offset: {pos: [0, 0, 0], rot: [0, 0, 0]},
        sampleCount,
        sampleSeed
    });
}

function testFullySubmergedBoxVolume() {
    const primitive = boxPrimitive();
    const state = computeSubmergedState(
        [primitive],
        {position: {x: 0, y: -2, z: 0}, orientation: {x: 0, y: 0, z: 0}},
        flatWaterField(0),
        0
    );
    approx(state.totalVolume, primitive.solidVolume, primitive.solidVolume * 0.02, "Fully submerged volume should converge");
}

function testHalfSubmergedBoxVolumeAndCob() {
    const primitive = boxPrimitive();
    const state = computeSubmergedState(
        [primitive],
        {position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0}},
        flatWaterField(0),
        0
    );
    approx(state.totalVolume, primitive.solidVolume * 0.5, primitive.solidVolume * 0.06, "Half submerged box volume should converge");
    assert(state.cobBody[2] > 0, "Half-submerged box center of buoyancy should be in the submerged down half.");
}

function testSampleDeterminism() {
    const primitive = {type: "box", dims: {length: 1, beam: 1, height: 1}, offset: {pos: [0, 0, 0]}};
    const a = JSON.stringify(generateSamples(primitive, 16, 88));
    const b = JSON.stringify(generateSamples(primitive, 16, 88));
    assert(a === b, "Sample generation should be deterministic for identical seed/count.");
}

function testFrameAdaptersFollowUpdatedConvention() {
    const flu = frdToFlu([1, 2, 3]);
    assert(flu[0] === 1 && flu[1] === -2 && flu[2] === -3, "FRD to FLU should flip lateral and vertical axes.");
    const frd = fluToFrd(flu);
    assert(frd[0] === 1 && frd[1] === 2 && frd[2] === 3, "FLU to FRD should invert FRD to FLU.");
    const attitude = attitudeFrdNedToFluEnu({roll: 0.1, pitch: -0.2, yaw: 0.3});
    approx(attitude.roll, -0.1, 1e-12, "FRD roll should flip in FLU.");
    approx(attitude.pitch, 0.2, 1e-12, "FRD pitch should flip in FLU.");
    const roundTrip = attitudeFluEnuToFrdNed(attitude);
    approx(roundTrip.roll, 0.1, 1e-12, "Attitude roll should round-trip.");
    approx(roundTrip.pitch, -0.2, 1e-12, "Attitude pitch should round-trip.");
    approx(roundTrip.yaw, 0.3, 1e-12, "Attitude yaw should round-trip.");
}

function testSnameDampingNormalizesAtBoundary() {
    const params = VehicleParameters.fromCoefficientSet({
        id: "test",
        geometry: {length: 2, beam: 1, draft: 0.2, height: 0.5},
        massProps: {mass: 10, cg: {x: 0, y: 0, z: 0}, inertia: {Iz: 4}},
        addedMass: {XuDot: -1, YvDot: -2, NrDot: -3},
        damping: {
            signConvention: "sname",
            linear: {Xu: -4, Yv: -5, Nr: -6},
            quadratic: {Xuu: -7, Yvv: -8, Nrr: -9}
        },
        restoring: {waterDensity: 1025, gravity: 9.81, waterplaneArea: 2, displacementVolume: 10 / 1025},
        actuator: {beam: 1, maxThrust: 10, motorTimeConstant: 0.2}
    });
    assert(params.damping.signConvention === "resistancePositive", "SNAME damping should normalize for runtime use.");
    assert(params.damping.sourceSignConvention === "sname", "Original SNAME convention should be recorded.");
    assert(params.damping.linear.Xu === 4 && params.damping.quadratic.Nrr === 9, "SNAME negative damping should become positive resistance.");
}

function testDragDerivationUsesPrimitiveArea() {
    const base = {
        geometry: {length: 2, beam: 1, draft: 0.2},
        restoring: {waterDensity: 1000},
        damping: {
            drag: {Cd_surge: 1, Cd_sway: 1, Cd_yaw: 1},
            linear: {},
            quadratic: {}
        },
        hullPrimitives: [{
            type: "box",
            dims: {length: 2, beam: 1, height: 0.5},
            offset: {pos: [0, 0, 0]}
        }]
    };
    const doubled = {
        ...base,
        geometry: {length: 2, beam: 2, draft: 0.2},
        hullPrimitives: [{
            type: "box",
            dims: {length: 2, beam: 2, height: 0.5},
            offset: {pos: [0, 0, 0]}
        }]
    };
    const a = deriveDampingFromGeometry(base);
    const b = deriveDampingFromGeometry(doubled);
    approx(b.quadratic.Xuu, a.quadratic.Xuu * 2, 1e-12, "Doubling frontal primitive beam should double Xuu.");
}

function testHydrostaticsNoLeakInCalmWater() {
    const hydro = new HydrostaticsAndWaves();
    const wrench = hydro.computeWrench({
        state: RigidBodyState.fromYaw({N: 0, E: 0, D: 0}, 0),
        params: {
            buoyancy: {rho: 1025, g: 9.81},
            restoring: {waterDensity: 1025, gravity: 9.81},
            massProps: {mass: 1000, cg: {x: 0, y: 0, z: 0}}
        },
        env: {
            submergedState: {
                perPrimitive: [{
                    volume: 1000 / 1025,
                    centroidBody: [0, 0, 0],
                    surfaceNormalWorld: {x: 0, y: 1, z: 0}
                }]
            }
        }
    });
    approx(wrench[0], 0, 1e-9, "Calm upright hydrostatics should not leak surge");
    approx(wrench[1], 0, 1e-9, "Calm upright hydrostatics should not leak sway");
    approx(wrench[2], 0, 1e-9, "Calm upright hydrostatics should not leak yaw");
}

function testWaveForcingIncludesSurge() {
    const hydro = new HydrostaticsAndWaves();
    const wrench = hydro.computeWrench({
        state: RigidBodyState.fromYaw({N: 0, E: 0, D: 0}, 0),
        params: {
            buoyancy: {rho: 1025, g: 9.81},
            restoring: {waterDensity: 1025, gravity: 9.81},
            massProps: {mass: 1000, cg: {x: 0, y: 0, z: 0}}
        },
        env: {
            submergedState: {
                perPrimitive: [{
                    volume: 1,
                    centroidBody: [0, 0, 0],
                    surfaceNormalWorld: {x: 0.1, y: 0.98, z: 0.2}
                }]
            }
        }
    });
    assert(Math.abs(wrench[0]) > 1, "Tilted wave normal should create surge forcing.");
    assert(Math.abs(wrench[1]) > 1, "Tilted wave normal should create sway forcing.");
}

function testDampingScalarCompatibility() {
    const model = new HydrodynamicDamping();
    const wrench = model.computeWrench({
        relativeVelocityVector: [2, -3, 0.5],
        params: {
            damping: {
                linear: {Xu: 4, Yv: 5, Nr: 6},
                quadratic: {Xuu: 7, Yvv: 8, Nrr: 9}
            }
        }
    });
    approx(wrench[0], -(4 * 2 + 7 * 4), 1e-12, "Scalar surge damping should match legacy behavior");
    approx(wrench[1], -((5 * -3) + (8 * 3 * -3)), 1e-12, "Scalar sway damping should match legacy behavior");
    approx(wrench[2], -((6 * 0.5) + (9 * 0.5 * 0.5)), 1e-12, "Scalar yaw damping should match legacy behavior");
}

function testDampingMatrixMode() {
    const model = new HydrodynamicDamping();
    const wrench = model.computeWrench({
        relativeVelocityVector: [2, 3, 0],
        params: {
            damping: {
                linear: {},
                quadratic: {},
                linearMatrix: [
                    [4, 1, 0],
                    [2, 5, 0],
                    [0, 0, 6]
                ],
                quadraticMatrix: [
                    [1, 0, 0],
                    [0, 2, 0],
                    [0, 0, 3]
                ]
            }
        }
    });
    approx(wrench[0], -15, 1e-12, "Matrix damping should include coupled linear and quadratic surge terms.");
    approx(wrench[1], -37, 1e-12, "Matrix damping should include coupled linear and quadratic sway terms.");
}

function buildScenario() {
    return new scenarioConfig(
        new simConfig(10, 1, 1, true, "planar3"),
        new boatConfig(
            2.5, 0.9, 0.9, 0.55, 0.08, 0.12,
            new vec3(20, 0, 20),
            new vec3(0, 0, 0),
            new vec3(2.4, 1.0, 4.8),
            120
        ),
        new sensorConfig({}),
        new envConfig(
            100, 100, [], [], [],
            new waterFieldConfig([], new vec3(0, 0, 0)),
            new visibility(1, 1),
            "day"
        ),
        new goalConfig([new vec3(80, 0, 80)], 1),
        new controlConfig("local", 5, "none", 100, "relative")
    );
}

function testSubmergedStateSampledOncePerStep() {
    const sim = new simulator(buildScenario());
    let sampleCalls = 0;
    const original = sim.envModel.waterField.sampleAt.bind(sim.envModel.waterField);
    sim.envModel.waterField.sampleAt = (pos, time) => {
        sampleCalls += 1;
        return original(pos, time);
    };
    sim.step();
    const expectedMax = sim.boatModel.vehicleParameters.hullPrimitives[0].samples.length + 20;
    assert(sampleCalls <= expectedMax, "Submerged geometry should be sampled once before RK4, not during every derivative.");
}

function testYawSignRegression() {
    const signs = positiveYawMomentDirection();
    assert(signs.appLateralSign > 0, "Positive body sway maps to app-world +x at heading 0.");
    assert(signs.nedYawDeltaSign > 0, "Positive yaw moment increases NED yaw.");
    assert(signs.enuYawDeltaSign < 0, "Positive NED yaw maps to decreasing ENU heading.");
}

const tests = [
    testFullySubmergedBoxVolume,
    testHalfSubmergedBoxVolumeAndCob,
    testSampleDeterminism,
    testFrameAdaptersFollowUpdatedConvention,
    testSnameDampingNormalizesAtBoundary,
    testDragDerivationUsesPrimitiveArea,
    testHydrostaticsNoLeakInCalmWater,
    testWaveForcingIncludesSurge,
    testDampingScalarCompatibility,
    testDampingMatrixMode,
    testSubmergedStateSampledOncePerStep,
    testYawSignRegression
];

try {
    const results = tests.map((test) => {
        test();
        return test.name;
    });
    console.log("Submerged geometry and hydrostatics tests passed.");
    console.log(JSON.stringify({tests: results}, null, 2));
} catch (error) {
    console.error("Submerged geometry and hydrostatics tests failed.");
    console.error(error.stack || error.message);
    process.exitCode = 1;
}
