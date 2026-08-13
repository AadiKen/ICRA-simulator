import fs from "node:fs";
import path from "node:path";
import {mkdtempSync} from "node:fs";
import {tmpdir} from "node:os";
import {
    addedMassMatrix3,
    assertAddedMassIsValid,
    renderModelConfig,
    renderModelSdf,
    renderWorldSdf,
    writeGenerated
} from "../gazebo/generateGazeboParity.js";
import {otterCoefficients} from "../core/vehicles/coefficients.js";
import {getParityManeuver} from "../gazebo/parityManeuvers.js";

function assert(condition, message) {
    if (!condition) {
        throw new Error(message);
    }
}

function testModelContainsHydrodynamicsAndStablePlantActuation() {
    const sdf = renderModelSdf(otterCoefficients);
    assert(sdf.includes("gz::sim::systems::Hydrodynamics"), "Model SDF should include hydrodynamics plugin.");
    assert(!sdf.includes("gz::sim::systems::Buoyancy"), "Model SDF should not include world-scoped buoyancy plugin.");
    assert(sdf.includes("gz::sim::systems::PosePublisher"), "Model SDF should include model-scoped pose publisher plugin.");
    assert(!sdf.includes("gz::sim::systems::Thruster"), "Plant parity SDF should not depend on fragile fixed-thruster propeller links.");
    assert(!sdf.includes("<joint name=\"port_joint\""), "Plant parity SDF should not emit fixed-thruster child joints.");
    assert(!sdf.includes("gz::sim::systems::JointController"), "Plant parity SDF should avoid joint controllers.");
    assert(!sdf.includes("<fluid_added_mass>"), "Model SDF should omit DART fluid_added_mass until Gazebo can simulate the valid MSS matrix without pose resets.");
    assert(sdf.includes("validates integration, frames, thrust mapping, and drag, but not added mass"), "SDF should document the Gazebo oracle scope while added mass is omitted.");
    assert(!sdf.includes("<xDotU>"), "Hydrodynamics plugin should not double-count added mass through legacy xDotU.");
    assert(!sdf.includes("<yDotV>"), "Hydrodynamics plugin should not double-count added mass through legacy yDotV.");
    assert(!sdf.includes("<nDotR>"), "Hydrodynamics plugin should not double-count added mass through legacy nDotR.");
}

function testAddedMassMatrixIsSymmetricAndTotalInertiaSpd() {
    const matrix = addedMassMatrix3(otterCoefficients);
    assert(Math.abs(matrix[0][0] - 5.28152100957639) < 1e-12, "Added mass surge diagonal should match pinned MSS.");
    assert(matrix[1][1] === 82.5, "Added mass sway diagonal should be positive.");
    assert(matrix[2][2] === 23.375, "Added mass yaw diagonal should match pinned MSS.");
    assertAddedMassIsValid(otterCoefficients);
}

function testWorldPinsStepSizeAndMetadata() {
    const maneuver = {...getParityManeuver("current-drift"), name: "current-drift"};
    const sdf = renderWorldSdf(otterCoefficients, maneuver);
    assert(sdf.includes("<max_step_size>0.05</max_step_size>"), "World SDF should pin max_step_size from maneuver dt.");
    assert(sdf.includes("<pose>0 0 0 0 0 1.5707963268</pose>"), "World SDF should start Gazebo ENU yaw at pi/2 for BCOD NED yaw zero.");
    assert(sdf.includes("gz::sim::systems::Physics"), "World SDF should include the Physics system plugin.");
    assert(sdf.includes("gz::sim::systems::ApplyLinkWrench"), "World SDF should include the ApplyLinkWrench plugin for plant-level open-loop actuation.");
    assert(sdf.includes("gz::sim::systems::Buoyancy"), "World SDF should include world-scoped buoyancy plugin.");
    assert(sdf.includes("<graded_buoyancy>"), "World SDF should configure the graded water/air density model.");
    assert(sdf.includes("<above_depth>0</above_depth>"), "World SDF should place the water surface at ENU z=0.");
    assert(sdf.includes("<density>0</density>"), "World SDF should not apply water-density buoyancy above the surface.");
    assert(!sdf.includes("<uniform_fluid_density>"), "Surface-vessel worlds must not use an unbounded uniform fluid.");
    assert(sdf.includes("<enable>otter</enable>"), "World SDF should explicitly enable buoyancy for the parity vessel.");
    assert(!sdf.includes("gz::sim::systems::PosePublisher"), "World SDF should not include model-scoped pose publisher plugin.");
    assert(sdf.includes("current ENU 0.3 0 0"), "World SDF should record east current in ENU coordinates.");
}

function testModelConfigIsGenerated() {
    const config = renderModelConfig(otterCoefficients);
    assert(config.includes("<name>otter</name>"), "model.config should name the model.");
    assert(config.includes("<sdf version=\"1.10\">model.sdf</sdf>"), "model.config should reference model.sdf.");
}

function testGeneratorWritesManifestModelAndWorld() {
    const outDir = mkdtempSync(path.join(tmpdir(), "bcod-gazebo-"));
    const result = writeGenerated({vehicle: "otter", maneuver: "turning-circle", outDir}, "turning-circle");
    assert(fs.existsSync(result.model), "Generator should write model.sdf.");
    assert(fs.existsSync(result.modelConfig), "Generator should write model.config.");
    assert(fs.existsSync(result.world), "Generator should write world.sdf.");
    assert(fs.existsSync(result.manifest), "Generator should write manifest JSON.");
    const manifest = JSON.parse(fs.readFileSync(result.manifest, "utf8"));
    assert(manifest.expectedGoldenCsv === "golden/otter/turning-circle.csv", "Manifest should point at expected frozen golden CSV path.");
    assert(manifest.gazebo.actuationMode === "bodyWrench", "Manifest should use plant-level body wrench capture mode.");
    assert(manifest.gazebo.commandTopics.length === 1, "Manifest should expose the net wrench command topic.");
    assert(manifest.gazebo.wrenchTopic === "/world/bcod_parity_turning-circle/wrench", "Manifest should retain the ApplyLinkWrench base wrench topic.");
    assert(manifest.gazebo.wrenchPersistentTopic === "/world/bcod_parity_turning-circle/wrench/persistent", "Manifest should expose the persistent wrench topic.");
    assert(manifest.gazebo.commandTopics[0] === "/world/bcod_parity_turning-circle/wrench/persistent", "Manifest should command ApplyLinkWrench persistent wrench topic.");
    assert(manifest.gazebo.evidenceScope === "planar maneuver implementation cross-check only", "Manifest must scope the stabilized Gazebo fixture honestly.");
    assert(manifest.gazebo.coupled6HydrostaticsValidated === false, "Gazebo planar fixture must not claim coupled6 hydrostatic validation.");
    assert(manifest.gazebo.fixtureStabilization.method.includes("vertical CG"), "Manifest must record the Gazebo fixture stabilization method.");
    const currentResult = writeGenerated({vehicle: "otter", maneuver: "current-drift", outDir}, "current-drift");
    const currentManifest = JSON.parse(fs.readFileSync(currentResult.manifest, "utf8"));
    assert(currentManifest.env.waterV.x === 0.3, "Current-drift manifest should preserve its configured east current.");
    assert(currentManifest.gazebo.currentEnu.x === 0.3, "East current should map to positive Gazebo ENU x.");
}

function testGeneratorEmitsPrimitiveAndEffectorMetadata() {
    const sdf = renderModelSdf({
        ...otterCoefficients,
        hullPrimitives: [
            {type: "box", dims: {length: 1, beam: 1, height: 0.2}, offset: {pos: [0, 0, 0], rot: [0, 0, 0]}},
            {type: "cylinder", dims: {length: 0.5, radius: 0.1}, offset: {pos: [0.4, 0, 0], rot: [0, 0, 0]}}
        ],
        effectors: [
            {id: "rudder", type: "ControlSurface", pos: [-0.5, 0, 0], foil: {A: 0.1, C_Lalpha: 4, C_D0: 0.01}},
            {id: "rotor", type: "Rotor", pos: [0, 0, 0], axis: [0, 0, -1], conversion: {k_T: 1, k_Q: 0.1}}
        ]
    });
    assert(sdf.includes("hull_collision_1"), "Model SDF should emit each configured hull primitive.");
    assert(sdf.includes("<cylinder>"), "Model SDF should preserve non-box primitive geometry.");
    assert(sdf.includes("gz::sim::systems::LiftDrag"), "Control surfaces should map to LiftDrag.");
    assert(sdf.includes("gz::sim::systems::MulticopterMotorModel"), "Rotors should map to MulticopterMotorModel.");
}

const tests = [
    testModelContainsHydrodynamicsAndStablePlantActuation,
    testAddedMassMatrixIsSymmetricAndTotalInertiaSpd,
    testWorldPinsStepSizeAndMetadata,
    testModelConfigIsGenerated,
    testGeneratorWritesManifestModelAndWorld,
    testGeneratorEmitsPrimitiveAndEffectorMetadata
];

try {
    const results = tests.map((test) => {
        test();
        return test.name;
    });
    console.log("Gazebo generator tests passed.");
    console.log(JSON.stringify({tests: results}, null, 2));
} catch (error) {
    console.error("Gazebo generator tests failed.");
    console.error(error.stack || error.message);
    process.exitCode = 1;
}
