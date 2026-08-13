import {
    Wave,
    waterFieldConfig,
    waterFieldModel,
    waveConfig,
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

function testParityLinearUsesDispersion() {
    const wavelength = 10;
    const wave = new Wave(0, 2, wavelength, 999, 0, {mode: "parityLinear", gravity: 9.81});
    const expectedK = 2 * Math.PI / wavelength;
    approx(wave.k, expectedK, 1e-12, "Parity wave should compute wavenumber from wavelength.");
    approx(wave.omega, Math.sqrt(9.81 * expectedK), 1e-12, "Parity wave should use deep-water dispersion.");
    return {wavenumber: wave.k, omega: wave.omega, expectedK};
}

function testParityLinearHeightAndNormal() {
    const field = new waterFieldModel(new waterFieldConfig(
        [new waveConfig(0, 2, 10, 0, 0, "parityLinear", 0)],
        new vec3(0, 0, 0),
        "parityLinear",
        9.81
    ));
    approx(field.heightAt(0, 0, 0), 1, 1e-12, "Parity wave height should use cosine phase.");
    approx(field.heightAt(2.5, 0, 0), 0, 1e-12, "Quarter wavelength should be at zero crossing.");
    const normal = field.normalAt(2.5, 0, 0);
    const expectedSlope = -1 * (2 * Math.PI / 10);
    const expectedX = -expectedSlope / Math.hypot(-expectedSlope, 1);
    const expectedY = 1 / Math.hypot(-expectedSlope, 1);
    approx(normal.x, expectedX, 1e-12, "Normal x should follow -dζ/dx.");
    approx(normal.y, expectedY, 1e-12, "Normal y should be normalized vertical component.");
    return {heightOrigin: field.heightAt(0, 0, 0), heightQuarter: field.heightAt(2.5, 0, 0), normal: {x: normal.x, y: normal.y, z: normal.z}};
}

function testParityLinearVelocityAndCurrent() {
    const current = new vec3(0.2, 0, -0.1);
    const field = new waterFieldModel(new waterFieldConfig(
        [new waveConfig(0, 2, 10, 0, 0, "parityLinear", 0)],
        current,
        "parityLinear",
        9.81
    ));
    const velocityAtCrest = field.velocityAt(0, 0, 0);
    const omega = Math.sqrt(9.81 * (2 * Math.PI / 10));
    approx(velocityAtCrest.x, omega + current.x, 1e-12, "Parity wave horizontal velocity should be deterministic and include current.");
    approx(velocityAtCrest.z, current.z, 1e-12, "Cross-wave velocity should include only current at heading 0.");
    const velocityAtQuarter = field.velocityAt(2.5, 0, 0);
    approx(velocityAtQuarter.y, omega, 1e-12, "Parity wave vertical velocity should follow sine phase.");
    return {velocityAtCrest: {x: velocityAtCrest.x, y: velocityAtCrest.y, z: velocityAtCrest.z}, velocityAtQuarter: {x: velocityAtQuarter.x, y: velocityAtQuarter.y, z: velocityAtQuarter.z}, omega};
}

function testLegacyModeRemainsDefault() {
    const field = new waterFieldModel(new waterFieldConfig(
        [new waveConfig(0, 2, 10, 1, 0.2)],
        new vec3(0, 0, 0)
    ));
    approx(field.heightAt(0, 0, 0), 0, 1e-12, "Legacy wave mode should keep sine phase at origin.");
    const velocity = field.velocityAt(0, 0, 0);
    approx(velocity.y, -field.waves[0].amplitude * field.waves[0].omega, 1e-12, "Legacy vertical velocity should remain unchanged.");
    return {heightOrigin: field.heightAt(0, 0, 0), velocity: {x: velocity.x, y: velocity.y, z: velocity.z}, omega: field.waves[0].omega};
}

const tests = [
    testParityLinearUsesDispersion,
    testParityLinearHeightAndNormal,
    testParityLinearVelocityAndCurrent,
    testLegacyModeRemainsDefault
];

try {
    const results = tests.map((test) => {
        const metrics = test();
        return {name: test.name, metrics};
    });
    console.log("Wave parity tests passed.");
    console.log(JSON.stringify({tests: results}, null, 2));
} catch (error) {
    console.error("Wave parity tests failed.");
    console.error(error.stack || error.message);
    process.exitCode = 1;
}
