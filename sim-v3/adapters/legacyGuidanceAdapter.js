export function legacyGuidanceToActuatorCommand(guidance, params, boatConfig) {
    if (!guidance || !Number.isFinite(guidance.a) || !Number.isFinite(guidance.w) || !Number.isFinite(guidance.desiredSpeed)) {
        throw new Error("Guidance-to-actuator conversion requires finite acceleration, yaw, and desired-speed commands.");
    }
    const mass = params.massProps.mass;
    const maxThrust = params.actuator.maxThrust || mass;
    const surgeForce = Math.max(
        Math.min(guidance.a * mass, 2 * maxThrust),
        -2 * maxThrust
    );
    const yawAuthority = (boatConfig?.maxTurn || 0.5) * (params.massProps.inertia.Iz || 1);
    const differentialForce = Math.max(
        Math.min(guidance.w * yawAuthority / Math.max((params.actuator.beam || 1) * 0.5, 0.001), maxThrust),
        -maxThrust
    );

    return {
        surgeForce,
        differentialForce,
        desiredSpeed: guidance.desiredSpeed,
        target: guidance?.target || null
    };
}
