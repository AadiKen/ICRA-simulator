export function coreStateToPlannerRow(coreState, t) {
    const euler = coreState.eulerAngles;
    return {
        time: t,
        north: coreState.position.N,
        east: coreState.position.E,
        down: coreState.position.D,
        surge: coreState.velocity.u,
        sway: coreState.velocity.v,
        yaw: euler.yaw,
        yawRate: coreState.angularRate.r,
        surgeAccel: coreState.acceleration.uDot,
        swayAccel: coreState.acceleration.vDot,
        yawAccel: coreState.angularAccel.rDot
    };
}
