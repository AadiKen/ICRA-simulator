export function constantThrustManeuver(surgeForce, differentialForce = 0) {
    return () => ({surgeForce, differentialForce});
}

export function turningCircleManeuver(surgeForce, differentialForce) {
    return () => ({surgeForce, differentialForce});
}

export function yawTurnManeuver(surgeForce, yawMoment) {
    return () => ({surgeForce, differentialForce: 0, yawMoment});
}

export function zigZagManeuver(surgeForce, differentialForce, periodSec = 5) {
    return (t) => ({
        surgeForce,
        differentialForce: (Math.floor(t / periodSec) % 2 === 0 ? 1 : -1) * differentialForce
    });
}

export function runManeuver(core, state, env, commandAt, dt, steps) {
    const samples = [];
    for (let i = 0; i < steps; i += 1) {
        const t = i * dt;
        core.step(state, env, commandAt(t), dt, t);
        samples.push({
            t: t + dt,
            N: state.position.N,
            E: state.position.E,
            yaw: state.eulerAngles.yaw,
            u: state.velocity.u,
            v: state.velocity.v,
            r: state.angularRate.r
        });
    }
    return samples;
}
