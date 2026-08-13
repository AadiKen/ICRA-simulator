export const PHYSICS_MODES = Object.freeze({
    PLANAR3: "planar3",
    COUPLED6: "coupled6"
});

export function normalizePhysicsMode(mode) {
    if (mode === PHYSICS_MODES.PLANAR3 || mode === PHYSICS_MODES.COUPLED6) return mode;
    throw new Error(`Unsupported physics mode "${mode}". Expected planar3 or coupled6.`);
}

export function validateStep(dt, state) {
    if (!Number.isFinite(dt) || dt <= 0) throw new Error(`Physics timestep must be finite and positive; got ${dt}.`);
    const values = [
        state.position.N, state.position.E, state.position.D,
        state.quaternion.w, state.quaternion.x, state.quaternion.y, state.quaternion.z,
        state.velocity.u, state.velocity.v, state.velocity.w,
        state.angularRate.p, state.angularRate.q, state.angularRate.r
    ];
    if (values.some((value) => !Number.isFinite(value))) {
        throw new Error("Marine plant state contains a non-finite value.");
    }
}

export function legacyWrenchToSix(wrench = []) {
    if (wrench.length >= 6) return wrench.slice(0, 6);
    return [wrench[0] || 0, wrench[1] || 0, 0, 0, 0, wrench[2] || 0];
}

export class MarinePlant {
    constructor(params, mode) {
        this.params = params;
        this.mode = normalizePhysicsMode(mode);
        this.lastWrench = Array(6).fill(0);
        this.forceBreakdown = {};
    }

    prepareStep() {}
    derivative() { throw new Error("MarinePlant.derivative() must be implemented."); }
    step() { throw new Error("MarinePlant.step() must be implemented."); }
}
