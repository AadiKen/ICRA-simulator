export function skew(v) {
    const [x, y, z] = v;
    return [
        [0, -z, y],
        [z, 0, -x],
        [-y, x, 0]
    ];
}

export function rotationBodyToNed({roll = 0, pitch = 0, yaw = 0} = {}) {
    const cr = Math.cos(roll), sr = Math.sin(roll);
    const cp = Math.cos(pitch), sp = Math.sin(pitch);
    const cy = Math.cos(yaw), sy = Math.sin(yaw);
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]
    ];
}

export function eulerRateMatrix({roll = 0, pitch = 0} = {}) {
    const sr = Math.sin(roll), cr = Math.cos(roll);
    const ct = Math.cos(pitch);
    const tt = Math.tan(pitch);
    const safeCt = Math.abs(ct) < 1e-9 ? Math.sign(ct || 1) * 1e-9 : ct;
    return [
        [1, sr * tt, cr * tt],
        [0, cr, -sr],
        [0, sr / safeCt, cr / safeCt]
    ];
}

export function quaternionDerivative(q, angularRate) {
    const {w, x, y, z} = q;
    const p = angularRate.p || 0;
    const qq = angularRate.q || 0;
    const r = angularRate.r || 0;
    return {
        w: -0.5 * (x * p + y * qq + z * r),
        x: 0.5 * (w * p + y * r - z * qq),
        y: 0.5 * (w * qq + z * p - x * r),
        z: 0.5 * (w * r + x * qq - y * p)
    };
}

export function normalizeQuaternion(q) {
    const mag = Math.hypot(q.w || 0, q.x || 0, q.y || 0, q.z || 0) || 1;
    return {
        w: (q.w || 0) / mag,
        x: (q.x || 0) / mag,
        y: (q.y || 0) / mag,
        z: (q.z || 0) / mag
    };
}

export {rigidBodyMassMatrix6, addedMassMatrix6, addMassMatrices as add6x6, totalMassMatrix6} from "../packages/core/src/mass.ts";

export {coriolisFromMass6} from "../packages/core/src/coriolis.ts";

export {restoringWrench6} from "../packages/core/src/hydrostatics.ts";
