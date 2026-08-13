import {normalizeAngle} from "./math.js";

export function yawToQuaternion(yaw) {
    const half = yaw * 0.5;
    return {w: Math.cos(half), x: 0, y: 0, z: Math.sin(half)};
}

export function normalizeQuaternion(q) {
    const mag = Math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (mag === 0) {
        return {w: 1, x: 0, y: 0, z: 0};
    }
    return {w: q.w / mag, x: q.x / mag, y: q.y / mag, z: q.z / mag};
}

export function eulerToQuaternion(roll = 0, pitch = 0, yaw = 0) {
    const cy = Math.cos(yaw * 0.5);
    const sy = Math.sin(yaw * 0.5);
    const cp = Math.cos(pitch * 0.5);
    const sp = Math.sin(pitch * 0.5);
    const cr = Math.cos(roll * 0.5);
    const sr = Math.sin(roll * 0.5);

    return normalizeQuaternion({
        w: cr * cp * cy + sr * sp * sy,
        x: sr * cp * cy - cr * sp * sy,
        y: cr * sp * cy + sr * cp * sy,
        z: cr * cp * sy - sr * sp * cy
    });
}

export function quaternionToEuler(q) {
    const normalized = normalizeQuaternion(q);
    const sinrCosp = 2 * (normalized.w * normalized.x + normalized.y * normalized.z);
    const cosrCosp = 1 - 2 * (normalized.x * normalized.x + normalized.y * normalized.y);
    const roll = Math.atan2(sinrCosp, cosrCosp);

    const sinp = 2 * (normalized.w * normalized.y - normalized.z * normalized.x);
    const pitch = Math.abs(sinp) >= 1
        ? Math.sign(sinp) * Math.PI / 2
        : Math.asin(sinp);

    const sinyCosp = 2 * (normalized.w * normalized.z + normalized.x * normalized.y);
    const cosyCosp = 1 - 2 * (normalized.y * normalized.y + normalized.z * normalized.z);
    const yaw = normalizeAngle(Math.atan2(sinyCosp, cosyCosp));

    return {roll, pitch, yaw};
}

export function nedBodyToWorld2D(u, v, yaw) {
    const cos = Math.cos(yaw);
    const sin = Math.sin(yaw);
    return {
        N: cos * u - sin * v,
        E: sin * u + cos * v
    };
}

export function nedWorldToBody2D(north, east, yaw) {
    const cos = Math.cos(yaw);
    const sin = Math.sin(yaw);
    return {
        u: cos * north + sin * east,
        v: -sin * north + cos * east
    };
}
