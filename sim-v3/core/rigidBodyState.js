import {eulerToQuaternion, normalizeQuaternion, quaternionToEuler, yawToQuaternion} from "./frames.js";

export class RigidBodyState {
    constructor({
        position = {N: 0, E: 0, D: 0},
        quaternion = {w: 1, x: 0, y: 0, z: 0},
        velocity = {u: 0, v: 0, w: 0},
        angularRate = {p: 0, q: 0, r: 0},
        acceleration = {uDot: 0, vDot: 0, wDot: 0},
        angularAccel = {pDot: 0, qDot: 0, rDot: 0}
    } = {}) {
        this.position = {...position};
        this.quaternion = normalizeQuaternion(quaternion);
        this.velocity = {...velocity};
        this.angularRate = {...angularRate};
        this.acceleration = {...acceleration};
        this.angularAccel = {...angularAccel};
    }

    static fromYaw(position, yaw) {
        return new RigidBodyState({position, quaternion: yawToQuaternion(yaw)});
    }

    static fromEuler(position, roll, pitch, yaw) {
        return new RigidBodyState({position, quaternion: eulerToQuaternion(roll, pitch, yaw)});
    }

    get eulerAngles() {
        return quaternionToEuler(this.quaternion);
    }

    get speedOverGround() {
        return Math.sqrt(this.velocity.u * this.velocity.u + this.velocity.v * this.velocity.v);
    }

    clone() {
        return new RigidBodyState({
            position: {...this.position},
            quaternion: {...this.quaternion},
            velocity: {...this.velocity},
            angularRate: {...this.angularRate},
            acceleration: {...this.acceleration},
            angularAccel: {...this.angularAccel}
        });
    }
}
