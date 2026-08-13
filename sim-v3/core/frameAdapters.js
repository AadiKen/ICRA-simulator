import {nedBodyToWorld2D, nedWorldToBody2D} from "./frames.js";

export function nedToEnu(ned) {
    return {
        x: ned.E || 0,
        y: ned.N || 0,
        z: -(ned.D || 0)
    };
}

export function enuToNed(enu) {
    return {
        N: enu.y || 0,
        E: enu.x || 0,
        D: -(enu.z || 0)
    };
}

export function appWorldToNed(pos) {
    return {
        N: pos.z || 0,
        E: pos.x || 0,
        D: -(pos.y || 0)
    };
}

export function nedToAppWorld(pos) {
    return {
        x: pos.E || 0,
        y: -(pos.D || 0),
        z: pos.N || 0
    };
}

export function appWorldVectorToBody2D(vector, yaw) {
    const body = nedWorldToBody2D(vector.z || 0, vector.x || 0, yaw);
    return {
        x: body.u,
        y: body.v
    };
}

export function bodyVectorToAppWorld2D(u, v, yaw) {
    const world = nedBodyToWorld2D(u, v, yaw);
    return {
        x: world.E,
        y: 0,
        z: world.N
    };
}

export function yawNedToEnu(yawNed) {
    return Math.PI / 2 - yawNed;
}

export function yawEnuToNed(yawEnu) {
    return Math.PI / 2 - yawEnu;
}

export function frdToFlu(vector) {
    return [
        vector[0] || 0,
        -(vector[1] || 0),
        -(vector[2] || 0)
    ];
}

export function fluToFrd(vector) {
    return [
        vector[0] || 0,
        -(vector[1] || 0),
        -(vector[2] || 0)
    ];
}

export function attitudeFrdNedToFluEnu({roll = 0, pitch = 0, yaw = 0} = {}) {
    return {
        roll: -roll,
        pitch: -pitch,
        yaw: yawNedToEnu(yaw)
    };
}

export function attitudeFluEnuToFrdNed({roll = 0, pitch = 0, yaw = 0} = {}) {
    return {
        roll: -roll,
        pitch: -pitch,
        yaw: yawEnuToNed(yaw)
    };
}

export function positiveYawMomentDirection() {
    const yaw0 = 0;
    const appTurn = bodyVectorToAppWorld2D(0, 1, yaw0);
    const enuHeading0 = yawNedToEnu(yaw0);
    const enuHeadingAfterPositiveNedYaw = yawNedToEnu(0.1);
    return {
        appLateralSign: Math.sign(appTurn.x),
        enuYawDeltaSign: Math.sign(enuHeadingAfterPositiveNedYaw - enuHeading0),
        nedYawDeltaSign: 1
    };
}
