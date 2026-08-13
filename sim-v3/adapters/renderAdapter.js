import {RigidBodyState} from "../core/rigidBodyState.js";

export function coreToRenderState(coreState) {
    const euler = coreState.eulerAngles;
    return {
        pos: {
            x: coreState.position.E,
            y: -coreState.position.D,
            z: coreState.position.N
        },
        heading: euler.yaw,
        orientation: {
            x: euler.pitch,
            y: euler.yaw,
            z: euler.roll
        }
    };
}

export function renderToCoreState(boatState) {
    return RigidBodyState.fromEuler(
        {
            N: boatState.pos.z,
            E: boatState.pos.x,
            D: -boatState.pos.y
        },
        boatState.orientation.z || 0,
        boatState.orientation.x || 0,
        boatState.heading || boatState.orientation.y || 0
    );
}
