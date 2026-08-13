export const parityManeuvers = {
    "constant-thrust": {
        description: "Straight-line acceleration under constant equal thrust.",
        dt: 0.05,
        steps: 120,
        command: {type: "constant", surgeForce: 60, differentialForce: 0},
        env: {waterV: {x: 0, y: 0, z: 0}},
        tolerances: {N: 0.5, E: 0.2, yaw: 0.1}
    },
    "turning-circle": {
        description: "Constant surge plus constant differential thrust.",
        dt: 0.05,
        steps: 160,
        command: {type: "constant", surgeForce: 65, differentialForce: 35},
        env: {waterV: {x: 0, y: 0, z: 0}},
        tolerances: {N: 0.8, E: 0.8, yaw: 0.15}
    },
    "yaw-turn": {
        description: "Constant surge plus direct yaw moment for maneuvering parity.",
        dt: 0.05,
        steps: 180,
        command: {type: "constant", surgeForce: 55, differentialForce: 0, yawMoment: 1},
        env: {waterV: {x: 0, y: 0, z: 0}},
        tolerances: {N: 0.5, E: 0.2, yaw: 0.1}
    },
    "zig-zag": {
        description: "Open-loop alternating differential thrust.",
        dt: 0.05,
        steps: 240,
        command: {type: "zigZag", surgeForce: 60, differentialForce: 28, periodSec: 2},
        env: {waterV: {x: 0, y: 0, z: 0}},
        tolerances: {N: 1.0, E: 1.0, yaw: 0.2}
    },
    "coast-down": {
        description: "Accelerate briefly, then cut thrust and coast down.",
        dt: 0.05,
        steps: 180,
        command: {type: "coastDown", surgeForce: 60, differentialForce: 0, thrustDurationSec: 2},
        env: {waterV: {x: 0, y: 0, z: 0}},
        tolerances: {N: 0.8, E: 0.2, yaw: 0.1}
    },
    "current-drift": {
        description: "Zero thrust in steady east current.",
        dt: 0.05,
        steps: 120,
        command: {type: "constant", surgeForce: 0, differentialForce: 0},
        env: {waterV: {x: 0.3, y: 0, z: 0}},
        tolerances: {N: 0.3, E: 0.5, yaw: 0.15}
    }
};

export function getParityManeuver(name = "constant-thrust") {
    return parityManeuvers[name] || parityManeuvers["constant-thrust"];
}

export function listParityManeuverNames() {
    return Object.keys(parityManeuvers);
}
