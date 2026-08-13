export const bcodUsvCoefficients = {
    id: "bcod_usv",
    vehicleClass: "surface_6dof",
    provenance: {
        source: "BCOD geometry bootstrap",
        sourceVersion: "2026-07-24",
        identificationMethod: "geometry-derived, unvalidated",
        uncertainty: "unknown",
        validatedRange: null
    },
    geometry: {
        length: 4.8,
        beam: 2.4,
        draft: 0.18,
        height: 1.0,
        waterplaneArea: 11.52
    },
    massProps: {
        mass: 1000,
        cg: {x: 0, y: 0, z: 0},
        inertia: {Ix: 283.33, Iy: 2003.33, Iz: 2400}
    },
    addedMass: {
        XuDot: -50,
        YvDot: -750,
        ZwDot: -900,
        KpDot: -28.333,
        MqDot: -500.833,
        NrDot: -192
    },
    damping: {
        signConvention: "sname",
        linear: {Xu: -80, Yv: -450, Nr: -5520},
        quadratic: {Xuu: -48.708, Yvv: -509.22, Nrr: -2428.68},
        linear6: [80, 450, 7000, 1200, 2400, 5520],
        quadratic6: [48.708, 509.22, 6488, 0, 0, 2428.68],
        drag: {
            Cd_surge: 0.22,
            Cd_sway: 1.15,
            Cd_yaw: 0.45,
            Cf_linear: 1,
            validated: false
        }
    },
    restoring: {
        waterDensity: 1025,
        gravity: 9.81,
        waterplaneArea: 11.52,
        displacementVolume: 1000 / 1025,
        metacentricHeightRoll: 0.288,
        metacentricHeightPitch: 0.576,
        cob: {x: 0, y: 0, z: -0.09}
    },
    actuator: {
        beam: 2.4,
        maxThrust: 1200,
        thrustCoefficient: 1,
        motorTimeConstant: 0.45
    },
    effectors: [
        {
            id: "port",
            type: "FixedThruster",
            pos: [0, -1.2, 0],
            axis: [1, 0, 0],
            dynamics: {tau: 0.45, min: -1200, max: 1200},
            conversion: {type: "linear"}
        },
        {
            id: "starboard",
            type: "FixedThruster",
            pos: [0, 1.2, 0],
            axis: [1, 0, 0],
            dynamics: {tau: 0.45, min: -1200, max: 1200},
            conversion: {type: "linear"}
        }
    ],
    controlledDOF: ["surge", "yaw"],
    allocator: {mode: "pinv", saturation: "scale"},
    controller: {type: "usv_heading_speed", gains: {}},
    hullPrimitives: [
        {
            type: "box",
            dims: {length: 4.8, beam: 2.4, height: 1.0},
            offset: {pos: [0, 0, 0], rot: [0, 0, 0]},
            sampleCount: 512,
            sampleSeed: 42
        }
    ],
    buoyancy: {
        rho: 1025,
        g: 9.81,
        sampleCount: 512,
        sampleSeed: 42,
        horizontalWaveScale: 1
    },
    validation: {
        validated: false,
        source: "BCOD shared coefficient seed"
    }
};

export const otterCoefficients = {
    id: "otter",
    vehicleClass: "surface_6dof",
    provenance: {
        source: "Thor I. Fossen MSS Otter model",
        sourceUrl: "https://github.com/cybergalactic/MSS",
        sourceCommit: "c660120aa7ea16d0022064bd759d12a934ec4f76",
        identificationMethod: "Planar mass, damping, and cross-flow terms derived from pinned MSS otter.m; remaining 6-DoF extensions unvalidated",
        uncertainty: "Planar open-loop implementation validated against pinned MSS traces; hydrostatics and remaining 6-DoF terms unvalidated",
        validatedRange: {surgeSpeed: [0, 3]}
    },
    geometry: {
        length: 2.0,
        beam: 1.08,
        draft: 0.13,
        height: 0.35,
        waterplaneArea: 2.16
    },
    massProps: {
        mass: 55,
        cg: {x: 0.2, y: 0, z: -0.2},
        inertia: {Ix: 9.504, Iy: 15.75, Iz: 15.95}
    },
    addedMass: {
        XuDot: -5.28152100957639,
        YvDot: -82.5,
        ZwDot: -55,
        KpDot: -1.584,
        MqDot: -11,
        NrDot: -23.375
    },
    damping: {
        signConvention: "sname",
        linear: {Xu: -77.5334370139969, Yv: -137.5, Nr: -39.325},
        quadratic: {Xuu: 0, Yvv: 0, Nrr: -393.25},
        linear6: [77.5334370139969, 137.5, 0, 0, 0, 39.325],
        quadratic6: [0, 0, 0, 0, 0, 393.25],
        crossFlow: {
            enabled: true,
            length: 2,
            draft: 0.13414634146341464,
            coefficient: 0.846865756,
            strips: 20,
            waterDensity: 1025
        },
        drag: {
            Cd_surge: 0.22,
            Cd_sway: 1.15,
            Cd_yaw: 0.45,
            Cf_linear: 1,
            validated: true
        }
    },
    restoring: {
        waterDensity: 1025,
        gravity: 9.81,
        waterplaneArea: 2.16,
        displacementVolume: 55 / 1025,
        metacentricHeightRoll: 0.13,
        metacentricHeightPitch: 0.24,
        cob: {x: 0, y: 0, z: -0.065}
    },
    actuator: {
        beam: 1.08,
        maxThrust: 95,
        thrustCoefficient: 1,
        motorTimeConstant: 0.25
    },
    effectors: [
        {
            id: "port",
            type: "FixedThruster",
            pos: [0, -0.54, 0],
            axis: [1, 0, 0],
            dynamics: {tau: 0.25, min: -95, max: 95},
            conversion: {type: "linear"}
        },
        {
            id: "starboard",
            type: "FixedThruster",
            pos: [0, 0.54, 0],
            axis: [1, 0, 0],
            dynamics: {tau: 0.25, min: -95, max: 95},
            conversion: {type: "linear"}
        }
    ],
    controlledDOF: ["surge", "yaw"],
    allocator: {mode: "pinv", saturation: "scale"},
    controller: {type: "usv_heading_speed", gains: {}},
    hullPrimitives: [
        {
            type: "box",
            dims: {length: 2.0, beam: 1.08, height: 0.35},
            offset: {pos: [0.2, 0, 0], rot: [0, 0, 0]},
            sampleCount: 512,
            sampleSeed: 7
        }
    ],
    buoyancy: {
        rho: 1025,
        g: 9.81,
        sampleCount: 512,
        sampleSeed: 7,
        horizontalWaveScale: 1
    },
    validation: {
        validated: true,
        source: "Otter USV reference-style 3-DOF preset"
    }
};
