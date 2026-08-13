import {invert3, invertMatrix, isPositiveDefinite, isSymmetric} from "./math.js";
import {prepareHullPrimitives} from "./forces/submergedGeometry.js";
import {totalMassMatrix6} from "./sixDof.js";
import {planarMassMatrix3} from "../packages/core/src/mass.ts";

export class VehicleParameters {
    constructor({
        id,
        vehicleClass = "surface_3dof",
        geometry,
        massProps,
        addedMass,
        damping,
        restoring,
        actuator,
        effectors = null,
        controlledDOF = null,
        allocator = null,
        controller = null,
        hullPrimitives = [],
        buoyancy = null,
        hydrodynamics = null,
        maneuveringModel = null,
        validation = {}
    }) {
        this.id = id;
        this.vehicleClass = vehicleClass;
        this.geometry = geometry;
        const mass = massProps?.mass;
        if (!Number.isFinite(mass) || mass <= 0) {
            throw new Error(`Vehicle ${id}: mass must be finite and positive.`);
        }
        const length = geometry?.length || 1;
        const beam = geometry?.beam || 1;
        const height = geometry?.height || geometry?.draft || 1;
        this.massProps = {
            ...massProps,
            cg: {...(massProps.cg || {})},
            inertia: {
                ...massProps.inertia,
                Ix: massProps.inertia?.Ix ?? massProps.inertia?.x ?? mass * (beam ** 2 + height ** 2) / 12,
                Iy: massProps.inertia?.Iy ?? massProps.inertia?.y ?? mass * (length ** 2 + height ** 2) / 12,
                Iz: massProps.inertia?.Iz ?? massProps.inertia?.z ?? mass * (length ** 2 + beam ** 2) / 12
            }
        };
        this.addedMass = addedMass || {};
        this.damping = damping;
        this.restoring = restoring;
        this.actuator = actuator;
        this.effectors = effectors || actuator?.effectors || null;
        this.controlledDOF = controlledDOF || ["surge", "yaw"];
        this.allocator = allocator || null;
        this.controller = controller || null;
        this.buoyancy = buoyancy || {
            rho: restoring?.waterDensity || 1025,
            g: restoring?.gravity || 9.81,
            sampleCount: 512,
            sampleSeed: 1
        };
        this.hullPrimitives = prepareHullPrimitives(hullPrimitives, this.buoyancy);
        this.hydrodynamics = hydrodynamics;
        this.validation = validation;
        this.maneuveringModel = mergedManeuveringModel(maneuveringModel);
        this.massMatrix = this.buildMassMatrix();
        this.massMatrixInv = invert3(this.massMatrix);
        this.massMatrix6 = totalMassMatrix6(this);
        if (!isSymmetric(this.massMatrix6, 1e-9)) {
            throw new Error(`Vehicle ${this.id}: total 6-DoF mass matrix must be symmetric.`);
        }
        if (!isPositiveDefinite(this.massMatrix6)) {
            throw new Error(`Vehicle ${this.id}: total 6-DoF mass matrix must be positive definite.`);
        }
        this.massMatrixInv6 = invertMatrix(this.massMatrix6);
    }

    static fromCoefficientSet(coeffs, overrides = {}) {
        const merged = mergeDeep(coeffs, overrides);
        const damping = normalizeDampingFromGeometry(merged);

        return new VehicleParameters({
            id: merged.id,
            vehicleClass: merged.vehicleClass || "surface_3dof",
            geometry: merged.geometry,
            massProps: merged.massProps,
            addedMass: merged.addedMass,
            damping,
            restoring: merged.restoring,
            actuator: merged.actuator,
            effectors: merged.effectors || null,
            controlledDOF: merged.controlledDOF || null,
            allocator: merged.allocator || null,
            controller: merged.controller || null,
            hullPrimitives: merged.hullPrimitives || [],
            buoyancy: merged.buoyancy || null,
            maneuveringModel: merged.maneuveringModel || null,
            validation: merged.validation || {}
        });
    }

    buildMassMatrix() {
        return planarMassMatrix3(this);
    }

    static fromGeometry(length, beam, draft, mass, options = {}) {
        const rho = options.waterDensity || 1025;
        const gravity = options.gravity || 9.81;
        if (!Number.isFinite(mass) || mass <= 0) throw new Error("Vehicle mass must be finite and positive.");
        const height = options.height || draft * 3;
        const ix = options.Ix || mass * (beam * beam + height * height) / 12;
        const iy = options.Iy || mass * (length * length + height * height) / 12;
        const iz = options.Iz || mass * (length * length + beam * beam) / 12;
        const frontalArea = beam * draft;
        const lateralArea = length * draft;
        const yawArea = lateralArea * length * length / 12;

        return new VehicleParameters({
            id: options.id || "geometry_bootstrap",
            vehicleClass: "surface_3dof",
            geometry: {length, beam, draft, height, waterplaneArea: length * beam},
            massProps: {
                mass,
                cg: {x: options.xg || 0, y: options.yg || 0, z: options.zg || 0},
                inertia: {Ix: ix, Iy: iy, Iz: iz}
            },
            addedMass: {
                XuDot: -0.05 * mass,
                YvDot: -0.75 * mass,
                ZwDot: -(options.heaveAddedMassRatio ?? 0.9) * mass,
                KpDot: -(options.rollAddedInertiaRatio ?? 0.1) * ix,
                MqDot: -(options.pitchAddedInertiaRatio ?? 0.25) * iy,
                NrDot: -0.08 * iz
            },
            damping: {
                linear: {
                    Xu: options.Xu || 6,
                    Yv: options.Yv || 18,
                    Nr: options.Nr || 0.9 * iz / Math.max(length, 0.001)
                },
                quadratic: {
                    Xuu: options.Xuu || 0.5 * rho * 0.22 * frontalArea,
                    Yvv: options.Yvv || 0.5 * rho * 1.15 * lateralArea,
                    Nrr: options.Nrr || 0.5 * rho * 0.45 * yawArea
                },
                linear6: [
                    options.Xu || 6,
                    options.Yv || 18,
                    options.Zw || 2 * Math.sqrt(Math.max(mass * rho * gravity * length * beam, 0)),
                    options.Kp || 2 * Math.sqrt(Math.max(ix * rho * gravity * mass / rho * beam * 0.12, 0)),
                    options.Mq || 2 * Math.sqrt(Math.max(iy * rho * gravity * mass / rho * length * 0.12, 0)),
                    options.Nr || 0.9 * iz / Math.max(length, 0.001)
                ],
                quadratic6: [
                    options.Xuu || 0.5 * rho * 0.22 * frontalArea,
                    options.Yvv || 0.5 * rho * 1.15 * lateralArea,
                    options.Zww || 0.5 * rho * 1.1 * length * beam,
                    options.Kpp || 0,
                    options.Mqq || 0,
                    options.Nrr || 0.5 * rho * 0.45 * yawArea
                ]
            },
            restoring: {
                waterDensity: rho,
                gravity,
                waterplaneArea: length * beam,
                displacementVolume: mass / rho,
                metacentricHeightRoll: options.metacentricHeightRoll ?? Math.max(beam * 0.12, 0.05),
                metacentricHeightPitch: options.metacentricHeightPitch ?? Math.max(length * 0.12, 0.05),
                cob: {x: options.xb || 0, y: options.yb || 0, z: options.zb ?? -draft * 0.5}
            },
            actuator: {
                beam,
                maxThrust: options.maxThrust || Math.max(mass * (options.maxAcceleration || 0.8) * 0.65, 1),
                thrustCoefficient: options.thrustCoefficient || 1,
                motorTimeConstant: options.motorTimeConstant || 0.35
            },
            effectors: options.effectors || null,
            allocator: options.allocator || null,
            controller: options.controller || null,
            controlledDOF: options.controlledDOF || null,
            hullPrimitives: options.hullPrimitives || [
                {
                    type: "box",
                    dims: {length, beam, height},
                    offset: {pos: [0, 0, 0], rot: [0, 0, 0]},
                    sampleCount: options.sampleCount || 512,
                    sampleSeed: options.sampleSeed || 1
                }
            ],
            buoyancy: {
                rho,
                g: gravity,
                sampleCount: options.sampleCount || 512,
                sampleSeed: options.sampleSeed || 1,
                horizontalWaveScale: options.horizontalWaveScale ?? 0.04
            },
            validation: {validated: false, source: "geometry bootstrap"}
        });
    }
}

function mergedManeuveringModel(value) {
    return value ? clone(value) : null;
}

function mergeDeep(base, override) {
    if (!override || typeof override !== "object") {
        return clone(base);
    }
    const result = clone(base);
    Object.entries(override).forEach(([key, value]) => {
        if (value && typeof value === "object" && !Array.isArray(value)) {
            result[key] = mergeDeep(result[key] || {}, value);
        }
        else {
            result[key] = clone(value);
        }
    });
    return result;
}

function clone(value) {
    if (value === undefined || value === null || typeof value !== "object") {
        return value;
    }
    return JSON.parse(JSON.stringify(value));
}

export function deriveDampingFromGeometry(coeffs) {
    const damping = clone(coeffs.damping || {});
    const drag = damping.drag;
    const geometry = coeffs.geometry || {};
    const rho = coeffs.buoyancy?.rho || coeffs.restoring?.waterDensity || 1025;
    if (!drag) {
        return damping;
    }
    const projected = projectedAreasFromPrimitives(coeffs.hullPrimitives || [], geometry);
    const length = projected.length || geometry.length || 0;
    const beam = projected.beam || geometry.beam || 0;
    const draft = projected.draft || geometry.draft || 0;
    const frontalArea = beam * draft;
    const lateralArea = length * draft;
    const yawArea = lateralArea * length * length / 12;

    damping.quadratic = {
        ...(damping.quadratic || {}),
        Xuu: damping.quadratic?.Xuu ?? 0.5 * rho * (drag.Cd_surge || 0) * frontalArea,
        Yvv: damping.quadratic?.Yvv ?? 0.5 * rho * (drag.Cd_sway || 0) * lateralArea,
        Nrr: damping.quadratic?.Nrr ?? 0.5 * rho * (drag.Cd_yaw || 0) * yawArea
    };
    return damping;
}

export function projectedAreasFromPrimitives(primitives = [], geometry = {}) {
    const boxes = primitives.filter((primitive) => primitive.type === "box");
    if (!boxes.length) {
        return {
            length: geometry.length || 0,
            beam: geometry.beam || 0,
            draft: geometry.draft || 0
        };
    }
    const extents = boxes.reduce((acc, primitive) => {
        const dims = primitive.dims || {};
        const offset = primitive.offset?.pos || [0, 0, 0];
        const halfLength = (dims.length || dims.x || 0) * 0.5;
        const halfBeam = (dims.beam || dims.y || 0) * 0.5;
        const halfHeight = (dims.height || dims.z || 0) * 0.5;
        acc.minX = Math.min(acc.minX, offset[0] - halfLength);
        acc.maxX = Math.max(acc.maxX, offset[0] + halfLength);
        acc.minY = Math.min(acc.minY, offset[1] - halfBeam);
        acc.maxY = Math.max(acc.maxY, offset[1] + halfBeam);
        acc.minZ = Math.min(acc.minZ, offset[2] - halfHeight);
        acc.maxZ = Math.max(acc.maxZ, offset[2] + halfHeight);
        return acc;
    }, {
        minX: Infinity,
        maxX: -Infinity,
        minY: Infinity,
        maxY: -Infinity,
        minZ: Infinity,
        maxZ: -Infinity
    });

    return {
        length: extents.maxX - extents.minX,
        beam: extents.maxY - extents.minY,
        draft: geometry.draft || Math.max((extents.maxZ - extents.minZ) * 0.5, 0)
    };
}

function normalizeDampingFromGeometry(coeffs) {
    const damping = deriveDampingFromGeometry(coeffs);
    if (damping.signConvention === "sname") {
        damping.linear = positiveResistanceTerms(damping.linear, ["Xu", "Yv", "Nr"]);
        damping.quadratic = positiveResistanceTerms(damping.quadratic, ["Xuu", "Yvv", "Nrr"]);
        damping.linearMatrix = positiveResistanceMatrix(damping.linearMatrix);
        damping.quadraticMatrix = positiveResistanceMatrix(damping.quadraticMatrix);
        damping.sourceSignConvention = "sname";
        damping.signConvention = "resistancePositive";
    }
    else if (!damping.signConvention) {
        damping.signConvention = "resistancePositive";
    }
    return damping;
}

function positiveResistanceTerms(terms = {}, keys = []) {
    const output = {...terms};
    keys.forEach((key) => {
        if (Number.isFinite(output[key])) {
            output[key] = Math.abs(output[key]);
        }
    });
    return output;
}

function positiveResistanceMatrix(matrix) {
    if (!matrix) {
        return matrix;
    }
    return matrix.map((row) => row.map((value) => Math.abs(value)));
}
