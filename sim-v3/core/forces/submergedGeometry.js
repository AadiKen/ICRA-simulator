function lcg(seed) {
    let state = (seed >>> 0) || 1;
    return () => {
        state = (1664525 * state + 1013904223) >>> 0;
        return state / 0x100000000;
    };
}

function normalize(v) {
    const mag = Math.hypot(v.x || 0, v.y || 0, v.z || 0);
    if (mag === 0) {
        return {x: 0, y: 1, z: 0};
    }
    return {x: v.x / mag, y: v.y / mag, z: v.z / mag};
}

function add(a, b) {
    return {x: (a.x || 0) + (b.x || 0), y: (a.y || 0) + (b.y || 0), z: (a.z || 0) + (b.z || 0)};
}

function scale(v, s) {
    return {x: (v.x || 0) * s, y: (v.y || 0) * s, z: (v.z || 0) * s};
}

export function primitiveSolidVolume(spec) {
    const dims = spec.dims || {};
    const length = Math.max(dims.length || dims.x || 0, 0);
    const beam = Math.max(dims.beam || dims.y || dims.radius * 2 || 0, 0);
    const height = Math.max(dims.height || dims.z || dims.radius * 2 || 0, 0);
    const radius = Math.max(dims.radius || beam * 0.5 || height * 0.5 || 0, 0);
    if (spec.type === "box") {
        return length * beam * height;
    }
    if (spec.type === "cylinder") {
        return Math.PI * radius * radius * length;
    }
    if (spec.type === "capsule") {
        const cylinderLength = Math.max(length - 2 * radius, 0);
        return Math.PI * radius * radius * cylinderLength + (4 / 3) * Math.PI * radius ** 3;
    }
    if (spec.type === "ellipsoid") {
        return (4 / 3) * Math.PI * (length * 0.5) * (beam * 0.5) * (height * 0.5);
    }
    if (spec.type === "cone") {
        return (Math.PI * radius * radius * length) / 3;
    }
    throw new Error(`Unsupported hull primitive type: ${spec.type}`);
}

export function generateSamples(primitive, sampleCount = 512, seed = 1) {
    const dims = primitive.dims || {};
    const length = dims.length || dims.x || 0;
    const radius = dims.radius || 0;
    const beam = dims.beam || dims.y || radius * 2 || 0;
    const height = dims.height || dims.z || radius * 2 || 0;
    const offset = primitive.offset || {};
    const n = Math.max(Math.floor(sampleCount), 1);
    const side = Math.ceil(Math.cbrt(n));
    const samples = [];
    const random = lcg(seed);

    if (primitive.type === "box") {
        for (let ix = 0; ix < side && samples.length < n; ix += 1) {
            for (let iy = 0; iy < side && samples.length < n; iy += 1) {
                for (let iz = 0; iz < side && samples.length < n; iz += 1) {
                    const mirrorX = ix >= side / 2;
                    const mirrorY = iy >= side / 2;
                    const mirrorZ = iz >= side / 2;
                    const baseX = Math.min(ix, side - 1 - ix);
                    const baseY = Math.min(iy, side - 1 - iy);
                    const baseZ = Math.min(iz, side - 1 - iz);
                    const jitterSeed = (baseX * 73856093) ^ (baseY * 19349663) ^ (baseZ * 83492791) ^ seed;
                    const jitter = lcg(jitterSeed);
                    const jx = (jitter() - 0.5) * 0.25;
                    const jy = (jitter() - 0.5) * 0.25;
                    const jz = (jitter() - 0.5) * 0.25;
                    const fx = ((ix + 0.5 + (mirrorX ? -jx : jx)) / side) - 0.5;
                    const fy = ((iy + 0.5 + (mirrorY ? -jy : jy)) / side) - 0.5;
                    const fz = ((iz + 0.5 + (mirrorZ ? -jz : jz)) / side) - 0.5;
                    samples.push(applyPrimitiveOffset([fx * length, fy * beam, fz * height], offset));
                }
            }
        }
        return samples;
    }

    const maxAttempts = n * 200;
    for (let attempts = 0; samples.length < n && attempts < maxAttempts; attempts += 1) {
        const candidate = [
            (random() - 0.5) * length,
            (random() - 0.5) * beam,
            (random() - 0.5) * height
        ];
        if (pointInsidePrimitive(candidate, primitive)) {
            samples.push(applyPrimitiveOffset(candidate, offset));
        }
    }
    if (samples.length !== n) {
        throw new Error(`Could not generate ${n} samples for ${primitive.type}`);
    }
    return samples;
}

function pointInsidePrimitive(point, primitive) {
    const dims = primitive.dims || {};
    const length = dims.length || dims.x || 0;
    const radius = dims.radius || Math.min(dims.beam || dims.y || 0, dims.height || dims.z || 0) * 0.5;
    const beam = dims.beam || dims.y || radius * 2 || 0;
    const height = dims.height || dims.z || radius * 2 || 0;
    const [x, y, z] = point;
    if (primitive.type === "cylinder") {
        return Math.abs(x) <= length * 0.5 && y * y + z * z <= radius * radius;
    }
    if (primitive.type === "capsule") {
        const halfCylinder = Math.max(length * 0.5 - radius, 0);
        const axial = Math.max(Math.abs(x) - halfCylinder, 0);
        return axial * axial + y * y + z * z <= radius * radius;
    }
    if (primitive.type === "ellipsoid") {
        const ax = Math.max(length * 0.5, 1e-9);
        const ay = Math.max(beam * 0.5, 1e-9);
        const az = Math.max(height * 0.5, 1e-9);
        return (x * x) / (ax * ax) + (y * y) / (ay * ay) + (z * z) / (az * az) <= 1;
    }
    if (primitive.type === "cone") {
        const half = Math.max(length * 0.5, 1e-9);
        if (x < -half || x > half) {
            return false;
        }
        const localRadius = radius * (half - x) / Math.max(length, 1e-9);
        return y * y + z * z <= localRadius * localRadius;
    }
    throw new Error(`Unsupported hull primitive type: ${primitive.type}`);
}

function applyPrimitiveOffset(point, offset = {}) {
    const rotated = rotateBodyPoint(point, offset.rot || [0, 0, 0]);
    const pos = offset.pos || [0, 0, 0];
    return [rotated[0] + pos[0], rotated[1] + pos[1], rotated[2] + pos[2]];
}

function rotateBodyPoint(point, rot = [0, 0, 0]) {
    const [roll, pitch, yaw] = rot;
    const [x, y, z] = point;
    const cr = Math.cos(roll || 0), sr = Math.sin(roll || 0);
    const cp = Math.cos(pitch || 0), sp = Math.sin(pitch || 0);
    const cy = Math.cos(yaw || 0), sy = Math.sin(yaw || 0);
    const y1 = y * cr - z * sr;
    const z1 = y * sr + z * cr;
    const x2 = x * cp + z1 * sp;
    const z2 = -x * sp + z1 * cp;
    return [
        x2 * cy - y1 * sy,
        x2 * sy + y1 * cy,
        z2
    ];
}

export function prepareHullPrimitive(spec, defaults = {}) {
    const sampleCount = spec.sampleCount || defaults.sampleCount || 512;
    const sampleSeed = spec.sampleSeed || defaults.sampleSeed || 1;
    return {
        ...spec,
        solidVolume: spec.solidVolume || primitiveSolidVolume(spec),
        samples: spec.samples || generateSamples(spec, sampleCount, sampleSeed),
        sampleCount,
        sampleSeed
    };
}

export function prepareHullPrimitives(primitives = [], defaults = {}) {
    return primitives.map((primitive) => prepareHullPrimitive(primitive, defaults));
}

export function bodySampleToAppLocal(sample) {
    return {
        x: sample[1],
        y: -sample[2],
        z: sample[0]
    };
}

export function rotateAppLocal(offset, orientation = {x: 0, y: 0, z: 0}) {
    const pitch = orientation.x || 0;
    const yaw = orientation.y || 0;
    const roll = orientation.z || 0;
    const cosR = Math.cos(roll);
    const sinR = Math.sin(roll);
    const rollX = offset.x * cosR - offset.y * sinR;
    const rollY = offset.x * sinR + offset.y * cosR;
    const cosP = Math.cos(pitch);
    const sinP = Math.sin(pitch);
    const pitchY = rollY * cosP - offset.z * sinP;
    const pitchZ = rollY * sinP + offset.z * cosP;
    const cosY = Math.cos(yaw);
    const sinY = Math.sin(yaw);
    return {
        x: rollX * cosY + pitchZ * sinY,
        y: pitchY,
        z: -rollX * sinY + pitchZ * cosY
    };
}

export function appWorldVectorToBody3(vector, yaw = 0) {
    const cos = Math.cos(yaw);
    const sin = Math.sin(yaw);
    const stbd = (vector.x || 0) * cos - (vector.z || 0) * sin;
    const fwd = (vector.x || 0) * sin + (vector.z || 0) * cos;
    return [fwd, stbd, -(vector.y || 0)];
}

export function cross3(a, b) {
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    ];
}

export function computeSubmergedState(primitives, bodyPose, waterField, time) {
    const prepared = prepareHullPrimitives(primitives || []);
    const position = bodyPose.position || {x: 0, y: 0, z: 0};
    const orientation = bodyPose.orientation || {x: 0, y: 0, z: 0};
    const perPrimitive = prepared.map((primitive) => {
        const submergedSamples = [];
        let normalSum = {x: 0, y: 0, z: 0};

        primitive.samples.forEach((sample) => {
            const local = bodySampleToAppLocal(sample);
            const rotated = rotateAppLocal(local, orientation);
            const world = add(position, rotated);
            const waterSample = waterField.sampleAt(world, time);
            const submerged = waterSample.submerged || waterSample.surfaceHeight >= world.y;
            if (!submerged) {
                return;
            }
            submergedSamples.push(sample);
            normalSum = add(normalSum, waterSample.normal || {x: 0, y: 1, z: 0});
        });

        const fraction = primitive.samples.length
            ? submergedSamples.length / primitive.samples.length
            : 0;
        const centroid = submergedSamples.reduce((sum, sample) => {
            return [sum[0] + sample[0], sum[1] + sample[1], sum[2] + sample[2]];
        }, [0, 0, 0]).map((value) => submergedSamples.length ? value / submergedSamples.length : 0);
        const volume = primitive.solidVolume * fraction;

        return {
            primitive,
            submergedFraction: fraction,
            volume,
            centroidBody: centroid,
            surfaceNormalWorld: normalize(normalSum)
        };
    });

    const totalVolume = perPrimitive.reduce((sum, entry) => sum + entry.volume, 0);
    const cobBody = perPrimitive.reduce((sum, entry) => {
        return [
            sum[0] + entry.centroidBody[0] * entry.volume,
            sum[1] + entry.centroidBody[1] * entry.volume,
            sum[2] + entry.centroidBody[2] * entry.volume
        ];
    }, [0, 0, 0]).map((value) => totalVolume > 0 ? value / totalVolume : 0);
    const solidVolume = perPrimitive.reduce((sum, entry) => sum + entry.primitive.solidVolume, 0);

    return {
        perPrimitive,
        totalVolume,
        cobBody,
        submergedFractionTotal: solidVolume > 0 ? totalVolume / solidVolume : 0,
        sampleCount: perPrimitive.reduce((sum, entry) => sum + entry.primitive.samples.length, 0)
    };
}

export function scaleAppVector(v, s) {
    return scale(v, s);
}
