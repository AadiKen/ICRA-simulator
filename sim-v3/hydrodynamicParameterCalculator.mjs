import {calculateRealisticBoatHydrodynamics, vec3} from "./schema.js";

function parseArgs(argv) {
    const args = {
        dimensions: new vec3(2.4, 1.0, 4.8),
        mass: 120,
        options: {}
    };

    argv.forEach((arg) => {
        const [key, value] = arg.replace(/^--/, "").split("=");
        const numeric = Number(value);

        if (key === "x" || key === "beam") args.dimensions.x = numeric;
        if (key === "y" || key === "height") args.dimensions.y = numeric;
        if (key === "z" || key === "length") args.dimensions.z = numeric;
        if (key === "mass") args.mass = numeric;
        if (key === "waterDensity") args.options.waterDensity = numeric;
        if (key === "blockCoefficient") args.options.blockCoefficient = numeric;
        if (key === "transDrag") args.options.transDrag = numeric;
        if (key === "angularDrag") args.options.angularDrag = numeric;
    });

    return args;
}

function vecToObject(v) {
    return {x: v.x, y: v.y, z: v.z};
}

function serializeHydrodynamics(params) {
    return {
        dimensions: vecToObject(params.dimensions),
        mass: params.mass,
        waterDensity: params.waterDensity,
        gravity: params.gravity,
        blockCoefficient: params.blockCoefficient,
        displacementVolume: params.displacementVolume,
        draft: params.draft,
        frontalArea: params.frontalArea,
        lateralArea: params.lateralArea,
        waterplaneArea: params.waterplaneArea,
        linearDamping: vecToObject(params.linearDamping),
        quadraticDamping: vecToObject(params.quadraticDamping),
        maxSubmergenceRatio: params.maxSubmergenceRatio,
        maxDragAcceleration: vecToObject(params.maxDragAcceleration),
        maxHeaveSpeed: params.maxHeaveSpeed,
        angularDamping: vecToObject(params.angularDamping),
        inertia: vecToObject(params.inertia)
    };
}

const {dimensions, mass, options} = parseArgs(process.argv.slice(2));
const params = calculateRealisticBoatHydrodynamics(dimensions, mass, options);
console.log(JSON.stringify(serializeHydrodynamics(params), null, 2));
