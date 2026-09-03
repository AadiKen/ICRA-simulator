import fs from "node:fs";
import path from "node:path";
import {fileURLToPath} from "node:url";
import {bcodUsvCoefficients, otterCoefficients} from "../core/vehicles/coefficients.js";
import {getParityManeuver, listParityManeuverNames} from "./parityManeuvers.js";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const repoRoot = path.resolve(__dirname, "..");

const vehicles = {
    bcod_usv: bcodUsvCoefficients,
    otter: otterCoefficients
};

function parseArgs(argv) {
    const args = {
        vehicle: "otter",
        maneuver: "constant-thrust",
        outDir: path.join(repoRoot, "gazebo", "generated"),
        all: false,
        includeAddedMass: false
    };
    for (let i = 2; i < argv.length; i += 1) {
        if (argv[i] === "--vehicle") {
            args.vehicle = argv[i + 1] || args.vehicle;
            i += 1;
        }
        else if (argv[i] === "--maneuver") {
            args.maneuver = argv[i + 1] || args.maneuver;
            i += 1;
        }
        else if (argv[i] === "--out") {
            args.outDir = path.resolve(argv[i + 1] || args.outDir);
            i += 1;
        }
        else if (argv[i] === "--all") {
            args.all = true;
        }
        else if (argv[i] === "--include-added-mass") {
            args.includeAddedMass = true;
        }
    }
    return args;
}

function xmlEscape(value) {
    return String(value).replace(/[<>&"']/g, (char) => ({
        "<": "&lt;",
        ">": "&gt;",
        "&": "&amp;",
        "\"": "&quot;",
        "'": "&apos;"
    }[char]));
}

function format(n) {
    return Number.isFinite(n) ? Number(n.toFixed(10)).toString() : "0";
}

function inertiaFromCoeffs(coeffs) {
    const mass = coeffs.massProps.mass;
    const length = coeffs.geometry.length;
    const beam = coeffs.geometry.beam;
    const height = coeffs.geometry.height || coeffs.geometry.draft * 3;
    const geometryBootstrap = coeffs.provenance?.identificationMethod === "geometry-derived, unvalidated";
    // Vehicle A's bootstrap craft is defined as a uniform rectangular hull.
    // Its checked-in diagonal is an obsolete, non-realizable value, so derive
    // the rigid-body tensor directly from that stated mass and geometry.
    if (geometryBootstrap) {
        return {
            ixx: mass * (beam * beam + height * height) / 12,
            iyy: mass * (length * length + height * height) / 12,
            izz: mass * (length * length + beam * beam) / 12,
            ixy: 0, ixz: 0, iyz: 0
        };
    }
    const inertia = {
        ixx: coeffs.massProps.inertia?.Ix || mass * (beam * beam + height * height) / 12,
        iyy: coeffs.massProps.inertia?.Iy || mass * (length * length + height * height) / 12,
        izz: coeffs.massProps.inertia?.Iz || coeffs.massProps.inertia?.z || mass * (length * length + beam * beam) / 12,
        ixy: 0,
        ixz: 0,
        iyz: 0
    };
    return inertia;
}

function skew(v) {
    const [x, y, z] = v;
    return [
        [0, -z, y],
        [z, 0, -x],
        [-y, x, 0]
    ];
}

function matMul(a, b) {
    return a.map((row) => b[0].map((_, colIdx) => row.reduce((sum, value, rowIdx) => sum + value * b[rowIdx][colIdx], 0)));
}

function matTranspose(matrix) {
    return matrix[0].map((_, colIdx) => matrix.map((row) => row[colIdx]));
}

function choleskySpd(matrix, tolerance = 1e-9) {
    const n = matrix.length;
    const l = Array.from({length: n}, () => Array(n).fill(0));
    for (let i = 0; i < n; i += 1) {
        for (let j = 0; j <= i; j += 1) {
            let sum = matrix[i][j];
            for (let k = 0; k < j; k += 1) {
                sum -= l[i][k] * l[j][k];
            }
            if (i === j) {
                if (sum <= tolerance || !Number.isFinite(sum)) {
                    return false;
                }
                l[i][j] = Math.sqrt(sum);
            }
            else {
                l[i][j] = sum / l[j][j];
            }
        }
    }
    return true;
}

function symmetricAddedMassMatrix6(coeffs) {
    const added = coeffs.addedMass || {};
    const yrValues = [added.YrDot, added.NvDot]
        .filter((value) => Number.isFinite(value))
        .map((value) => -value);
    const yr = yrValues.length
        ? yrValues.reduce((sum, value) => sum + value, 0) / yrValues.length
        : 0;
    const matrix = Array.from({length: 6}, () => Array(6).fill(0));
    matrix[0][0] = -(added.XuDot || 0);
    matrix[1][1] = -(added.YvDot || 0);
    matrix[1][5] = yr;
    matrix[5][1] = yr;
    matrix[5][5] = -(added.NrDot || 0);
    return matrix;
}

function addedMassMatrix3(coeffs) {
    const matrix = symmetricAddedMassMatrix6(coeffs);
    return [
        [matrix[0][0], matrix[0][1], matrix[0][5]],
        [matrix[1][0], matrix[1][1], matrix[1][5]],
        [matrix[5][0], matrix[5][1], matrix[5][5]]
    ];
}

function blockDiagonalTransform(rotation3) {
    const transform = Array.from({length: 6}, () => Array(6).fill(0));
    for (let i = 0; i < 3; i += 1) {
        for (let j = 0; j < 3; j += 1) {
            transform[i][j] = rotation3[i][j];
            transform[i + 3][j + 3] = rotation3[i][j];
        }
    }
    return transform;
}

function translateSpatialMassMatrix(matrix, offset) {
    const h = Array.from({length: 6}, (_, i) => Array.from({length: 6}, (_, j) => i === j ? 1 : 0));
    const s = skew(offset);
    for (let i = 0; i < 3; i += 1) {
        for (let j = 0; j < 3; j += 1) {
            h[i][j + 3] = s[i][j];
        }
    }
    return matMul(matMul(matTranspose(h), matrix), h);
}

function gazeboFluidAddedMassMatrix6(coeffs) {
    const sourceBody = symmetricAddedMassMatrix6(coeffs);
    if (sourceBody[2][2] === 0 && sourceBody[1][1] > 0) {
        sourceBody[2][2] = sourceBody[1][1];
    }
    const bodyToGazebo = blockDiagonalTransform([
        [1, 0, 0],
        [0, -1, 0],
        [0, 0, -1]
    ]);
    const sourceGazebo = matMul(matMul(matTranspose(bodyToGazebo), sourceBody), bodyToGazebo);
    const cg = coeffs.massProps.cg || {};
    const rg = [cg.x || 0, -(cg.y || 0), -(cg.z || 0)];
    return translateSpatialMassMatrix(sourceGazebo, rg);
}

function rigidBodySpatialInertia(coeffs, inertia = inertiaFromCoeffs(coeffs)) {
    const m = coeffs.massProps.mass;
    const cg = coeffs.massProps.cg || {};
    const rg = [cg.x || 0, cg.y || 0, cg.z || 0];
    const s = skew(rg);
    const ss = matMul(s, s);
    const matrix = Array.from({length: 6}, () => Array(6).fill(0));
    for (let i = 0; i < 3; i += 1) {
        matrix[i][i] = m;
        for (let j = 0; j < 3; j += 1) {
            matrix[i][j + 3] = -m * s[i][j];
            matrix[i + 3][j] = m * s[i][j];
        }
    }
    const iG = [
        [inertia.ixx, inertia.ixy, inertia.ixz],
        [inertia.ixy, inertia.iyy, inertia.iyz],
        [inertia.ixz, inertia.iyz, inertia.izz]
    ];
    for (let i = 0; i < 3; i += 1) {
        for (let j = 0; j < 3; j += 1) {
            matrix[i + 3][j + 3] = iG[i][j] - m * ss[i][j];
        }
    }
    return matrix;
}

function assertAddedMassIsValid(coeffs, inertia = inertiaFromCoeffs(coeffs)) {
    const added = symmetricAddedMassMatrix6(coeffs);
    return assertSpatialAddedMassIsValid(coeffs, added, inertia);
}

function assertGazeboFluidAddedMassIsValid(coeffs, inertia = inertiaFromCoeffs(coeffs)) {
    const added = gazeboFluidAddedMassMatrix6(coeffs);
    return assertSpatialAddedMassIsValid(coeffs, added, inertia);
}

function assertSpatialAddedMassIsValid(coeffs, added, inertia = inertiaFromCoeffs(coeffs)) {
    const rigid = rigidBodySpatialInertia(coeffs, inertia);
    for (let i = 0; i < 6; i += 1) {
        for (let j = 0; j < 6; j += 1) {
            if (Math.abs(added[i][j] - added[j][i]) > 1e-9) {
                throw new Error(`Generated fluid_added_mass is not symmetric at [${i},${j}].`);
            }
        }
    }
    const total = rigid.map((row, i) => row.map((value, j) => value + added[i][j]));
    if (!choleskySpd(total)) {
        throw new Error("Generated M_RB + M_A is not strictly positive definite.");
    }
    return {added, rigid, total};
}

function renderFluidAddedMass(coeffs, options = {}) {
    if (!options.includeAddedMass) {
        return `
        <!-- fluid_added_mass intentionally omitted for Gazebo capture: this oracle validates integration, frames, thrust mapping, and drag, but not added mass. On this Gazebo/DART build, both the direct MSS matrix and the spatially completed CG-shifted matrix reset poses under positive surge. -->`;
    }
    const matrix = gazeboFluidAddedMassMatrix6(coeffs);
    const names = [
        ["xx", 0, 0], ["xy", 0, 1], ["xz", 0, 2], ["xp", 0, 3], ["xq", 0, 4], ["xr", 0, 5],
        ["yy", 1, 1], ["yz", 1, 2], ["yp", 1, 3], ["yq", 1, 4], ["yr", 1, 5],
        ["zz", 2, 2], ["zp", 2, 3], ["zq", 2, 4], ["zr", 2, 5],
        ["pp", 3, 3], ["pq", 3, 4], ["pr", 3, 5],
        ["qq", 4, 4], ["qr", 4, 5],
        ["rr", 5, 5]
    ];
    const entries = names
        .map(([name, row, col]) => [name, matrix[row][col]])
        .filter(([, value]) => Math.abs(value) > 1e-12)
        .map(([name, value]) => `          <${name}>${format(value)}</${name}>`)
        .join("\n");
    return `
        <fluid_added_mass>
${entries}
        </fluid_added_mass>`;
}

function primitivePose(primitive) {
    const pos = primitive.offset?.pos || [0, 0, 0];
    const rot = primitive.offset?.rot || [0, 0, 0];
    return `${format(pos[0])} ${format(-pos[1])} ${format(-pos[2])} ${format(-rot[0])} ${format(-rot[1])} ${format(-rot[2])}`;
}

function primitiveGeometry(primitive, fallbackGeometry = {}) {
    const dims = primitive.dims || {};
    if (primitive.type === "box") {
        return `<box><size>${format(dims.length || dims.x || fallbackGeometry.length || 0)} ${format(dims.beam || dims.y || fallbackGeometry.beam || 0)} ${format(dims.height || dims.z || fallbackGeometry.height || fallbackGeometry.draft * 3 || 0)}</size></box>`;
    }
    if (primitive.type === "cylinder" || primitive.type === "capsule") {
        return `<cylinder><radius>${format(dims.radius || 0)}</radius><length>${format(dims.length || dims.x || 0)}</length></cylinder>`;
    }
    if (primitive.type === "ellipsoid") {
        return `<ellipsoid><radii>${format((dims.length || dims.x || 0) * 0.5)} ${format((dims.beam || dims.y || 0) * 0.5)} ${format((dims.height || dims.z || 0) * 0.5)}</radii></ellipsoid>`;
    }
    if (primitive.type === "cone") {
        return `<cone><radius>${format(dims.radius || 0)}</radius><length>${format(dims.length || dims.x || 0)}</length></cone>`;
    }
    throw new Error(`Unsupported primitive for SDF generation: ${primitive.type}`);
}

function buoyancyCollisionPrimitive(primitive, coeffs, idx, count) {
    if (primitive.type !== "box" || count !== 1) {
        return primitive;
    }
    const dims = primitive.dims || {};
    const length = dims.length || dims.x || coeffs.geometry?.length || 0;
    const beam = dims.beam || dims.y || coeffs.geometry?.beam || 0;
    const rho = coeffs.buoyancy?.rho || coeffs.restoring?.waterDensity || 1025;
    const displacementVolume = coeffs.restoring?.displacementVolume || coeffs.massProps?.mass / rho;
    const neutralHeight = length > 0 && beam > 0
        ? 2 * displacementVolume / (length * beam)
        : dims.height || dims.z || coeffs.geometry?.draft || 0;
    return {
        ...primitive,
        dims: {
            ...dims,
            height: Math.min(neutralHeight, dims.height || dims.z || neutralHeight),
            z: Math.min(neutralHeight, dims.height || dims.z || neutralHeight)
        }
    };
}

function renderHullPrimitives(coeffs) {
    const primitives = coeffs.hullPrimitives?.length
        ? coeffs.hullPrimitives
        : [{type: "box", dims: coeffs.geometry, offset: {pos: [0, 0, 0], rot: [0, 0, 0]}}];
    return primitives.map((primitive, idx) => `
      <collision name="hull_collision_${idx}">
        <pose>${primitivePose(primitive)}</pose>
        <geometry>${primitiveGeometry(buoyancyCollisionPrimitive(primitive, coeffs, idx, primitives.length), coeffs.geometry)}</geometry>
      </collision>
      <visual name="hull_visual_${idx}">
        <pose>${primitivePose(primitive)}</pose>
        <geometry>${primitiveGeometry(primitive, coeffs.geometry)}</geometry>
      </visual>`).join("\n");
}

function renderEffectors(coeffs, options = {}) {
    return (coeffs.effectors || []).map((effector) => {
        if (effector.type === "ControlSurface") {
            return `
      <plugin filename="gz-sim-lift-drag-system" name="gz::sim::systems::LiftDrag">
        <air_density>${format(effector.foil?.rho || 1025)}</air_density>
        <cla>${format(effector.foil?.C_Lalpha || effector.foil?.CLalpha || 0)}</cla>
        <cda>${format(effector.foil?.C_D0 || effector.foil?.CD0 || 0)}</cda>
        <area>${format(effector.foil?.A || effector.foil?.area || 0)}</area>
        <link_name>${xmlEscape(effector.id)}_link</link_name>
      </plugin>`;
        }
        if (effector.type === "Rotor") {
            return `
      <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
        <jointName>${xmlEscape(effector.id)}_joint</jointName>
        <linkName>${xmlEscape(effector.id)}_link</linkName>
        <motorConstant>${format(effector.conversion?.k_T || effector.conversion?.kT || 0)}</motorConstant>
        <momentConstant>${format(effector.conversion?.k_Q || effector.conversion?.kQ || 0)}</momentConstant>
        <turningDirection>${(effector.spinDirection || 1) > 0 ? "ccw" : "cw"}</turningDirection>
      </plugin>`;
        }
        if (!options.perThrusterActuation && (effector.type === "FixedThruster" || effector.type === "AzimuthThruster")) {
            return "";
        }
        return `
      <plugin filename="gz-sim-thruster-system" name="gz::sim::systems::Thruster">
        <namespace>/</namespace>
        <joint_name>${xmlEscape(effector.id)}_joint</joint_name>
        <thrust_coefficient>1</thrust_coefficient>
        <fluid_density>${format(coeffs.buoyancy?.rho || coeffs.restoring?.waterDensity || 1025)}</fluid_density>
        <propeller_diameter>1</propeller_diameter>
        <use_angvel_cmd>false</use_angvel_cmd>
        <max_thrust_cmd>${format(effector.dynamics?.max || coeffs.actuator?.maxThrust || 0)}</max_thrust_cmd>
        <min_thrust_cmd>${format(effector.dynamics?.min || -(coeffs.actuator?.maxThrust || 0))}</min_thrust_cmd>
        <topic>${thrusterCommandTopic(coeffs.id, effector.id)}</topic>
      </plugin>`;
    }).join("\n");
}

function renderJointControllers(coeffs) {
    return "";
}

function thrusterCommandTopic(modelName, effectorId) {
    return advertisedThrusterCommandTopic(modelName, effectorId);
}

function advertisedThrusterCommandTopic(modelName, effectorId) {
    return `/model/${xmlEscape(modelName)}/joint/${xmlEscape(effectorId)}_joint/cmd_thrust`;
}

function wrenchTopicForWorld(maneuverName) {
    return `/world/bcod_parity_${xmlEscape(maneuverName)}/wrench`;
}

function persistentWrenchTopicForWorld(maneuverName) {
    return `${wrenchTopicForWorld(maneuverName)}/persistent`;
}

function renderThrusterLinks(coeffs, options = {}) {
    return (coeffs.effectors || [])
        .filter((effector) => options.perThrusterActuation || (effector.type !== "FixedThruster" && effector.type !== "AzimuthThruster"))
        .map((effector) => {
        const pos = effector.pos || [0, 0, 0];
        const linkName = effector.type === "FixedThruster" || effector.type === "AzimuthThruster"
            ? `${effector.id}_thruster`
            : `${effector.id}_link`;
        const jointType = effector.type === "FixedThruster" || effector.type === "AzimuthThruster" || effector.type === "Rotor"
            ? "revolute"
            : "fixed";
        return `
    <link name="${xmlEscape(linkName)}">
      <pose relative_to="base_link">${format(pos[0])} ${format(-pos[1])} ${format(-pos[2])} 0 0 0</pose>
      <inertial><mass>0.001</mass><inertia><ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
    </link>
    <joint name="${xmlEscape(effector.id)}_joint" type="${jointType}">
      <parent>base_link</parent>
      <child>${xmlEscape(linkName)}</child>
      ${jointType === "revolute" ? `<axis><xyz>${axisToSdf(effector.axis || [1, 0, 0])}</xyz></axis>` : ""}
    </joint>`;
    }).join("\n");
}

function axisToSdf(axis = [1, 0, 0]) {
    return `${format(axis[0] || 0)} ${format(-(axis[1] || 0))} ${format(-(axis[2] || 0))}`;
}

function renderPosePublisher() {
    return `
    <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
      <publish_model_pose>true</publish_model_pose>
      <publish_link_pose>true</publish_link_pose>
      <use_pose_vector_msg>true</use_pose_vector_msg>
      <static_publisher>false</static_publisher>
    </plugin>`;
}

function renderPhaseASensors(options = {}) {
    if (!options.phaseASensors) return "";
    return `
      <sensor name="task_imu" type="imu">
        <always_on>true</always_on><update_rate>20</update_rate>
        <topic>imu</topic>
      </sensor>
      <sensor name="task_gps" type="navsat">
        <always_on>true</always_on><update_rate>20</update_rate>
        <topic>gps</topic>
      </sensor>`;
}

function renderOdometryPublisher(options = {}) {
    if (!options.trueOdometry) return "";
    return `
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_publish_frequency>${format(options.odomHz || 20)}</odom_publish_frequency>
      <odom_topic>odometry</odom_topic>
      <dimensions>3</dimensions>
    </plugin>`;
}

function renderModelSdf(coeffs, options = {}) {
    const inertia = inertiaFromCoeffs(coeffs);
    assertAddedMassIsValid(coeffs, inertia);
    const linear = coeffs.damping.linear || {};
    const quadratic = coeffs.damping.quadratic || {};
    return `<?xml version="1.0" ?>
<sdf version="1.10">
  <model name="${xmlEscape(coeffs.id)}">
    <pose>0 0 0 0 0 0</pose>
    <link name="base_link">
      <inertial>
        <!-- Planar parity fixture: the vertical CG is stabilized below the primitive buoyancy center because Gazebo's collision buoyancy does not reproduce the Otter's metacentric hydrostatics. This fixture cannot validate coupled6 hydrostatics. -->
        <pose>${format(coeffs.massProps.cg?.x || 0)} ${format(-(coeffs.massProps.cg?.y || 0))} ${format(coeffs.massProps.cg?.z || 0)} 0 0 0</pose>
        <mass>${format(coeffs.massProps.mass)}</mass>
        <inertia>
          <ixx>${format(inertia.ixx)}</ixx><iyy>${format(inertia.iyy)}</iyy><izz>${format(inertia.izz)}</izz>
          <ixy>${format(inertia.ixy)}</ixy><ixz>${format(inertia.ixz)}</ixz><iyz>${format(inertia.iyz)}</iyz>
        </inertia>
${renderFluidAddedMass(coeffs, options)}
      </inertial>
${renderHullPrimitives(coeffs)}
${renderPhaseASensors(options)}
    </link>
${renderThrusterLinks(coeffs, options)}
    <plugin filename="gz-sim-hydrodynamics-system" name="gz::sim::systems::Hydrodynamics">
      <link_name>base_link</link_name>
      <water_density>${format(coeffs.buoyancy?.rho || coeffs.restoring?.waterDensity || 1025)}</water_density>
      <xU>${format(linear.Xu || 0)}</xU>
      <yV>${format(linear.Yv || 0)}</yV>
      <nR>${format(linear.Nr || 0)}</nR>
      <xUabsU>${format(quadratic.Xuu || 0)}</xUabsU>
      <yVabsV>${format(quadratic.Yvv || 0)}</yVabsV>
      <nRabsR>${format(quadratic.Nrr || 0)}</nRabsR>
      ${options.includeAddedMass ? "<disable_added_mass>true</disable_added_mass>" : ""}
    </plugin>
${renderPosePublisher()}
    ${options.phaseASensors ? '<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"/>' : ""}
${renderOdometryPublisher(options)}
${renderEffectors(coeffs, options)}
${renderJointControllers(coeffs)}
  </model>
</sdf>
`;
}

function renderModelConfig(coeffs) {
    return `<?xml version="1.0"?>
<model>
  <name>${xmlEscape(coeffs.id)}</name>
  <version>1.0</version>
  <sdf version="1.10">model.sdf</sdf>
  <author>
    <name>BCOD</name>
  </author>
  <description>Generated ${xmlEscape(coeffs.id)} parity model for BCOD Gazebo comparison.</description>
</model>
`;
}

function renderBuoyancyPlugin(coeffs) {
    const rho = coeffs.buoyancy?.rho || coeffs.restoring?.waterDensity || 1025;
    return `
    <plugin filename="gz-sim-buoyancy-system" name="gz::sim::systems::Buoyancy">
      <graded_buoyancy>
        <default_density>${format(rho)}</default_density>
        <density_change>
          <above_depth>0</above_depth>
          <density>0</density>
        </density_change>
      </graded_buoyancy>
      <enable>${xmlEscape(coeffs.id)}</enable>
    </plugin>`;
}

function renderWorldSdf(coeffs, maneuver, options = {}) {
    const maxStep = maneuver.dt;
    const current = maneuver.env?.waterV || {x: 0, y: 0, z: 0};
    const gravity = coeffs.buoyancy?.g || coeffs.restoring?.gravity || 9.81;
    const initialYawEnu = options.initialStateNed ? Math.PI / 2 - options.initialStateNed.yaw : Math.PI / 2;
    const initialEnu = options.initialStateNed ? {x:options.initialStateNed.E,y:options.initialStateNed.N,z:0} : {x:0,y:0,z:0};
    return `<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="bcod_parity_${xmlEscape(maneuver.name)}">
    <physics name="parity_physics" type="dart">
      <max_step_size>${format(maxStep)}</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <gravity>0 0 -${format(gravity)}</gravity>
    <include>
      <uri>model://${xmlEscape(coeffs.id)}</uri>
      <pose>${format(initialEnu.x)} ${format(initialEnu.y)} ${format(initialEnu.z)} 0 0 ${format(initialYawEnu)}</pose>
    </include>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <plugin filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench"/>
${renderBuoyancyPlugin(coeffs)}
    <!-- BCOD parity metadata: current ENU ${format(current.x || 0)} ${format(current.z || 0)} ${format(current.y || 0)} -->
  </world>
</sdf>
`;
}

function renderManifest(vehicleName, coeffs, maneuverName, maneuver) {
    return {
        schema: "bcod-gazebo-parity-manifest-v1",
        vehicle: vehicleName,
        coefficientSource: "../core/vehicles/coefficients.js",
        maneuver: maneuverName,
        dt: maneuver.dt,
        steps: maneuver.steps,
        durationSec: maneuver.dt * maneuver.steps,
        command: maneuver.command,
        env: maneuver.env,
        tolerances: maneuver.tolerances,
        expectedGoldenCsv: `golden/${vehicleName}/${maneuverName}.csv`,
        gazebo: {
            model: `models/${coeffs.id}/model.sdf`,
            world: `worlds/${vehicleName}_${maneuverName}.sdf`,
            actuationMode: "bodyWrench",
            commandTopics: [persistentWrenchTopicForWorld(maneuverName)],
            wrenchTopic: wrenchTopicForWorld(maneuverName),
            wrenchPersistentTopic: persistentWrenchTopicForWorld(maneuverName),
            wrenchClearTopic: `/world/bcod_parity_${maneuverName}/wrench/clear`,
            wrenchEntity: {name: `${coeffs.id}::base_link`, type: "LINK"},
            currentTopic: "/ocean_current",
            currentEnu: {x: maneuver.env?.waterV?.x || 0, y: maneuver.env?.waterV?.z || 0, z: -(maneuver.env?.waterV?.y || 0)},
            evidenceScope: "planar maneuver implementation cross-check only",
            coupled6HydrostaticsValidated: false,
            fixtureStabilization: {
                method: "vertical CG reflected below primitive buoyancy center",
                sourceBodyNedCg: [coeffs.massProps.cg?.x || 0, coeffs.massProps.cg?.y || 0, coeffs.massProps.cg?.z || 0],
                gazeboFixtureCg: [coeffs.massProps.cg?.x || 0, -(coeffs.massProps.cg?.y || 0), coeffs.massProps.cg?.z || 0],
                reason: "Gazebo primitive collision buoyancy does not reproduce surface-vessel metacentric hydrostatics."
            },
            effectors: (coeffs.effectors || []).map((effector) => ({
                id: effector.id,
                type: effector.type,
                gazeboPlugin: "ApplyLinkWrench",
                plannedGazeboPlugin: effector.type === "ControlSurface"
                    ? "LiftDrag"
                    : effector.type === "Rotor"
                        ? "MulticopterMotorModel"
                        : "Thruster"
            })),
            hullPrimitiveCount: coeffs.hullPrimitives?.length || 1
        }
    };
}

function writeGenerated(args, maneuverName) {
    const coeffs = vehicles[args.vehicle];
    if (!coeffs) {
        throw new Error(`Unknown vehicle '${args.vehicle}'. Available: ${Object.keys(vehicles).join(", ")}`);
    }
    const maneuver = {...getParityManeuver(maneuverName), name: maneuverName};
    const modelDir = path.join(args.outDir, "models", coeffs.id);
    const worldDir = path.join(args.outDir, "worlds");
    const manifestDir = path.join(args.outDir, "manifests");
    fs.mkdirSync(modelDir, {recursive: true});
    fs.mkdirSync(worldDir, {recursive: true});
    fs.mkdirSync(manifestDir, {recursive: true});
    fs.writeFileSync(path.join(modelDir, "model.sdf"), renderModelSdf(coeffs, args));
    fs.writeFileSync(path.join(modelDir, "model.config"), renderModelConfig(coeffs));
    fs.writeFileSync(path.join(worldDir, `${args.vehicle}_${maneuverName}.sdf`), renderWorldSdf(coeffs, maneuver));
    fs.writeFileSync(
        path.join(manifestDir, `${args.vehicle}_${maneuverName}.json`),
        JSON.stringify(renderManifest(args.vehicle, coeffs, maneuverName, maneuver), null, 2) + "\n"
    );
    return {
        vehicle: args.vehicle,
        maneuver: maneuverName,
        model: path.join(modelDir, "model.sdf"),
        modelConfig: path.join(modelDir, "model.config"),
        world: path.join(worldDir, `${args.vehicle}_${maneuverName}.sdf`),
        manifest: path.join(manifestDir, `${args.vehicle}_${maneuverName}.json`)
    };
}

if (process.argv[1] && process.argv[1].endsWith("generateGazeboParity.js")) {
    const args = parseArgs(process.argv);
    const maneuvers = args.all ? listParityManeuverNames() : [args.maneuver];
    const outputs = maneuvers.map((name) => writeGenerated(args, name));
    console.log(JSON.stringify({generated: outputs}, null, 2));
}

export {
    addedMassMatrix3,
    assertAddedMassIsValid,
    assertGazeboFluidAddedMassIsValid,
    gazeboFluidAddedMassMatrix6,
    renderManifest,
    renderModelConfig,
    renderModelSdf,
    renderWorldSdf,
    writeGenerated
};
