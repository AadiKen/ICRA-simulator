import {ForceModel} from "./force-model.js";

function clamp(value, min, max) {
    return Math.min(Math.max(value, min), max);
}

function requireFinite(value, label) {
    if (!Number.isFinite(value)) {
        throw new Error(`${label} must be finite; received ${String(value)}.`);
    }
    return value;
}

function optionalFinite(value, fallback, label) {
    return value === undefined || value === null ? fallback : requireFinite(value, label);
}

function requireFiniteVector(value, length, label) {
    if (!Array.isArray(value) || value.length !== length) {
        throw new Error(`${label} must be a ${length}-component vector.`);
    }
    return value.map((component, index) => requireFinite(component, `${label}[${index}]`));
}

function normalizeWrench(value, label = "desired wrench") {
    if (!Array.isArray(value) || (value.length !== 3 && value.length !== 6)) {
        throw new Error(`${label} must be a 3- or 6-component vector.`);
    }
    const finite = value.map((component, index) => requireFinite(component, `${label}[${index}]`));
    return finite.length === 6 ? finite : [finite[0], finite[1], 0, 0, 0, finite[2]];
}

function normalize(v, fallback = [1, 0, 0]) {
    const finite = requireFiniteVector(v, 3, "actuator direction");
    const mag = Math.hypot(finite[0], finite[1], finite[2]);
    if (mag === 0) {
        return [...fallback];
    }
    return finite.map((component) => component / mag);
}

function cross(a, b) {
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    ];
}

function dot(a, b) {
    return a.reduce((sum, value, idx) => sum + value * b[idx], 0);
}

function scale(v, s) {
    return v.map((value) => value * s);
}

function add3(a, b) {
    return a.map((value, idx) => value + b[idx]);
}

function add6(a, b) {
    return a.map((value, idx) => value + b[idx]);
}

function cgVector(params) {
    const cg = params.massProps?.cg || {x: 0, y: 0, z: 0};
    return [cg.x || 0, cg.y || 0, cg.z || 0];
}

function armFromCg(pos, params) {
    const cg = cgVector(params);
    return [(pos[0] || 0) - cg[0], (pos[1] || 0) - cg[1], (pos[2] || 0) - cg[2]];
}

function firstOrderStep(current, target, dynamics, dt) {
    requireFinite(current, "actuator state");
    requireFinite(target, "actuator target");
    requireFinite(dt, "actuator dt");
    const min = dynamics.min ?? -Infinity;
    const max = dynamics.max ?? Infinity;
    const clampedTarget = clamp(target, min, max);
    if (dt <= 0) {
        return clampedTarget;
    }
    const configuredTau = optionalFinite(dynamics.tau, 0.001, "actuator time constant");
    const tau = Math.max(configuredTau === 0 ? 0.001 : configuredTau, 0.001);
    const rawDelta = (clampedTarget - current) * (1 - Math.exp(-dt / tau));
    const maxDelta = Number.isFinite(dynamics.rateMax)
        ? Math.max(dynamics.rateMax, 0) * Math.max(dt, 0)
        : Infinity;
    return current + clamp(rawDelta, -maxDelta, maxDelta);
}

class ScalarEffector {
    constructor(config, params, defaultDynamics = {}) {
        this.id = config.id;
        this.type = config.type;
        this.pos = config.pos || config.positionBody || [0, 0, 0];
        this.axis = normalize(config.axis || config.directionBody || [1, 0, 0]);
        this.dynamics = {
            tau: config.dynamics?.tau ?? defaultDynamics.tau ?? params.actuator?.motorTimeConstant ?? 0.001,
            rateMax: config.dynamics?.rateMax ?? defaultDynamics.rateMax ?? Infinity,
            min: config.dynamics?.min ?? defaultDynamics.min ?? -1,
            max: config.dynamics?.max ?? defaultDynamics.max ?? 1
        };
        this.conversion = config.conversion || {type: "linear"};
        this.integratedCapabilities = (config.behaviorVersion ?? params.actuator?.behaviorVersion) === "integrated-v1";
        this.deadZone = this.integratedCapabilities ? Math.max(0, config.deadZone ?? this.conversion.deadZone ?? 0) : 0;
        this.power = {
            idle: Math.max(0, config.power?.idleW ?? config.idlePowerW ?? 0),
            control: Math.max(0, config.power?.controlW ?? config.controlPowerW ?? 0),
            maximum: Math.max(0, config.power?.maxW ?? config.maxPowerW ?? 0)
        };
        this.failureMode = "healthy";
        this.stuckState = null;
        this.energyJ = 0;
        this.propulsionEnergyJ = 0;
        this.lastPowerW = 0;
        this.command = 0;
        this.value = 0;
        this.failureMode = "healthy";
        this.stuckState = null;
        this.energyJ = 0;
        this.propulsionEnergyJ = 0;
        this.lastPowerW = 0;
    }

    reset() {
        this.command = 0;
        this.value = 0;
    }

    commandToValue(command) {
        return command;
    }

    advance(dt, command = {}) {
        for (const field of ["thrust", "value", "deflection", "speed", "command"]) {
            if (command[field] !== undefined && command[field] !== null) {
                requireFinite(command[field], `${this.id}.${field}`);
            }
        }
        if (this.integratedCapabilities && this.failureMode === "failed-off") {
            this.value = 0;
            this.command = 0;
            this.lastPowerW = 0;
            return this.value;
        }
        if (this.integratedCapabilities && this.failureMode === "stuck" && this.stuckState) {
            this.restoreStuckState();
            this.lastPowerW = this.powerFor(this.command, this.value);
            this.energyJ += this.lastPowerW * Math.max(dt, 0);
            this.propulsionEnergyJ += this.propulsionPowerFor(this.value) * Math.max(dt, 0);
            return this.value;
        }
        let target;
        if (Number.isFinite(command.thrust)) {
            target = command.thrust;
        }
        else if (Number.isFinite(command.value)) {
            target = command.value;
        }
        else if (Number.isFinite(command.deflection)) {
            target = command.deflection;
        }
        else if (Number.isFinite(command.speed)) {
            target = command.speed;
        }
        else {
            const normalized = clamp(command.command ?? 0, -1, 1);
            target = this.integratedCapabilities && Math.abs(normalized) < this.deadZone ? 0 : this.commandToValue(normalized);
            this.command = normalized;
        }
        this.value = firstOrderStep(this.value, target, this.dynamics, dt);
        this.lastPowerW = this.integratedCapabilities ? this.powerFor(this.command, this.value) : 0;
        this.energyJ += this.lastPowerW * Math.max(dt, 0);
        this.propulsionEnergyJ += this.propulsionPowerFor(this.value) * Math.max(dt, 0);
        return this.value;
    }

    powerFor(command, value) {
        if (!this.integratedCapabilities || this.failureMode === "failed-off" || Math.abs(command) === 0) return 0;
        const productive = Math.abs(value) / Math.max(Math.abs(this.dynamics.min), Math.abs(this.dynamics.max), 1e-12);
        return Math.min(this.power.maximum || Infinity, this.power.idle + this.power.control * Math.min(Math.abs(command), 1) + (this.power.maximum || 0) * productive);
    }

    propulsionPowerFor(value) {
        if (!this.integratedCapabilities || this.failureMode === "failed-off") return 0;
        const productive = Math.abs(value) / Math.max(Math.abs(this.dynamics.min), Math.abs(this.dynamics.max), 1e-12);
        return (this.power.maximum || 0) * productive;
    }

    captureStuckState() {
        this.stuckState = Object.fromEntries(["command", "value", "thrust", "azimuth", "deflection", "omega", "axis"].filter((key) => this[key] !== undefined).map((key) => [key, Array.isArray(this[key]) ? [...this[key]] : this[key]]));
    }

    restoreStuckState() {
        for (const [key, value] of Object.entries(this.stuckState || {})) this[key] = Array.isArray(value) ? [...value] : value;
    }

    forceMoment(force, params, extraMoment = [0, 0, 0]) {
        const moment = add3(cross(armFromCg(this.pos, params), force), extraMoment);
        return [force[0], force[1], force[2], moment[0], moment[1], moment[2]];
    }

    wrenchWithUnitCommand(params, state = null, env = {}) {
        const oldValue = this.value;
        const oldCommand = this.command;
        this.advance(0, {command: 1});
        const wrench = this.wrench(params, state, env);
        this.value = oldValue;
        this.command = oldCommand;
        return wrench;
    }
}

export class FixedThruster extends ScalarEffector {
    constructor(config, params) {
        const max = config.maxForwardThrust ?? config.maxThrust ?? params.actuator?.maxThrust ?? 0;
        const min = -(config.maxReverseThrust ?? config.maxThrust ?? params.actuator?.maxThrust ?? 0);
        super(config, params, {min, max});
        this.type = "FixedThruster";
        this.thrust = this.value;
    }

    commandToValue(command) {
        if (this.conversion.type === "quadratic" || this.conversion.type === "curve") {
            const max = command >= 0 ? this.dynamics.max : Math.abs(this.dynamics.min);
            return Math.sign(command) * max * command * command;
        }
        if (this.conversion.type === "propeller") {
            const rho = this.conversion.rho || 1025;
            const ct = this.conversion.CT || this.conversion.C_T || 0;
            const diameter = this.conversion.diameter || 0;
            const n = command * (this.conversion.maxRevPerSec || 1);
            return rho * ct * diameter ** 4 * n * Math.abs(n);
        }
        return command >= 0
            ? command * this.dynamics.max
            : command * Math.abs(this.dynamics.min);
    }

    advance(dt, command = {}) {
        this.thrust = super.advance(dt, command);
        return this.thrust;
    }

    wrench(params) {
        const force = scale(this.axis, this.thrust);
        let reaction = [0, 0, 0];
        if (this.conversion.type === "propeller" && Number.isFinite(this.conversion.CQ || this.conversion.C_Q)) {
            const rho = this.conversion.rho || 1025;
            const cq = this.conversion.CQ || this.conversion.C_Q || 0;
            const diameter = this.conversion.diameter || 0;
            const n = Math.sign(this.thrust) * Math.sqrt(Math.abs(this.thrust) / Math.max(rho * (this.conversion.CT || this.conversion.C_T || 1) * diameter ** 4, 1e-9));
            reaction = scale(this.axis, rho * cq * diameter ** 5 * n * Math.abs(n));
        }
        return this.forceMoment(force, params, reaction);
    }

    wrenchWithUnitThrust(params) {
        const old = this.thrust;
        const oldValue = this.value;
        this.thrust = 1;
        this.value = 1;
        const wrench = this.wrench(params);
        this.thrust = old;
        this.value = oldValue;
        return wrench;
    }
}

export class AzimuthThruster extends FixedThruster {
    constructor(config, params) {
        super(config, params);
        this.type = "AzimuthThruster";
        this.steer = {
            tau: config.steer?.tau ?? config.azimuth?.tau ?? 0.001,
            rateMax: config.steer?.rateMax ?? config.azimuth?.rateMax ?? Infinity,
            min: config.steer?.min ?? config.azimuth?.min ?? -Math.PI,
            max: config.steer?.max ?? config.azimuth?.max ?? Math.PI
        };
        this.azimuth = config.azimuth?.initial ?? config.steer?.initial ?? 0;
    }

    advance(dt, command = {}) {
        super.advance(dt, command);
        if (this.integratedCapabilities && this.failureMode === "stuck") return this.thrust;
        if (command.azimuth !== undefined && command.azimuth !== null) requireFinite(command.azimuth, `${this.id}.azimuth`);
        const target = Number.isFinite(command.azimuth) ? command.azimuth : this.azimuth;
        this.azimuth = firstOrderStep(this.azimuth, target, this.steer, dt);
        this.axis = [Math.cos(this.azimuth), Math.sin(this.azimuth), 0];
        return this.thrust;
    }
}

export class ControlSurface extends ScalarEffector {
    constructor(config, params) {
        super(config, params, {
            min: config.dynamics?.min ?? -0.6,
            max: config.dynamics?.max ?? 0.6
        });
        this.type = "ControlSurface";
        this.foil = config.foil || {};
        this.forceAxis = normalize(config.forceAxis || config.axis || [0, 1, 0], [0, 1, 0]);
        this.inducedInflow = config.inducedInflow === true;
        this.deflection = this.value;
    }

    commandToValue(command) {
        const maxAbs = Math.max(Math.abs(this.dynamics.min), Math.abs(this.dynamics.max));
        return command * maxAbs;
    }

    advance(dt, command = {}) {
        this.deflection = super.advance(dt, command);
        return this.deflection;
    }

    liftCoefficient(alpha) {
        const clAlpha = this.foil.C_Lalpha ?? this.foil.CLalpha ?? 2 * Math.PI;
        const stall = Math.abs(this.foil.alphaStall ?? 0.6);
        const sign = Math.sign(alpha) || 1;
        const absAlpha = Math.abs(alpha);
        if (absAlpha <= stall) {
            return clAlpha * alpha;
        }
        return sign * clAlpha * stall * Math.max(0.25, stall / absAlpha);
    }

    dragCoefficient(cl) {
        return (this.foil.C_D0 ?? this.foil.CD0 ?? 0.01) + (this.foil.k ?? 0.05) * cl * cl;
    }

    localVelocity(state = null) {
        const velocity = state?.velocity || {u: 0, v: 0, w: 0};
        const angular = state?.angularRate || {p: 0, q: 0, r: 0};
        const r = armFromCg(this.pos, {massProps: {cg: {x: 0, y: 0, z: 0}}});
        const rotational = cross([angular.p || 0, angular.q || 0, angular.r || 0], r);
        return [
            (velocity.u || 0) + rotational[0],
            (velocity.v || 0) + rotational[1],
            (velocity.w || 0) + rotational[2]
        ];
    }

    wrench(params, state = null, env = {}) {
        let v = this.localVelocity(state);
        if (this.inducedInflow && env.inducedInflowBody) {
            v = add3(v, env.inducedInflowBody);
        }
        const speed = Math.hypot(v[0], v[1], v[2]);
        if (speed < 1e-9) {
            return [0, 0, 0, 0, 0, 0];
        }
        const rho = this.foil.rho || params.buoyancy?.rho || 1025;
        const area = this.foil.A || this.foil.area || 0;
        const alpha = this.deflection + (this.foil.alpha0 || 0);
        const cl = this.liftCoefficient(alpha);
        const cd = this.dragCoefficient(cl);
        const q = 0.5 * rho * speed * speed;
        const lift = q * area * cl;
        const drag = q * area * cd;
        const dragAxis = normalize(scale(v, -1), [-1, 0, 0]);
        const liftAxis = normalize(this.forceAxis, [0, 1, 0]);
        const force = add3(scale(liftAxis, lift), scale(dragAxis, drag));
        return this.forceMoment(force, params);
    }
}

export class Rotor extends ScalarEffector {
    constructor(config, params) {
        super(config, params, {
            min: config.dynamics?.min ?? 0,
            max: config.dynamics?.max ?? config.conversion?.maxOmega ?? 1000
        });
        this.type = "Rotor";
        this.spinDirection = config.spinDirection || config.conversion?.spinDirection || 1;
        this.omega = this.value;
    }

    commandToValue(command) {
        return command * this.dynamics.max;
    }

    advance(dt, command = {}) {
        this.omega = super.advance(dt, command);
        return this.omega;
    }

    wrench(params) {
        const kt = this.conversion.k_T ?? this.conversion.kT ?? 0;
        const kq = this.conversion.k_Q ?? this.conversion.kQ ?? 0;
        const thrust = kt * this.omega * this.omega;
        const torque = -Math.sign(this.spinDirection || 1) * kq * this.omega * this.omega;
        const force = scale(this.axis, thrust);
        return this.forceMoment(force, params, scale(this.axis, torque));
    }
}

const DOF_ROWS = {
    surge: 0,
    sway: 1,
    heave: 2,
    roll: 3,
    pitch: 4,
    yaw: 5
};

function transpose(matrix) {
    return matrix[0].map((_, col) => matrix.map((row) => row[col]));
}

function matMul(a, b) {
    return a.map((row) => b[0].map((_, col) => row.reduce((sum, value, idx) => sum + value * b[idx][col], 0)));
}

function matVec(matrix, vector) {
    return matrix.map((row) => dot(row, vector));
}

function solveLinearSystem(a, b) {
    const n = b.length;
    const m = a.map((row, idx) => [...row, b[idx]]);
    for (let col = 0; col < n; col += 1) {
        let pivot = col;
        for (let row = col + 1; row < n; row += 1) {
            if (Math.abs(m[row][col]) > Math.abs(m[pivot][col])) {
                pivot = row;
            }
        }
        if (Math.abs(m[pivot][col]) < 1e-10) {
            m[pivot][col] = 1e-10;
        }
        [m[col], m[pivot]] = [m[pivot], m[col]];
        const divisor = m[col][col];
        for (let c = col; c <= n; c += 1) {
            m[col][c] /= divisor;
        }
        for (let row = 0; row < n; row += 1) {
            if (row === col) {
                continue;
            }
            const factor = m[row][col];
            for (let c = col; c <= n; c += 1) {
                m[row][c] -= factor * m[col][c];
            }
        }
    }
    return m.map((row) => row[n]);
}

function allocationSpectrum(matrix) {
    if (!matrix.length || !matrix[0]?.length) return {rank:0,singularValues:[],conditionNumber:null};
    const gram=matMul(matrix,transpose(matrix)), n=gram.length;
    for(let sweep=0;sweep<50;sweep+=1){let p=0,q=Math.min(1,n-1),largest=0;for(let i=0;i<n;i+=1)for(let j=i+1;j<n;j+=1)if(Math.abs(gram[i][j])>largest){largest=Math.abs(gram[i][j]);p=i;q=j;}if(largest<1e-12)break;const phi=0.5*Math.atan2(2*gram[p][q],gram[q][q]-gram[p][p]),c=Math.cos(phi),s=Math.sin(phi);for(let k=0;k<n;k+=1)if(k!==p&&k!==q){const kp=gram[k][p],kq=gram[k][q];gram[k][p]=gram[p][k]=c*kp-s*kq;gram[k][q]=gram[q][k]=s*kp+c*kq;}const pp=c*c*gram[p][p]-2*s*c*gram[p][q]+s*s*gram[q][q],qq=s*s*gram[p][p]+2*s*c*gram[p][q]+c*c*gram[q][q];gram[p][p]=pp;gram[q][q]=qq;gram[p][q]=gram[q][p]=0;}
    const singularValues=gram.map((row,index)=>Math.sqrt(Math.max(row[index],0))).sort((a,b)=>b-a), tolerance=Math.max((singularValues[0]||0)*1e-9,1e-10), rank=singularValues.filter((value)=>value>tolerance).length;
    return {rank,singularValues,conditionNumber:rank===n&&singularValues.at(-1)>tolerance?singularValues[0]/singularValues.at(-1):null};
}

export function allocationMatrix(effectors, params, controlledDOF = ["surge", "yaw"], state = null, env = {}) {
    const rows = controlledDOF.map((dof) => DOF_ROWS[dof]).filter((idx) => Number.isFinite(idx));
    const columns = effectors.map((effector) => requireFiniteVector(
        effector.wrenchWithUnitCommand(params, state, env),
        6,
        `unit wrench for effector ${effector.id ?? "<unknown>"}`
    ));
    return rows.map((row) => columns.map((column) => column[row]));
}

export function allocateLeastSquares(effectors, params, tau6, options = {}, state = null, env = {}) {
    const normalizedWrench = normalizeWrench(tau6, "allocation target");
    const controlledDOF = options.controlledDOF || params.controlledDOF || ["surge", "yaw"];
    const rows = controlledDOF.map((dof) => DOF_ROWS[dof]).filter((idx) => Number.isFinite(idx));
    const desired = rows.map((row) => normalizedWrench[row]);
    const b = allocationMatrix(effectors, params, controlledDOF, state, env);
    const bt = transpose(b);
    const gram = matMul(b, bt);
    const lambda = options.regularization ?? 1e-9;
    const regularized = gram.map((row, idx) => row.map((value, col) => value + (idx === col ? lambda : 0)));
    const y = solveLinearSystem(regularized, desired);
    const commandVector = matVec(bt, y).map((value, index) => requireFinite(value, `allocation command[${index}]`));
    const saturation = options.saturation || params.allocator?.saturation || "scale";
    const limited = commandVector.map((value) => clamp(value, -1, 1));
    if (saturation === "scale") {
        const maxAbs = commandVector.reduce((max, value) => Math.max(max, Math.abs(value)), 1);
        return commandVector.map((value) => value / maxAbs);
    }
    return limited;
}

export class ActuationModel extends ForceModel {
    constructor(params) {
        super();
        if (params.actuator?.maxThrust !== undefined) requireFinite(params.actuator.maxThrust, "actuator maxThrust");
        if (params.actuator?.beam !== undefined) requireFinite(params.actuator.beam, "actuator beam");
        if (params.actuator?.motorTimeConstant !== undefined) requireFinite(params.actuator.motorTimeConstant, "actuator motorTimeConstant");
        if (params.geometry?.beam !== undefined) requireFinite(params.geometry.beam, "geometry beam");
        this.params = params;
        this.effectors = buildEffectors(params);
        this.lastFullWrench = [0, 0, 0, 0, 0, 0];
        this.lastEffectorCommands = {};
        this.integratedCapabilities = params.actuator?.behaviorVersion === "integrated-v1";
        this.stepIndex = 0;
        this.events = [];
        this.lastAllocationDiagnostics = null;
    }

    reset() {
        this.effectors.forEach((effector) => effector.reset());
        this.lastFullWrench = [0, 0, 0, 0, 0, 0];
        this.lastEffectorCommands = {};
        this.stepIndex = 0;
        this.events = [];
        this.lastAllocationDiagnostics = null;
    }

    saveState() {
        const keys = ["command", "value", "thrust", "azimuth", "deflection", "omega", "failureMode", "stuckState", "energyJ", "propulsionEnergyJ", "lastPowerW"];
        return {effectors: this.effectors.map((effector) => Object.fromEntries([["id", effector.id], ...keys.filter((key) => effector[key] !== undefined).map((key) => [key, structuredClone(effector[key])]), ...[effector.axis ? ["axis", [...effector.axis]] : null].filter(Boolean)])), lastFullWrench: [...this.lastFullWrench], lastEffectorCommands: structuredClone(this.lastEffectorCommands), stepIndex:this.stepIndex, events:structuredClone(this.events), lastAllocationDiagnostics:structuredClone(this.lastAllocationDiagnostics)};
    }

    loadState(state) {
        if (!state || !Array.isArray(state.effectors)) throw new Error("Invalid actuator checkpoint state.");
        const byId = new Map(state.effectors.map((effector) => [effector.id, effector]));
        this.effectors.forEach((effector) => {
            const saved = byId.get(effector.id);
            if (!saved) throw new Error(`Actuator checkpoint is missing effector ${effector.id}.`);
            for (const key of ["command", "value", "thrust", "azimuth", "deflection", "omega", "failureMode", "stuckState", "energyJ", "propulsionEnergyJ", "lastPowerW"]) if (saved[key] !== undefined) effector[key] = structuredClone(saved[key]);
            if (saved.axis) effector.axis = [...saved.axis];
        });
        this.lastFullWrench = [...state.lastFullWrench];
        this.lastEffectorCommands = structuredClone(state.lastEffectorCommands);
        this.stepIndex = state.stepIndex ?? 0;
        this.events = structuredClone(state.events ?? []);
        this.lastAllocationDiagnostics = structuredClone(state.lastAllocationDiagnostics ?? null);
    }

    setFailureMode(id, mode, source = "command") {
        if (!this.integratedCapabilities) return;
        if (!["healthy", "failed-off", "stuck"].includes(mode)) throw new Error(`Unsupported actuator failure mode: ${mode}`);
        const effector = this.effectors.find((item) => item.id === id);
        if (!effector) throw new Error(`Unknown actuator: ${id}`);
        if (effector.failureMode === mode) return;
        const previous = effector.failureMode;
        if (mode === "stuck") effector.captureStuckState();
        if (mode === "failed-off") effector.stuckState = null;
        effector.failureMode = mode;
        this.events.push({type:mode === "healthy" ? "ACTUATOR_STATE_TRANSITION" : "ACTUATOR_FAILURE", actuator_id:id, step:this.stepIndex, mode, previous_mode:previous, source});
    }

    drainEvents() { const drained=structuredClone(this.events); this.events=[]; return drained; }

    getEnergyMetrics() {
        return {actuator_energy_j:this.effectors.reduce((sum, effector) => sum + effector.energyJ, 0), propulsion_energy_j:this.effectors.reduce((sum, effector) => sum + effector.propulsionEnergyJ, 0), actuator_power_w:this.effectors.reduce((sum, effector) => sum + effector.lastPowerW, 0)};
    }

    commandWrench(command = {}, dt = 0, state = null, env = {}) {
        requireFinite(dt, "actuator dt");
        if (command.appliedWrench) {
            this.lastFullWrench = normalizeWrench(command.appliedWrench, "appliedWrench");
            return command.appliedWrench;
        }

        if (this.integratedCapabilities && command.failureStates) for (const [id, failure] of Object.entries(command.failureStates)) this.setFailureMode(id, typeof failure === "string" ? failure : failure.mode, typeof failure === "string" ? "command" : failure.source ?? "command");
        const effectorCommands = this.resolveEffectorCommands(command, state, env);
        this.lastEffectorCommands = effectorCommands;
        this.effectors.forEach((effector) => {
            effector.advance(dt, effectorCommands[effector.id] || {});
        });
        const full = this.effectors.reduce((sum, effector) => {
            return add6(sum, effector.wrench(this.params, state, env));
        }, [0, 0, 0, 0, 0, 0]);
        this.lastFullWrench = full;
        this.stepIndex += 1;
        return [full[0], full[1], full[5]];
    }

    prepareStep(ctx) {
        this.commandWrench(ctx.command || {}, ctx.dt ?? 0, ctx.state, ctx.env);
    }

    computeWrench(ctx) {
        if (ctx.command?.appliedWrench) {
            return ctx.command.appliedWrench;
        }
        const full = this.effectors.reduce((sum, effector) => {
            return add6(sum, effector.wrench(this.params, ctx.state, ctx.env));
        }, [0, 0, 0, 0, 0, 0]);
        this.lastFullWrench = full;
        return [full[0], full[1], full[5]];
    }

    resolveEffectorCommands(command = {}, state = null, env = {}) {
        if (command.effectors) {
            return {...command.effectors};
        }
        const explicit = this.resolveExplicitCommands(command);
        if (explicit) {
            return explicit;
        }
        const desired = this.resolveDesiredWrench(command);
        return this.allocate(desired, state, env);
    }

    resolveExplicitCommands(command) {
        const output = {};
        let found = false;
        this.effectors.forEach((effector) => {
            const id = effector.id;
            const commandKey = `${id}Command`;
            const thrustKey = `${id}Thrust`;
            const deflectionKey = `${id}Deflection`;
            for (const key of [commandKey, thrustKey, deflectionKey]) {
                if (command[key] !== undefined && command[key] !== null) requireFinite(command[key], key);
            }
            if (Number.isFinite(command[commandKey])) {
                output[id] = {command: command[commandKey]};
                found = true;
            }
            if (Number.isFinite(command[thrustKey])) {
                output[id] = {thrust: command[thrustKey]};
                found = true;
            }
            if (Number.isFinite(command[deflectionKey])) {
                output[id] = {deflection: command[deflectionKey]};
                found = true;
            }
        });
        ["port", "starboard"].forEach((id) => {
            if (Number.isFinite(command[`${id}Command`]) && this.effectors.find((e) => e.id === id)) {
                output[id] = {command: command[`${id}Command`]};
                found = true;
            }
            if (Number.isFinite(command[`${id}Thrust`]) && this.effectors.find((e) => e.id === id)) {
                output[id] = {thrust: command[`${id}Thrust`]};
                found = true;
            }
        });
        return found ? output : null;
    }

    resolveDesiredWrench(command) {
        if (command.tauDes) {
            return normalizeWrench(command.tauDes, "tauDes");
        }
        if (command.desiredWrench) {
            return normalizeWrench(command.desiredWrench, "desiredWrench");
        }
        const beam = optionalFinite(this.params.actuator?.beam ?? this.params.geometry?.beam, 1, "actuator beam");
        const maxThrust = optionalFinite(this.params.actuator?.maxThrust, 0, "actuator maxThrust");
        const surge = clamp(optionalFinite(command.surgeForce, 0, "surgeForce"), -2 * maxThrust, 2 * maxThrust);
        const yaw = command.yawMoment !== undefined && command.yawMoment !== null
            ? requireFinite(command.yawMoment, "yawMoment")
            : clamp(optionalFinite(command.differentialForce, 0, "differentialForce"), -maxThrust, maxThrust) * (beam * 0.5);
        return [surge, 0, 0, 0, 0, yaw];
    }

    allocate(tau6 = [0, 0, 0, 0, 0, 0], state = null, env = {}) {
        tau6 = normalizeWrench(tau6, "allocation target");
        if (this.integratedCapabilities) return this.allocateIntegrated(tau6, state, env);
        const ids = this.effectors.map((effector) => effector.id);
        if (ids.includes("port") && ids.includes("starboard") && this.effectors.every((e) => e instanceof FixedThruster)) {
            const port = this.effectors.find((effector) => effector.id === "port");
            const starboard = this.effectors.find((effector) => effector.id === "starboard");
            const portYawPerNewton = port.wrenchWithUnitThrust(this.params)[5];
            const starboardYawPerNewton = starboard.wrenchWithUnitThrust(this.params)[5];
            const surge = tau6[0];
            const yaw = tau6[5];
            const denom = starboardYawPerNewton - portYawPerNewton;
            let starboardThrust = surge * 0.5;
            let portThrust = surge * 0.5;
            if (Math.abs(denom) > 1e-9) {
                const delta = yaw / denom;
                starboardThrust += delta;
                portThrust -= delta;
            }
            return {
                port: {thrust: clamp(portThrust, port.dynamics.min, port.dynamics.max)},
                starboard: {thrust: clamp(starboardThrust, starboard.dynamics.min, starboard.dynamics.max)}
            };
        }

        const commandVector = allocateLeastSquares(this.effectors, this.params, tau6, {
            ...(this.params.allocator || {}),
            controlledDOF: this.params.controlledDOF || ["surge", "yaw"]
        }, state, env);
        return Object.fromEntries(this.effectors.map((effector, idx) => [
            effector.id,
            {command: requireFinite(commandVector[idx], `allocation command[${idx}]`)}
        ]));
    }

    allocateIntegrated(tau6, state = null, env = {}) {
        const stuck = this.effectors.filter((effector) => effector.failureMode === "stuck");
        const controllable = this.effectors.filter((effector) => effector.failureMode === "healthy");
        const bias = stuck.reduce((sum, effector) => add6(sum, effector.wrench(this.params, state, env)), [0,0,0,0,0,0]);
        const residualTarget = tau6.map((value, index) => value - bias[index]);
        const controlledDOF = this.params.controlledDOF || ["surge", "yaw"];
        const matrix = controllable.length ? allocationMatrix(controllable, this.params, controlledDOF, state, env) : controlledDOF.map(() => []);
        const commands = controllable.length ? allocateLeastSquares(controllable, this.params, residualTarget, {...(this.params.allocator || {}), controlledDOF, regularization:this.params.allocator?.regularization ?? 1e-3}, state, env) : [];
        const output = Object.fromEntries(this.effectors.map((effector) => [effector.id, effector.failureMode === "healthy" ? {command:requireFinite(commands[controllable.indexOf(effector)], `allocation command for ${effector.id}`)} : {}]));
        const achieved = controllable.reduce((sum, effector, index) => { const oldCommand=effector.command, oldValue=effector.value; effector.advance(0,{command:requireFinite(commands[index], `allocation command for ${effector.id}`)}); const wrench=effector.wrench(this.params,state,env); effector.command=oldCommand; effector.value=oldValue; if ("thrust" in effector) effector.thrust=oldValue; return add6(sum,wrench); }, [...bias]);
        const residual = tau6.map((value,index)=>value-achieved[index]);
        const spectrum=allocationSpectrum(matrix);
        const residualNorm = Math.hypot(...controlledDOF.map((dof)=>requireFinite(residual[DOF_ROWS[dof]], `allocation residual ${dof}`)));
        this.lastAllocationDiagnostics={mode:stuck.length?"stuck-bias-rejection":this.effectors.some((e)=>e.failureMode==="failed-off")?"failed-off-reduced-set":"nominal",rank:spectrum.rank,condition_number:spectrum.conditionNumber,singular_values:spectrum.singularValues,bias_wrench:[...bias],achieved_wrench:achieved,residual_wrench:residual,reachable:residualNorm<=Math.max(this.params.allocator?.reachabilityTolerance ?? 1e-6,1e-12),degradation:spectrum.rank<controlledDOF.length?"rank-deficient":residualNorm>1e-6?"infeasible":"none"};
        return output;
    }
}

export function buildEffectors(params) {
    const configured = params.effectors || params.actuator?.effectors;
    if (configured?.length) {
        return configured.map((config) => {
            if (config.type === "FixedThruster") {
                return new FixedThruster(config, params);
            }
            if (config.type === "AzimuthThruster") {
                return new AzimuthThruster(config, params);
            }
            if (config.type === "ControlSurface") {
                return new ControlSurface(config, params);
            }
            if (config.type === "Rotor") {
                return new Rotor(config, params);
            }
            throw new Error(`Unsupported effector type: ${config.type}`);
        });
    }
    const beam = optionalFinite(params.actuator?.beam ?? params.geometry?.beam, 1, "actuator beam");
    const maxThrust = optionalFinite(params.actuator?.maxThrust, 0, "actuator maxThrust");
    const configuredTau = optionalFinite(params.actuator?.motorTimeConstant, 0.001, "actuator motorTimeConstant");
    const tau = configuredTau === 0 ? 0.001 : configuredTau;
    return [
        new FixedThruster({
            id: "port",
            type: "FixedThruster",
            pos: [0, -beam * 0.5, 0],
            axis: [1, 0, 0],
            dynamics: {tau, min: -maxThrust, max: maxThrust},
            conversion: {type: "linear"}
        }, params),
        new FixedThruster({
            id: "starboard",
            type: "FixedThruster",
            pos: [0, beam * 0.5, 0],
            axis: [1, 0, 0],
            dynamics: {tau, min: -maxThrust, max: maxThrust},
            conversion: {type: "linear"}
        }, params)
    ];
}

export class ActuatorModel extends ActuationModel {}
