import {RigidBodyState} from "./core/rigidBodyState.js";
import {VehicleParameters} from "./core/vehicleParameters.js";
import {DynamicsCore} from "./core/dynamicsCore.js";
import {AddedMassCoriolis,HydrodynamicDamping} from "./packages/core/src/force-components.js";
import {HydrostaticsAndWaves} from "./packages/core/src/wave-forces.js";
import {ActuatorModel} from "./packages/core/src/actuators.js";
import {computeSubmergedState} from "./core/forces/submergedGeometry.js";
import {legacyGuidanceToActuatorCommand} from "./adapters/legacyGuidanceAdapter.js";
import {CoupledSixPlant} from "./core/coupledSixPlant.js";
import {PHYSICS_MODES, normalizePhysicsMode} from "./core/marinePlant.js";
import {WindLoad} from "./packages/core/src/environment-forces.js";
import {VehicleBMmgForceModel} from "./packages/core/src/vehicle-b-mmg.js";
import {rotationBodyToNed} from "./core/sixDof.js";

export class scenarioConfig{
    constructor(simConfig, boatConfig, sensorConfig, envConfig, goalConfig, controlConfig){
        this.simConfig = simConfig;
        this.boatConfig = boatConfig;
        this.sensorConfig = sensorConfig;
        this.envConfig = envConfig;
        this.goalConfig = goalConfig;
        this.controlConfig = controlConfig;

    }
}

export class simConfig{
    constructor(simHz, durationSec, seed, allowGroundTruth, physicsMode = PHYSICS_MODES.COUPLED6, options = {}){
        this.simHz = simHz;
        this.durationSec = durationSec;
        this.seed = seed
        this.allowGroundTruth = allowGroundTruth;
        this.physicsMode = normalizePhysicsMode(physicsMode);
        this.waveCoupling = options.waveCoupling || "none";
        this.logEvery = Math.max(Math.floor(options.logEvery || 1), 1);
    }
}

export function timeOfDayFromTime(time){
    const dayTime = time % (24 * 3600);

    if (dayTime > 21 * 3600 || dayTime < 3 * 3600){
        return "night";
    }
    if (dayTime >= 3 * 3600 && dayTime < 9 * 3600){
        return "dawn";
    }
    if (dayTime >= 9 * 3600 && dayTime < 15 * 3600){
        return "day";
    }
    return "dusk";
}

export function startTimeFromTimeOfDay(timeOfDay){
    if (timeOfDay === "night"){
        return 0;
    }
    if (timeOfDay === "dawn"){
        return 6 * 3600;
    }
    if (timeOfDay === "day"){
        return 12 * 3600;
    }
    if (timeOfDay === "dusk"){
        return 18 * 3600;
    }
    return 0;
}

export class simState{
    constructor(startTime, boatState, boatBelief, goalState, sensorsState, envState, controlState, metricsState){
        this.isSimulating = true;
        this.stopReason = null;

        this.startTime = startTime;
        this.time = startTime;
        this.tick = 0;
        this.steps = 0;
        this.stepTime = null;
        this.timeOfDay = timeOfDayFromTime(startTime);

        this.boat = boatState;
        this.boatBelief = boatBelief;
        this.goal = goalState;
        this.sensors = sensorsState;
        this.env = envState;
        this.control = controlState;
        this.metrics = metricsState;

        this.localEnv = null;
        this.activeSensors = {};
        this.lastCommand = null;
        this.lastCommandTime = -Infinity;
        this.controlInvocationCount = 0;
        this.lastObservation = {};
    }

    updateTimeOfDay(){
        this.timeOfDay = timeOfDayFromTime(this.time);
        if (this.env){
            this.env.time = this.time;
            this.env.timeOfDay = this.timeOfDay;
        }
    }
}

export class boatModel{
    constructor(boatC, simC = null){
        this.boatConfig = boatC;
        this.maxSpeed = boatC.maxSpeed;
        this.maxAcceleration = boatC.maxAcceleration;
        this.maxDeceleration = boatC.maxDeceleration;
        this.maxTurn = boatC.maxTurn;
        this.basePowerDraw = boatC.basePowerDraw;
        this.movementPowerFactor = boatC.movementPowerFactor;
        this.mass = boatC.mass;
        this.dimensions = boatC.dimensions;
        this.translationalDamping = boatC.translationalDamping;
        this.angularDamping = boatC.angularDamping;
        this.inertia = boatC.inertia;
        this.hydrodynamics = boatC.hydrodynamics;
        this.buoyancyStrength = boatC.buoyancyStrength;
        this.waterDragStrength = boatC.waterDragStrength;
        this.waterTorqueScale = boatC.waterTorqueScale;
        this.rollStability = boatC.rollStability;
        this.pitchStability = boatC.pitchStability;
        this.heaveDamping = boatC.heaveDamping;
        this.maxUpwardWaterAccel = boatC.maxUpwardWaterAccel;
        this.maxDownwardWaterAccel = boatC.maxDownwardWaterAccel;
        this.maxEnvAngularAcceleration = boatC.maxEnvAngularAcceleration;
        this.normalForceHorizontalScale = boatC.normalForceHorizontalScale;
        this.maxAngularVelocity = boatC.maxAngularVelocity;
        this.maxRollAngle = boatC.maxRollAngle;
        this.maxPitchAngle = boatC.maxPitchAngle;
        this.physicsMode = normalizePhysicsMode(simC?.physicsMode || PHYSICS_MODES.COUPLED6);
        this.waveCoupling = simC?.waveCoupling || "none";

        this.angularDamping = boatC.angularDamping;

        this.vehicleParameters = boatC.vehicleParameters || VehicleParameters.fromGeometry(
            Math.max(this.dimensions.z || 1, 0.001),
            Math.max(this.dimensions.x || 1, 0.001),
            Math.max(this.hydrodynamics.draft || this.dimensions.y * 0.25, 0.001),
            Math.max(this.mass || 1, 0.001),
            {
                id: boatC.vehicleId || "bcod_usv",
                height: Math.max(this.dimensions.y || 1, 0.001),
                maxAcceleration: this.maxAcceleration,
                maxThrust: boatC.maxThrust || Math.max(this.mass * this.maxAcceleration, 1),
                motorTimeConstant: boatC.motorTimeConstant || 0.35,
                Xu: boatC.hydrodynamics.linearDamping.z * this.mass,
                Yv: boatC.hydrodynamics.linearDamping.x * this.mass,
                Nr: boatC.hydrodynamics.angularDamping.y * this.inertia.y,
                Xuu: boatC.hydrodynamics.quadraticDamping.z * this.mass,
                Yvv: boatC.hydrodynamics.quadraticDamping.x * this.mass,
                Nrr: 0.5 * boatC.hydrodynamics.waterDensity * 0.45 * boatC.hydrodynamics.lateralArea * (this.dimensions.z ** 2) / 12
            }
        );
        this.actuatorModel = new ActuatorModel(this.vehicleParameters);
        this.maneuveringModel = boatC.maneuveringModelParameters ? new VehicleBMmgForceModel(boatC.maneuveringModelParameters) : null;
        this.hydrostaticsModel = new HydrostaticsAndWaves();
        this.dynamicsCore = new DynamicsCore(
            this.vehicleParameters,
            [
                this.actuatorModel,
                new AddedMassCoriolis(),
                new HydrodynamicDamping(),
                this.hydrostaticsModel
            ],
            "rk4"
        );
        this.coupledPlant = new CoupledSixPlant(
            this.vehicleParameters,
            [this.maneuveringModel || this.actuatorModel, new WindLoad()],
            "rk4"
        );
        this.lastEnvSample = null;
    }

    updatePosEnv(boatState, envSample, dt) {
        this.lastEnvSample = envSample;
        this.ensureCoreState(boatState);
        boatState.environmentAcceleration = new vec3(0, 0, 0);
        boatState.environmentAngularAcceleration = new vec3(0, 0, 0);
        boatState.waterVelocity = copyVec(envSample.waterV || new vec3(0, 0, 0));
        boatState.waterHeight = envSample.waterH;
        boatState.buoyancyAcceleration = new vec3(0, 0, 0);
        boatState.waterDragAcceleration = new vec3(0, 0, 0);
    }

    calculateHullWaterForces(boatState, samples) {
        if (samples.length === 0){
            return {
                linearAcceleration: new vec3(0, 0, 0),
                angularAcceleration: new vec3(0, 0, 0),
                buoyancyAcceleration: new vec3(0, 0, 0),
                dragAcceleration: new vec3(0, 0, 0),
                averageWaterVelocity: null
            };
        }

        const totalAcceleration = new vec3(0, 0, 0);
        const buoyancyAcceleration = new vec3(0, 0, 0);
        const dragAcceleration = new vec3(0, 0, 0);
        const averageWaterVelocity = new vec3(0, 0, 0);
        const torque = new vec3(0, 0, 0);
        const draft = Math.max(boatState.draft || this.hydrodynamics.draft, 0.001);

        samples.forEach((sample) => {
            averageWaterVelocity.add(sample.waterV);

            const depthRatio = this.clamp(sample.depth / draft, 0, this.hydrodynamics.maxSubmergenceRatio);
            const normal = sample.waterNormal || new vec3(0, 1, 0);
            const verticalSupport = this.clamp(
                this.hydrodynamics.gravity * depthRatio,
                0,
                this.maxUpwardWaterAccel
            ) / samples.length;
            const sampleBuoyancy = new vec3(
                normal.x * verticalSupport * this.normalForceHorizontalScale,
                verticalSupport,
                normal.z * verticalSupport * this.normalForceHorizontalScale
            );
            const sampleBoatVelocity = new vec3(
                boatState.velocity.x + cross(boatState.angularVel, sample.offset).x,
                boatState.velocity.y + cross(boatState.angularVel, sample.offset).y,
                boatState.velocity.z + cross(boatState.angularVel, sample.offset).z
            );
            const relativeWaterVelocity = new vec3(
                sample.waterV.x - sampleBoatVelocity.x,
                sample.waterV.y - sampleBoatVelocity.y,
                sample.waterV.z - sampleBoatVelocity.z
            );
            const sampleDrag = this.calculateBodyFrameDragAcceleration(
                relativeWaterVelocity,
                boatState.heading,
                samples.length,
                depthRatio
            );
            const sampleAcceleration = new vec3(
                sampleBuoyancy.x + sampleDrag.x,
                sampleBuoyancy.y + sampleDrag.y,
                sampleBuoyancy.z + sampleDrag.z
            );

            buoyancyAcceleration.add(sampleBuoyancy);
            dragAcceleration.add(sampleDrag);
            totalAcceleration.add(sampleAcceleration);
            const sampleTorque = cross(sample.offset, new vec3(
                sampleAcceleration.x * this.mass,
                sampleAcceleration.y * this.mass,
                sampleAcceleration.z * this.mass
            ));
            sampleTorque.mult(new vec3(
                this.waterTorqueScale,
                this.waterTorqueScale,
                this.waterTorqueScale
            ));
            torque.add(sampleTorque);
        });

        averageWaterVelocity.div(new vec3(samples.length, samples.length, samples.length));
        const angularAcceleration = this.clampVec(new vec3(
            torque.x / this.inertia.x,
            torque.y / this.inertia.y,
            torque.z / this.inertia.z
        ), new vec3(
            -this.maxEnvAngularAcceleration.x,
            -this.maxEnvAngularAcceleration.y,
            -this.maxEnvAngularAcceleration.z
        ), this.maxEnvAngularAcceleration);

        return {
            linearAcceleration: totalAcceleration,
            angularAcceleration,
            buoyancyAcceleration,
            dragAcceleration,
            averageWaterVelocity
        };
    }



    updatePosGuidance(boatState, guidance, dt, actuatorCommandOverride = null) {
        if (!guidance || dt <= 0) {
            return;
        }
        const coreState = this.ensureCoreState(boatState);
        const actuatorCommand = actuatorCommandOverride || (this.boatConfig.guidanceActuatorMapper
            ? this.boatConfig.guidanceActuatorMapper.map(guidance)
            : legacyGuidanceToActuatorCommand(guidance, this.vehicleParameters, this.boatConfig));
        const envSample = this.lastEnvSample || {waterV: new vec3(0, 0, 0), hullWaterSamples: []};
        if (this.physicsMode === PHYSICS_MODES.COUPLED6) {
            this.coupledPlant.step(coreState, envSample, actuatorCommand, dt, envSample.t || 0);
        }
        else {
            actuatorCommand.appliedWrench = this.actuatorModel.commandWrench(actuatorCommand, dt);
            this.dynamicsCore.step(coreState, envSample, actuatorCommand, dt, envSample.t || 0);
            this.applyPresentationSeakeeping(boatState, envSample, dt, this.hydrostaticsModel.lastFullWrench);
        }
        this.syncBoatFromCore(boatState, coreState, guidance, actuatorCommand);
    }

    ensureCoreState(boatState) {
        if (!boatState.rigidBody) {
            boatState.rigidBody = RigidBodyState.fromEuler(
                {N: boatState.pos.z, E: boatState.pos.x, D: -boatState.pos.y},
                boatState.orientation.z || 0,
                boatState.orientation.x || 0,
                boatState.heading || boatState.orientation.y || 0
            );
        }
        if (this.physicsMode === PHYSICS_MODES.COUPLED6) {
            return boatState.rigidBody;
        }
        boatState.rigidBody.position.N = boatState.pos.z;
        boatState.rigidBody.position.E = boatState.pos.x;
        boatState.rigidBody.position.D = -boatState.pos.y;
        boatState.rigidBody.quaternion = RigidBodyState.fromEuler(
            boatState.rigidBody.position,
            boatState.orientation.z || 0,
            boatState.orientation.x || 0,
            boatState.heading || boatState.orientation.y || 0
        ).quaternion;
        const bodyVelocity = worldToBodyVector(boatState.velocity || new vec3(0, 0, 0), boatState.heading || 0);
        boatState.rigidBody.velocity.u = bodyVelocity.z;
        boatState.rigidBody.velocity.v = bodyVelocity.x;
        boatState.rigidBody.angularRate.r = boatState.angularVel ? boatState.angularVel.y : 0;
        return boatState.rigidBody;
    }

    syncBoatFromCore(boatState, coreState, guidance, actuatorCommand) {
        const euler = coreState.eulerAngles;
        const yaw = euler.yaw;
        const rotation = rotationBodyToNed(euler);
        const toAppWorld = (body) => {
            const ned = rotation.map((row) => row.reduce((sum, value, idx) => sum + value * body[idx], 0));
            return new vec3(ned[1], -ned[2], ned[0]);
        };
        const worldVelocity = toAppWorld([coreState.velocity.u, coreState.velocity.v, coreState.velocity.w]);
        const worldAcceleration = toAppWorld([coreState.acceleration.uDot, coreState.acceleration.vDot, coreState.acceleration.wDot]);
        const forward = headingForwardVector(yaw);
        const applied = actuatorCommand.appliedWrench || this.maneuveringModel?.lastFullWrench || this.actuatorModel.lastFullWrench || [0, 0, 0, 0, 0, 0];
        const guidanceAccelMag = (applied[0] || 0) / Math.max(this.mass, 0.001);

        boatState.pos.x = coreState.position.E;
        boatState.pos.z = coreState.position.N;
        boatState.pos.y = -coreState.position.D;
        boatState.velocity = new vec3(worldVelocity.x, worldVelocity.y, worldVelocity.z);
        boatState.acceleration = new vec3(worldAcceleration.x, worldAcceleration.y, worldAcceleration.z);
        boatState.guidanceAcceleration = new vec3(
            forward.x * guidanceAccelMag,
            0,
            forward.z * guidanceAccelMag
        );
        boatState.waterDragAcceleration = new vec3(
            boatState.acceleration.x - boatState.guidanceAcceleration.x,
            0,
            boatState.acceleration.z - boatState.guidanceAcceleration.z
        );
        boatState.environmentAcceleration = copyVec(boatState.waterDragAcceleration);
        if (this.physicsMode === PHYSICS_MODES.COUPLED6) {
            boatState.orientation.x = euler.pitch;
            boatState.orientation.z = euler.roll;
        }
        boatState.orientation.y = this.normalizeAngle(yaw);
        boatState.heading = boatState.orientation.y;
        if (this.physicsMode === PHYSICS_MODES.COUPLED6) {
            boatState.angularVel.x = coreState.angularRate.q;
            boatState.angularVel.z = coreState.angularRate.p;
            boatState.angularAcceleration.x = coreState.angularAccel.qDot;
            boatState.angularAcceleration.z = coreState.angularAccel.pDot;
        }
        boatState.angularVel.y = coreState.angularRate.r;
        boatState.angularAcceleration.y = coreState.angularAccel.rDot;
        boatState.guidanceAngularAcceleration.y = (applied.length >= 6 ? applied[5] : applied[2] || 0) / Math.max(this.inertia.y, 0.001);
        boatState.environmentAngularAcceleration.y = boatState.angularAcceleration.y - boatState.guidanceAngularAcceleration.y;
        boatState.lastActuatorCommand = actuatorCommand;
        boatState.lastDynamicsWrench = this.physicsMode === PHYSICS_MODES.COUPLED6
            ? [...this.coupledPlant.lastWrench]
            : this.dynamicsCore.lastWrench;
        boatState.forceBreakdown = this.physicsMode === PHYSICS_MODES.COUPLED6
            ? structuredClone(this.coupledPlant.forceBreakdown)
            : {};
        boatState.physicsMode = this.physicsMode;
        boatState.hydrostaticWrench = this.physicsMode === PHYSICS_MODES.COUPLED6
            ? [...(this.coupledPlant.forceBreakdown.Hydrostatics6 || Array(6).fill(0))]
            : boatState.hydrostaticWrench;
    }

    savePropulsionState() {
        return this.maneuveringModel ? {kind:"vehicle-b-mmg",state:this.maneuveringModel.saveState()} : this.actuatorModel.saveState();
    }

    loadPropulsionState(checkpoint) {
        if (checkpoint?.kind === "vehicle-b-mmg" && this.maneuveringModel) return this.maneuveringModel.loadState(checkpoint.state);
        if (!checkpoint?.kind && !this.maneuveringModel) return this.actuatorModel.loadState(checkpoint);
        throw new Error("Checkpoint propulsion model does not match the configured vehicle path.");
    }

    applyPresentationSeakeeping(boatState, envSample, dt, hydrostaticWrench = null) {
        const samples = envSample.hullWaterSamples || [];
        let pitchTarget = 0;
        let rollTarget = 0;
        let pitchAccelSeed = 0;
        let rollAccelSeed = 0;
        if (samples.length) {
            let forwardSlope = 0;
            let sideSlope = 0;
            samples.forEach((sample) => {
                const normal = sample.waterNormal || new vec3(0, 1, 0);
                forwardSlope += -(normal.z || 0);
                sideSlope += -(normal.x || 0);
            });
            pitchTarget = forwardSlope / samples.length;
            rollTarget = sideSlope / samples.length;
        }
        const waveTilted = Math.abs(pitchTarget) > 1e-5 || Math.abs(rollTarget) > 1e-5;
        if (waveTilted && hydrostaticWrench && hydrostaticWrench.length >= 6) {
            pitchAccelSeed = this.clamp(
                hydrostaticWrench[4] / Math.max(this.inertia.x, 0.001),
                -this.maxEnvAngularAcceleration.x,
                this.maxEnvAngularAcceleration.x
            );
            rollAccelSeed = this.clamp(
                hydrostaticWrench[3] / Math.max(this.inertia.z, 0.001),
                -this.maxEnvAngularAcceleration.z,
                this.maxEnvAngularAcceleration.z
            );
        }
        const pitchStiffness = Math.max(this.pitchStability || 0, 0);
        const rollStiffness = Math.max(this.rollStability || 0, 0);
        const pitchDamping = 2 * Math.sqrt(pitchStiffness);
        const rollDamping = 2 * Math.sqrt(rollStiffness);
        const pitchAccel = pitchAccelSeed + (pitchTarget - boatState.orientation.x) * pitchStiffness - boatState.angularVel.x * pitchDamping;
        const rollAccel = rollAccelSeed + (rollTarget - boatState.orientation.z) * rollStiffness - boatState.angularVel.z * rollDamping;

        boatState.angularVel.x += pitchAccel * dt;
        boatState.angularVel.z += rollAccel * dt;
        boatState.orientation.x += boatState.angularVel.x * dt;
        boatState.orientation.z += boatState.angularVel.z * dt;
        boatState.restoringAngularAcceleration = new vec3(pitchAccel, 0, rollAccel);
        boatState.angularAcceleration.x = pitchAccel;
        boatState.angularAcceleration.z = rollAccel;
        boatState.hydrostaticWrench = hydrostaticWrench ? [...hydrostaticWrench] : null;
    }

    calculateRestoringAngularAcceleration(boatState) {
        const pitchDamping = 2 * Math.sqrt(this.pitchStability);
        const rollDamping = 2 * Math.sqrt(this.rollStability);
        return new vec3(
            -boatState.orientation.x * this.pitchStability - boatState.angularVel.x * pitchDamping,
            0,
            -boatState.orientation.z * this.rollStability - boatState.angularVel.z * rollDamping
        );
    }

    constrainAttitude(boatState) {
        boatState.orientation.x = this.clamp(boatState.orientation.x, -this.maxPitchAngle, this.maxPitchAngle);
        boatState.orientation.z = this.clamp(boatState.orientation.z, -this.maxRollAngle, this.maxRollAngle);
    }

    calculateBodyFrameDragAcceleration(relativeWaterVelocity, heading, sampleCount, submergence = 1) {
        const bodyVelocity = worldToBodyVector(relativeWaterVelocity, heading);
        const linear = this.hydrodynamics.linearDamping;
        const quadratic = this.hydrodynamics.quadraticDamping;
        const maxDrag = this.hydrodynamics.maxDragAcceleration;
        const immersion = this.clamp(submergence, 0, 1);

        const localAcceleration = new vec3(
            immersion * (bodyVelocity.x * linear.x + bodyVelocity.x * Math.abs(bodyVelocity.x) * quadratic.x) / sampleCount,
            immersion * (bodyVelocity.y * linear.y + bodyVelocity.y * Math.abs(bodyVelocity.y) * quadratic.y) / sampleCount,
            immersion * (bodyVelocity.z * linear.z + bodyVelocity.z * Math.abs(bodyVelocity.z) * quadratic.z) / sampleCount
        );

        return bodyToWorldVector(this.clampVec(
            localAcceleration,
            new vec3(-maxDrag.x / sampleCount, -maxDrag.y / sampleCount, -maxDrag.z / sampleCount),
            new vec3(maxDrag.x / sampleCount, maxDrag.y / sampleCount, maxDrag.z / sampleCount)
        ), heading);
    }

    applySmallAirDamping(boatState, dt) {
        const airDamping = 0.015;
        const dampingFactor = Math.exp(-airDamping * dt);
        boatState.velocity.x *= dampingFactor;
        boatState.velocity.z *= dampingFactor;
    }

    bodyForwardSpeed(worldVelocity, heading) {
        return worldToBodyVector(worldVelocity, heading).z;
    }

    clampVec(value, min, max) {
        return new vec3(
            this.clamp(value.x, min.x, max.x),
            this.clamp(value.y, min.y, max.y),
            this.clamp(value.z, min.z, max.z)
        );
    }

    clamp(value, min, max){
        return Math.min(Math.max(value, min), max);
    }

    normalizeAngle(angle){
        while (angle > Math.PI){
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI){
            angle += 2 * Math.PI;
        }
        return angle;
    }
}

export function copyVec(v) {
    return new vec3(v.x, v.y, v.z);
}

export function headingForwardVector(heading) {
    return new vec3(
        Math.sin(heading || 0),
        0,
        Math.cos(heading || 0)
    );
}

export function horizontalDistance(a, b) {
    return ((a.x - b.x) ** 2 + (a.z - b.z) ** 2) ** 0.5;
}

export function bodyToWorldVector(v, heading) {
    const cosH = Math.cos(heading || 0);
    const sinH = Math.sin(heading || 0);

    return new vec3(
        v.x * cosH + v.z * sinH,
        v.y,
        -v.x * sinH + v.z * cosH
    );
}

export function worldToBodyVector(v, heading) {
    const cosH = Math.cos(heading || 0);
    const sinH = Math.sin(heading || 0);

    return new vec3(
        v.x * cosH - v.z * sinH,
        v.y,
        v.x * sinH + v.z * cosH
    );
}

export function rotateBodyOffset(offset, orientation = new vec3(0, 0, 0)){
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
    return new vec3(
        rollX * cosY + pitchZ * sinY,
        pitchY,
        -rollX * sinY + pitchZ * cosY
    );
}

export function addOrientation(base = new vec3(0, 0, 0), offset = new vec3(0, 0, 0)){
    return new vec3(
        (base.x || 0) + (offset.x || 0),
        (base.y || 0) + (offset.y || 0),
        (base.z || 0) + (offset.z || 0)
    );
}

export function orientationForwardVector(orientation = new vec3(0, 0, 0)){
    return rotateBodyOffset(new vec3(0, 0, 1), orientation);
}

export function orientationRightVector(orientation = new vec3(0, 0, 0)){
    return rotateBodyOffset(new vec3(1, 0, 0), orientation);
}

export function orientationUpVector(orientation = new vec3(0, 0, 0)){
    return rotateBodyOffset(new vec3(0, 1, 0), orientation);
}

export function sensorWorldPose(sensor, boatState){
    const mountPosition = sensor.mountPosition || new vec3(0, 0, 0);
    const mountOrientation = sensor.mountOrientation || new vec3(0, 0, 0);
    const worldOffset = rotateBodyOffset(mountPosition, boatState.orientation);
    const orientation = addOrientation(boatState.orientation, mountOrientation);
    const position = new vec3(
        boatState.pos.x + worldOffset.x,
        boatState.pos.y + worldOffset.y,
        boatState.pos.z + worldOffset.z
    );

    return {
        position,
        orientation,
        forward: orientationForwardVector(orientation),
        right: orientationRightVector(orientation),
        up: orientationUpVector(orientation)
    };
}

export function copyBoatState(source) {
    const copy = new boatState(
        copyVec(source.pos),
        copyVec(source.orientation),
        copyVec(source.dimensions),
        source.powerDraw
    );

    copy.velocity = copyVec(source.velocity);
    copy.acceleration = copyVec(source.acceleration);
    copy.environmentAcceleration = copyVec(source.environmentAcceleration);
    copy.guidanceAcceleration = copyVec(source.guidanceAcceleration);
    copy.buoyancyAcceleration = copyVec(source.buoyancyAcceleration);
    copy.waterDragAcceleration = copyVec(source.waterDragAcceleration);
    copy.waterVelocity = copyVec(source.waterVelocity);
    copy.waterHeight = source.waterHeight;
    copy.heading = source.heading;
    copy.angularVel = copyVec(source.angularVel);
    copy.angularAcceleration = copyVec(source.angularAcceleration);
    copy.environmentAngularAcceleration = copyVec(source.environmentAngularAcceleration);
    copy.guidanceAngularAcceleration = copyVec(source.guidanceAngularAcceleration);
    copy.restoringAngularAcceleration = copyVec(source.restoringAngularAcceleration);
    copy.draft = source.draft;
    copy.dimensions.draft = source.draft;
    copy.hitBox = source.hitBox;
    copy.rigidBody = source.rigidBody ? source.rigidBody.clone() : null;
    copy.lastActuatorCommand = source.lastActuatorCommand ? {...source.lastActuatorCommand} : null;
    copy.lastDynamicsWrench = source.lastDynamicsWrench ? [...source.lastDynamicsWrench] : null;
    copy.hydrostaticWrench = source.hydrostaticWrench ? [...source.hydrostaticWrench] : null;
    copy.forceBreakdown = source.forceBreakdown ? structuredClone(source.forceBreakdown) : {};
    copy.physicsMode = source.physicsMode;
    return copy;
}

export function snapshotBoatState(boatState, t){
    return {
        t,
        pos: copyVec(boatState.pos),
        velocity: copyVec(boatState.velocity),
        acceleration: copyVec(boatState.acceleration),
        orientation: copyVec(boatState.orientation),
        angularVel: copyVec(boatState.angularVel),
        angularAcceleration: copyVec(boatState.angularAcceleration),
        heading: boatState.heading,
        powerDraw: boatState.powerDraw,
        rigidBody: boatState.rigidBody ? boatState.rigidBody.clone() : null,
        dynamicsWrench: boatState.lastDynamicsWrench ? [...boatState.lastDynamicsWrench] : null,
        hydrostaticWrench: boatState.hydrostaticWrench ? [...boatState.hydrostaticWrench] : null,
        physicsMode: boatState.physicsMode,
        quaternion: boatState.rigidBody ? {...boatState.rigidBody.quaternion} : null,
        bodyVelocity: boatState.rigidBody ? {...boatState.rigidBody.velocity} : null,
        bodyAngularRate: boatState.rigidBody ? {...boatState.rigidBody.angularRate} : null,
        bodyAcceleration: boatState.rigidBody ? {...boatState.rigidBody.acceleration} : null,
        bodyAngularAccel: boatState.rigidBody ? {...boatState.rigidBody.angularAccel} : null,
        forceBreakdown: boatState.forceBreakdown ? structuredClone(boatState.forceBreakdown) : {}
    };
}

export function snapshotMetricState(metricState, t){
    return {
        t,
        totalEnergy: metricState.totalEnergy,
        totalSensorCost: metricState.totalSensorCost,
        movementCost: metricState.movementCost,
        strategicPenalty: metricState.strategicPenalty,
        lastSensorCost: metricState.lastSensorCost,
        lastMovementCost: metricState.lastMovementCost,
        lastTotalCost: metricState.lastTotalCost,
        lastSpeed: metricState.lastSpeed
    };
}

export function cross(a, b) {
    return new vec3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

export class goalModel{
    constructor(goalC){
        this.locations = goalC.waypoints;
        this.tolerance = goalC.tolerance;
    }
    updateMissionProgress(goalState, boatState, t){
        if (goalState.completed || goalState.failed){
            return;
        }

        const nextWaypoint = goalState.waypoints[goalState.waypointIdx];
        if (!nextWaypoint){
            goalState.completed = true;
            return;
        }

        if (this.hasReachedWaypoint(boatState, nextWaypoint, goalState.tolerance)){
            goalState.waypointHistory.push(new waypointHitRecord(
                {idx: goalState.waypointIdx, pos: nextWaypoint},
                boatState,
                t
            ));
            goalState.waypointIdx++;
        }

        if (goalState.waypointIdx >= goalState.waypoints.length){
            goalState.completed = true;
        }
    }

    hasReachedWaypoint(boatState, waypoint, tolerance){
        if (horizontalDistance(boatState.pos, waypoint) < tolerance){
            return true;
        }

        const local = worldToBodyVector(new vec3(
            waypoint.x - boatState.pos.x,
            0,
            waypoint.z - boatState.pos.z
        ), boatState.heading);
        const halfBeam = Math.max((boatState.dimensions ? boatState.dimensions.x : 0) * 0.5, 0);
        const halfLength = Math.max((boatState.dimensions ? boatState.dimensions.z : 0) * 0.5, 0);
        const reachX = halfBeam + tolerance;
        const reachZ = halfLength + tolerance;

        if (reachX <= 0 || reachZ <= 0){
            return false;
        }

        return (local.x / reachX) ** 2 + (local.z / reachZ) ** 2 <= 1;
    }
}

export class guidanceObj{
    constructor(w,a,target=null,desiredHeading=0,desiredSpeed=0){
        this.w = w;
        this.a = a;
        this.target = target;
        this.desiredHeading = desiredHeading;
        this.desiredSpeed = desiredSpeed;
    }
}

export class controlWaypoint{
    constructor(pos, activeSensors){
        this.pos = pos;
        this.activeSensors = activeSensors;
    }
}

export class controlCommand{
    constructor(waypoints = [], activeSensors = []){
        this.waypoints = waypoints;
        this.activeSensors = activeSensors;
    }
}

export class controlModel{
    constructor(controlC){
        this.stepTime = 1/controlC.controlHz;
        this.strategy = controlC.strategy;
        this.timeout = controlC.timeout;
        this.mode = controlC.mode;
        this.guidanceMode = controlC.guidanceMode || "relative";
    }

    step(observation, state){
        if (this.strategy === "heuristic"){
            const currentIdx = state.goal ? state.goal.waypointIdx : 0;
            const waypoints = state.goal && state.goal.waypoints
                ? state.goal.waypoints.slice(currentIdx, currentIdx + 2)
                : [];
            const activeSensors = state.sensors && state.sensors.sensors
                ? state.sensors.sensors.map((sensor) => sensor.id || sensor.name)
                : [];
            return new controlCommand(
                waypoints,
                activeSensors
            );
        }

        if (this.strategy === "local"){
            const currentIdx = state.goal ? state.goal.waypointIdx : 0;
            const waypoints = state.goal && state.goal.waypoints
                ? state.goal.waypoints.slice(currentIdx, currentIdx + 1)
                : [];
            return new controlCommand(waypoints, []);
        }

        return new controlCommand([], []);
    }
}

export class sensorModel{
    constructor(sensorC){
        this.sensors = sensorC.sensors;
        this.sensorProvider = null;
    }

    setProvider(sensorProvider){
        this.sensorProvider = sensorProvider;
    }

    getObservations(state, lastObservation = {}, sensorProvider = null){
        const activeSensors = Array.isArray(state.activeSensors)
            ? state.activeSensors
            : Object.values(state.activeSensors || {});
        const outputs = {...lastObservation};
        const provider = sensorProvider || this.sensorProvider;
        if (provider && provider.beginFrame){
            provider.beginFrame(state);
        }

        this.sensors.forEach((sensor) => {
            if (!this.isActive(sensor, activeSensors)){
                return;
            }
            const outputKey = sensor.id || sensor.name;
            if (!this.shouldSample(sensor, state)){
                return;
            }

            const observation = this.observeSensor(sensor, state, provider);
            outputs[outputKey] = observation;
            outputs[sensor.name] = observation;
            state.sensors.sampleCounts[outputKey] = (state.sensors.sampleCounts[outputKey] || 0) + 1;
        });

        state.sensors.activeSet = activeSensors;
        state.sensors.lastOutputs = outputs;
        return outputs;
    }

    observeSensor(sensor, state, sensorProvider = null){
        if (sensorProvider && sensorProvider.canObserve && sensorProvider.canObserve(sensor)){
            const providerObservation = sensorProvider.observe(sensor, state);
            if (providerObservation){
                return providerObservation;
            }
        }

        // Deprecated compatibility observation. Production experiments resolve
        // the stable "gps" ID to the typed GPS plugin in LegacyProductionEngine.
        if (sensor.type === "gps"){
            return {
                t: state.time,
                type: sensor.type,
                sensorId: sensor.id,
                sensorName: sensor.name,
                pose: sensorWorldPose(sensor, state.boat),
                pos: copyVec(state.boat.pos),
                velocity: copyVec(state.boat.velocity)
            };
        }

        // Deprecated compatibility observation. Production experiments resolve
        // the stable "imu" ID to the typed IMU plugin in LegacyProductionEngine.
        if (sensor.type === "imu"){
            return {
                t: state.time,
                type: sensor.type,
                sensorId: sensor.id,
                sensorName: sensor.name,
                pose: sensorWorldPose(sensor, state.boat),
                acceleration: copyVec(state.boat.acceleration),
                angularVel: copyVec(state.boat.angularVel),
                angularAcceleration: copyVec(state.boat.angularAcceleration),
                orientation: copyVec(state.boat.orientation)
            };
        }

        if (sensor.type === "lidar"){
            return this.placeholderObservation(sensor, state, "Requires the Three.js simulation layer.");
        }

        if (sensor.type === "exo2"){
            return this.placeholderObservation(sensor, state, "EXO2 behavior is not defined yet.");
        }

        if (sensor.type === "dayCam" || sensor.type === "nightCam"){
            return this.placeholderObservation(sensor, state, "Requires the Three.js simulation layer.");
        }

        return this.placeholderObservation(sensor, state, "Sensor behavior is not defined yet.");
    }

    isActive(sensor, activeSensors){
        return activeSensors.includes(sensor.id) || activeSensors.includes(sensor.name);
    }

    shouldSample(sensor, state){
        const hz = Math.max(sensor.hz || 0, 0);
        if (hz === 0){
            return true;
        }
        if (!Number.isFinite(state.stepTime) || state.stepTime <= 0){
            throw new Error("Sensor sampling requires a positive simulation stepTime.");
        }
        const phase = this.sensorSamplePhase(sensor);
        const outputKey = sensor.id || sensor.name;
        const sampleIndex = state.sensors.sampleCounts[outputKey] || 0;
        const scheduledTime = phase + sampleIndex / hz;
        const elapsedTime = state.steps * state.stepTime;
        return elapsedTime + Math.max(state.stepTime, 1 / hz) * 1e-9 >= scheduledTime;
    }

    sensorSamplePhase(sensor){
        if (Number.isFinite(sensor.samplePhaseSec)){
            return Math.max(sensor.samplePhaseSec, 0);
        }
        if (sensor.type === "dayCam" || sensor.type === "nightCam"){
            return 0.18;
        }
        if (sensor.type === "lidar"){
            return 0.62;
        }
        return 0;
    }

    placeholderObservation(sensor, state, reason){
        return {
            t: state.time,
            type: sensor.type,
            sensorId: sensor.id,
            sensorName: sensor.name,
            pose: sensorWorldPose(sensor, state.boat),
            placeholder: true,
            reason
        };
    }

    getSensorFeeds(state){
        const activeSensors = Array.isArray(state.activeSensors)
            ? state.activeSensors
            : Object.values(state.activeSensors || {});
        return this.sensors.map((sensor) => {
            const observation = state.lastObservation[sensor.id || sensor.name] || state.lastObservation[sensor.name];
            return this.formatSensorFeed(
                sensor,
                observation,
                this.isActive(sensor, activeSensors),
                state
            );
        });
    }

    formatSensorFeed(sensor, observation, active, state){
        const base = {
            id: sensor.id || sensor.name,
            name: sensor.name,
            type: sensor.type,
            active,
            t: observation && Number.isFinite(observation.t) ? observation.t : state.time,
            hz: sensor.hz,
            config: this.sensorDisplayConfig(sensor),
            pose: observation && observation.pose ? observation.pose : sensorWorldPose(sensor, state.boat)
        };

        if (!active){
            return {
                ...base,
                status: "inactive",
                displayType: "placeholder",
                data: null,
                summary: "inactive"
            };
        }

        if (!observation){
            return {
                ...base,
                status: "stale",
                displayType: "placeholder",
                data: null,
                summary: "waiting for first sample"
            };
        }

        if (observation.placeholder){
            return {
                ...base,
                status: "placeholder",
                displayType: "placeholder",
                data: observation,
                summary: observation.reason
            };
        }

        if (sensor.type === "gps"){
            return {
                ...base,
                status: "live",
                displayType: "position",
                data: {
                    pos: observation.pos,
                    velocity: observation.velocity
                },
                summary: `${this.formatVec(observation.pos)}`
            };
        }

        if (sensor.type === "imu"){
            return {
                ...base,
                status: "live",
                displayType: "motion",
                data: {
                    acceleration: observation.acceleration,
                    angularVel: observation.angularVel,
                    angularAcceleration: observation.angularAcceleration,
                    orientation: observation.orientation
                },
                summary: `acc ${this.formatVec(observation.acceleration)}`
            };
        }

        if (sensor.type === "dayCam" || sensor.type === "nightCam"){
            return {
                ...base,
                status: "live",
                displayType: "image",
                data: {
                    width: observation.width,
                    height: observation.height,
                    fov: observation.fov,
                    imageDataUrl: observation.imageDataUrl,
                    rgbaPixels: observation.rgbaPixels
                },
                summary: `${observation.width}x${observation.height}`
            };
        }

        if (sensor.type === "lidar"){
            return {
                ...base,
                status: "live",
                displayType: "pointCloud",
                data: {
                    width: observation.width,
                    height: observation.height,
                    ranges: observation.ranges,
                    points: observation.points,
                    maxDistance: observation.maxDistance,
                    hitCount: observation.hitCount,
                    minRange: observation.minRange
                },
                summary: `${observation.hitCount || 0}/${(observation.width || 0) * (observation.height || 0)} hits`
            };
        }

        return {
            ...base,
            status: "live",
            displayType: "raw",
            data: observation,
            summary: "raw observation"
        };
    }

    sensorDisplayConfig(sensor){
        return {
            width: sensor.width,
            height: sensor.height,
            fov: sensor.fov,
            hRange: sensor.hRange,
            vRange: sensor.vRange,
            dRange: sensor.dRange,
            angularRes: sensor.angularRes,
            mountPosition: sensor.mountPosition,
            mountOrientation: sensor.mountOrientation
        };
    }

    formatVec(v){
        if (!v){
            return "n/a";
        }
        return `${v.x.toFixed(2)}, ${v.y.toFixed(2)}, ${v.z.toFixed(2)}`;
    }
}

export class metricModel{
    captureMetrics(state, dt, boatModel){
        const activeSensors = Array.isArray(state.activeSensors)
            ? state.activeSensors
            : Object.values(state.activeSensors || {});
        const activeSensorSet = new Set(activeSensors);
        const sensorCost = state.sensors.sensors.reduce((total, sensor) => {
            return activeSensorSet.has(sensor.id) || activeSensorSet.has(sensor.name)
                ? total + sensor.cost * dt
                : total;
        }, 0);
        const speed = state.boat.velocity.dist(new vec3(0, 0, 0));
        const movementCost = (
            boatModel.basePowerDraw +
            boatModel.movementPowerFactor * speed * speed
        ) * dt;

        state.metrics.lastSensorCost = sensorCost;
        state.metrics.lastMovementCost = movementCost;
        state.metrics.lastTotalCost = sensorCost + movementCost;
        state.metrics.lastSpeed = speed;
        state.metrics.totalSensorCost += sensorCost;
        state.metrics.movementCost += movementCost;
        state.metrics.totalEnergy = state.metrics.totalSensorCost + state.metrics.movementCost;
        return state.metrics;
    }
}

export class logger{
    constructor(){
        this.logs = {
            boatStates: [],
            boatBeliefs: [],
            metrics: [],
            sensorActivations: [],
            uncertaintySamples: [],
            predictedPaths: [],
            missionEvents: [],
        };
    }

    log(state){
        this.logs.boatStates.push(snapshotBoatState(state.boat, state.time));
        this.logs.boatBeliefs.push(snapshotBoatState(state.boatBelief, state.time));
        this.logMetrics(state);
        this.logs.sensorActivations.push({
            t: state.time,
            activeSensors: Array.isArray(state.activeSensors)
                ? [...state.activeSensors]
                : Object.values(state.activeSensors || {})
        });
    }

    logMetrics(state){
        this.logs.metrics.push(snapshotMetricState(state.metrics, state.time));
    }

    getLogs(){
        return this.logs;
    }
}

export class skipperModel{
    constructor(boatC, controlC){
        this.maxSpeed = boatC.maxSpeed;
        this.maxAcceleration = boatC.maxAcceleration;
        this.maxDeceleration = boatC.maxDeceleration;
        this.maxTurn = boatC.maxTurn;
        this.guidanceMode = controlC.guidanceMode || "relative";
        this.maxYawRate = boatC.maxAngularVelocity ? boatC.maxAngularVelocity.y : Math.max(this.maxTurn, 0.75);
    }

    getGuidance(command, state, boatOverride = null){
        const commandPoints = this.getCommandPoints(command, state);
        if (commandPoints.length === 0){
            return new guidanceObj(0, 0);
        }

        const boat = boatOverride || state.boatBelief || state.boat;
        const boatPos = boat.pos;
        const nextPoint = commandPoints[0];
        const upcomingPoint = commandPoints.length > 1 ? commandPoints[1] : nextPoint;

        const distToNext = this.horizontalDistance(boatPos, nextPoint);
        const tolerance = state.goal ? state.goal.tolerance : 1;
        const speed = this.horizontalSpeed(boat.velocity);
        const stoppingDistance = this.stoppingDistance(speed);
        const turnDistance = this.turnPlanningDistance(speed);
        const routeBlendDistance = Math.max(tolerance * 3, stoppingDistance + turnDistance);
        const hardWaypoint = state.goal && state.goal.waypoints && state.goal.waypoints.length > 0;
        const lookaheadAlpha = !hardWaypoint && commandPoints.length > 1
            ? this.clamp(1 - distToNext / routeBlendDistance, 0, 0.35)
            : 0;
        const target = this.lookaheadTarget(nextPoint, upcomingPoint, lookaheadAlpha);

        const dx = target.x - boatPos.x;
        const dz = target.z - boatPos.z;
        const desiredHeading = Math.atan2(dx, dz);
        const headingError = this.normalizeAngle(desiredHeading - boat.heading);
        const directToWaypoint = this.unitHorizontalVector(boatPos, nextPoint);
        const closingSpeed = this.horizontalDot(boat.velocity, directToWaypoint);
        const currentForwardSpeed = this.forwardSpeed(boat.velocity, boat.heading);
        const yawRate = boat.angularVel ? boat.angularVel.y : 0;
        const desiredSpeed = this.planDesiredSpeed(
            distToNext,
            tolerance,
            headingError,
            closingSpeed,
            currentForwardSpeed
        );
        const linearAcceleration = this.planLinearAcceleration(
            desiredSpeed,
            currentForwardSpeed,
            closingSpeed,
            speed,
            distToNext,
            tolerance
        );
        const rudderCommand = this.planRudderCommand(headingError, yawRate, speed);

        return new guidanceObj(
            rudderCommand,
            linearAcceleration,
            target,
            desiredHeading,
            desiredSpeed
        );
    }

    planDesiredSpeed(distToWaypoint, tolerance, headingError, closingSpeed, currentForwardSpeed){
        const arrivalDistance = Math.max(distToWaypoint - tolerance * 1.2, 0);
        const stopLimitedSpeed = 0.75 * Math.sqrt(2 * this.maxDeceleration * arrivalDistance);
        const headingDemand = Math.abs(headingError);
        const headingSpeedFactor = this.clamp(1 - headingDemand / (Math.PI * 0.85), 0.15, 1);
        const turnLimitedSpeed = this.maxSpeed * headingSpeedFactor;
        const desiredSpeed = Math.min(this.maxSpeed, stopLimitedSpeed, turnLimitedSpeed);

        if (distToWaypoint <= tolerance){
            return 0;
        }

        const minimumSteerageSpeed = headingDemand > Math.PI * 0.55
            ? this.maxSpeed * 0.12
            : this.maxSpeed * 0.22;
        const approachingTooFast = closingSpeed > stopLimitedSpeed;
        if (approachingTooFast || currentForwardSpeed < 0){
            return desiredSpeed;
        }
        return Math.max(desiredSpeed, Math.min(minimumSteerageSpeed, this.maxSpeed));
    }

    planLinearAcceleration(desiredSpeed, currentForwardSpeed, closingSpeed, speed, distToWaypoint, tolerance){
        const speedError = desiredSpeed - currentForwardSpeed;
        const speedToCheck = distToWaypoint < tolerance * 5
            ? Math.max(closingSpeed, speed * 0.8)
            : closingSpeed;
        const needsHardBrake = (
            distToWaypoint > tolerance &&
            speedToCheck > 0 &&
            this.stoppingDistance(speedToCheck) > Math.max(distToWaypoint - tolerance * 0.5, 0)
        );

        if (needsHardBrake){
            return -this.maxDeceleration;
        }

        return speedError >= 0
            ? this.clamp(speedError, 0, this.maxAcceleration)
            : this.clamp(speedError, -this.maxDeceleration, 0);
    }

    planRudderCommand(headingError, yawRate, speed){
        const headingDemand = Math.abs(headingError);
        const speedFactor = this.clamp(speed / Math.max(this.maxSpeed, 0.001), 0, 1);
        const plannedYawRate = this.clamp(
            headingError * (0.8 + 0.7 * speedFactor),
            -this.maxYawRate,
            this.maxYawRate
        );
        const yawRateError = plannedYawRate - yawRate;
        const yawAccelerationDemand = yawRateError * 1.4 + headingError * 0.25;
        const command = yawAccelerationDemand / Math.max(this.maxTurn, 0.001);

        if (headingDemand < 0.04 && Math.abs(yawRate) < 0.03){
            return 0;
        }
        return this.clamp(command, -1, 1);
    }

    getCommandPoints(command, state){
        if (command && Array.isArray(command.waypoints)){
            return command.waypoints.map((item) => item && item.pos ? item.pos : item);
        }

        if (Array.isArray(command) && command.length > 0){
            return command.map((item) => item.pos ? item.pos : item);
        }

        if (command && command.pos){
            return [command.pos];
        }

        if (state.goal && state.goal.waypoints){
            const currentIdx = state.goal.waypointIdx;
            return state.goal.waypoints.slice(currentIdx, currentIdx + 2);
        }

        return [];
    }

    lookaheadTarget(p1, p2, t){
        return new vec3(
            p1.x + (p2.x - p1.x) * t,
            p1.y + (p2.y - p1.y) * t,
            p1.z + (p2.z - p1.z) * t
        );
    }

    horizontalDistance(a, b){
        return ((a.x - b.x) ** 2 + (a.z - b.z) ** 2) ** 0.5;
    }

    forwardSpeed(velocity, heading){
        if (!velocity){
            return 0;
        }

        const forward = headingForwardVector(heading);
        return velocity.x * forward.x + velocity.z * forward.z;
    }

    horizontalSpeed(velocity){
        if (!velocity){
            return 0;
        }
        return Math.sqrt(velocity.x * velocity.x + velocity.z * velocity.z);
    }

    unitHorizontalVector(from, to){
        const dx = to.x - from.x;
        const dz = to.z - from.z;
        const dist = Math.sqrt(dx * dx + dz * dz);
        if (dist === 0){
            return new vec3(0, 0, 0);
        }
        return new vec3(dx / dist, 0, dz / dist);
    }

    horizontalDot(a, b){
        if (!a || !b){
            return 0;
        }
        return a.x * b.x + a.z * b.z;
    }

    stoppingDistance(speed){
        return (Math.max(speed, 0) ** 2) / (2 * Math.max(this.maxDeceleration, 0.001));
    }

    turnPlanningDistance(speed){
        const plannedYawRate = Math.max(this.maxYawRate * 0.65, 0.1);
        return speed / plannedYawRate;
    }

    normalizeAngle(angle){
        while (angle > Math.PI){
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI){
            angle += 2 * Math.PI;
        }
        return angle;
    }

    clamp(value, min, max){
        return Math.min(Math.max(value, min), max);
    }


}


export class simulator{
    constructor(scenarioC){
        this.seed = scenarioC.simConfig.seed;   
        this.stepTime = 1 / scenarioC.simConfig.simHz;
        this.durationSec = scenarioC.simConfig.durationSec;
        this.durationSteps = Math.ceil(this.durationSec / this.stepTime - 1e-12);
        this.allowGroundTruth = scenarioC.simConfig.allowGroundTruth;
        this.state = createInitialSimState(scenarioC);
        this.state.stepTime = this.stepTime;

        this.physicsMode = normalizePhysicsMode(scenarioC.simConfig.physicsMode || PHYSICS_MODES.COUPLED6);
        this.logEvery = scenarioC.simConfig.logEvery || 1;
        this.boatModel = new boatModel(scenarioC.boatConfig, scenarioC.simConfig);
        this.goalModel = new goalModel(scenarioC.goalConfig);
        this.controlModel = new controlModel(scenarioC.controlConfig);
        this.sensorModel = new sensorModel(scenarioC.sensorConfig);
        this.skipperModel = new skipperModel(scenarioC.boatConfig, scenarioC.controlConfig);
        this.envModel = new envModel(scenarioC.envConfig);
        this.metricModel = new metricModel();
        this.logger = new logger();
        this.logs = this.logger.getLogs();
    }

    setSensorProvider(sensorProvider){
        this.sensorProvider = sensorProvider;
        this.sensorModel.setProvider(sensorProvider);
    }
        

    step(options = {}){
        if (this.shouldStop()){
            this.state.isSimulating = false;
            this.state.stopReason = this.getStopReason();
            return;
        }

        const t = this.state.time;
        const localEnv = this.envModel.getLocalSample(
            this.state,
            this.boatModel.vehicleParameters,
            this.physicsMode
        );
        this.state.localEnv = localEnv;
        this.state.lastObservation = this.sensorModel.getObservations(
            this.state,
            this.state.lastObservation,
            options.sensorProvider || this.sensorProvider
        );

        if (options.controlCommand){
            this.state.lastCommand = options.controlCommand;
            this.state.lastCommandTime = t;
            this.state.activeSensors = this.getActiveSensors(this.state.lastCommand);
        }
        else if (this.controlIsDue()){
            this.state.lastCommand = this.controlModel.step(this.state.lastObservation, this.state);
            this.state.lastCommandTime = t;
            this.state.controlInvocationCount += 1;
            this.state.activeSensors = this.getActiveSensors(this.state.lastCommand);
        }
        this.updateBoatBelief();
        const guidance = this.skipperModel.getGuidance(this.state.lastCommand, this.state, this.state.boatBelief);

        this.boatModel.updatePosEnv(this.state.boat, localEnv, this.stepTime);
        this.boatModel.updatePosGuidance(this.state.boat, guidance, this.stepTime, options.actuatorCommand || null);
        this.updateBoatBelief();

        this.updateFailureState(localEnv);
        this.goalModel.updateMissionProgress(this.state.goal, this.state.boat, t);
        this.state.isSimulating = !this.shouldStop();
        this.metricModel.captureMetrics(this.state, this.stepTime, this.boatModel);

        if (this.state.steps % this.logEvery === 0) {
            this.logger.log(this.state);
        }

        this.state.tick += 1;
        this.state.steps += 1;
        this.state.time = this.state.startTime + this.state.steps * this.stepTime;
        this.state.updateTimeOfDay();
        this.state.isSimulating = !this.shouldStop();
        this.state.stopReason = this.state.isSimulating ? null : this.getStopReason();
    }

    updateBoatBelief(){
        if (this.skipperModel.guidanceMode === "absolute"){
            this.state.boatBelief = copyBoatState(this.state.boat);
            return this.state.boatBelief;
        }

        const belief = copyBoatState(this.state.boat);
        belief.pos.y = this.state.boat.waterHeight;
        belief.orientation.x = 0;
        belief.orientation.y = this.state.boat.heading;
        belief.orientation.z = 0;
        belief.heading = this.state.boat.heading;
        belief.angularVel.x = 0;
        belief.angularVel.z = 0;
        belief.angularAcceleration.x = 0;
        belief.angularAcceleration.z = 0;
        belief.environmentAngularAcceleration.x = 0;
        belief.environmentAngularAcceleration.z = 0;
        belief.restoringAngularAcceleration = new vec3(0, 0, 0);
        this.state.boatBelief = belief;
        return belief;
    }


    simStatus(){
        return this.state.isSimulating;
    }

    runSteps(stepCount){
        for (let i = 0; i < stepCount && this.simStatus(); i++){
            this.step();
        }
        return this.state;
    }

    runUntilDone(maxSteps = Infinity){
        let stepsRun = 0;
        while (this.simStatus() && stepsRun < maxSteps){
            this.step();
            stepsRun++;
        }
        return this.state;
    }

    getActiveSensors(command){
        if (!command){
            return [];
        }
        const activeSensors = command.activeSensors || [];
        return this.applyDeniedZones(activeSensors);
    }

    controlIsDue(){
        const scheduledTime = this.state.controlInvocationCount * this.controlModel.stepTime;
        const elapsedTime = this.state.steps * this.stepTime;
        const tolerance = Math.max(this.stepTime, this.controlModel.stepTime) * 1e-9;
        return elapsedTime + tolerance >= scheduledTime;
    }

    applyDeniedZones(activeSensors){
        const deniedZones = this.state.env && Array.isArray(this.state.env.deniedZones)
            ? this.state.env.deniedZones
            : [];
        if (!deniedZones.length || !activeSensors.length){
            return activeSensors;
        }
        const denied = new Set();
        deniedZones.forEach((zone) => {
            if (this.pointInZone(this.state.boat.pos, zone)){
                (zone.sensors || []).forEach((sensorId) => denied.add(sensorId));
            }
        });
        if (!denied.size){
            return activeSensors;
        }
        return activeSensors.filter((sensorId) => !denied.has(sensorId));
    }

    pointInZone(pos, zone){
        const points = zone.points || [];
        if (points.length < 3){
            return false;
        }
        let inside = false;
        for (let i = 0, j = points.length - 1; i < points.length; j = i, i++){
            const xi = points[i].x;
            const zi = points[i].z;
            const xj = points[j].x;
            const zj = points[j].z;
            const intersects = ((zi > pos.z) !== (zj > pos.z)) &&
                (pos.x < ((xj - xi) * (pos.z - zi)) / ((zj - zi) || 1e-9) + xi);
            if (intersects){
                inside = !inside;
            }
        }
        return inside;
    }

    getSensorFeeds(){
        return this.sensorModel.getSensorFeeds(this.state);
    }

    shouldStop(){
        if (this.state.goal.completed || this.state.goal.failed){
            return true;
        }
        return this.state.steps >= this.durationSteps;
    }

    getStopReason(){
        if (this.state.goal.completed){
            return "goal_completed";
        }
        if (this.state.goal.failed){
            return this.state.stopReason || "goal_failed";
        }
        if (this.state.steps >= this.durationSteps){
            return "duration_elapsed";
        }
        return null;
    }

    updateFailureState(localEnv){
        if (this.hasCollision(localEnv)){
            this.state.goal.failed = true;
            this.state.stopReason = "collision";
            return;
        }

        if (this.isOutOfBounds()){
            this.state.goal.failed = true;
            this.state.stopReason = "out_of_bounds";
        }
    }

    hasCollision(localEnv){
        return Object.values(localEnv.obstaclesHit || {}).some((obs) => obs.collision);
    }

    isOutOfBounds(){
        const bounds = this.state.env.bounds;
        const pos = this.state.boat.pos;

        return pos.x < 0 ||
            pos.z < 0 ||
            pos.x > bounds.width ||
            pos.z > bounds.height;
    }

}

export class boatConfig{
    constructor(
        maxSpeed,
        maxAcceleration,
        maxDeceleration,
        maxTurn,
        basePowerDraw,
        movementPowerFactor,
        startPos = new vec3(0, 0, 0),
        startOrientation = new vec3(0, 0, 0),
        dimensions = new vec3(2, 1, 4),
        mass = 100,
        transDrag = 1,
        angularDrag = 1,
        buoyancyStrength = 9.81,
        waterDragStrength = 0.5,
        waterTorqueScale = 0.15,
        rollStability = 2.4,
        pitchStability = 2.8,
        heaveDamping = 2.2,
        maxUpwardWaterAccel = 19.62,
        maxDownwardWaterAccel = 1.4,
        maxEnvAngularAcceleration = new vec3(0.28, 0.12, 0.32),
        normalForceHorizontalScale = 0.12,
        maxAngularVelocity = new vec3(0.45, Math.max(maxTurn, 0.75), 0.45),
        maxRollAngle = 0.38,
        maxPitchAngle = 0.34
    ){
        this.maxSpeed = maxSpeed;
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
        this.maxTurn = maxTurn;
        this.basePowerDraw = basePowerDraw;
        this.movementPowerFactor = movementPowerFactor;
        this.startPos = startPos;
        this.startOrientation = startOrientation;
        this.dimensions = dimensions;
        this.mass = mass;
        this.transDrag = transDrag;
        this.angularDrag = angularDrag;
        this.buoyancyStrength = buoyancyStrength;
        this.waterDragStrength = waterDragStrength;
        this.waterTorqueScale = waterTorqueScale;
        this.rollStability = rollStability;
        this.pitchStability = pitchStability;
        this.heaveDamping = heaveDamping;
        this.maxUpwardWaterAccel = maxUpwardWaterAccel;
        this.maxDownwardWaterAccel = maxDownwardWaterAccel;
        this.maxEnvAngularAcceleration = maxEnvAngularAcceleration;
        this.normalForceHorizontalScale = normalForceHorizontalScale;
        this.maxAngularVelocity = maxAngularVelocity;
        this.maxRollAngle = maxRollAngle;
        this.maxPitchAngle = maxPitchAngle;

        this.hydrodynamics = calculateRealisticBoatHydrodynamics(
            this.dimensions,
            this.mass,
            {
                transDrag: this.transDrag,
                angularDrag: this.angularDrag
            }
        );
        this.translationalDamping = this.hydrodynamics.linearDamping;
        this.angularDamping = this.hydrodynamics.angularDamping;
        this.inertia = this.hydrodynamics.inertia;
    }
    calculateBoatDampingRates(dimensions, mass, kTrans = 1, kRot = 1) {
        const hydrodynamics = calculateRealisticBoatHydrodynamics(dimensions, mass, {
            transDrag: kTrans,
            angularDrag: kRot
        });

        return {
            translationalDamping: hydrodynamics.linearDamping,
            angularDamping: hydrodynamics.angularDamping,
            inertia: hydrodynamics.inertia,
            hydrodynamics
        };
    }

}

export function calculateRealisticBoatHydrodynamics(dimensions, mass, options = {}) {
    const beam = Math.max(dimensions.x, 0.001);
    const height = Math.max(dimensions.y, 0.001);
    const length = Math.max(dimensions.z, 0.001);
    const waterDensity = options.waterDensity || 1025;
    const gravity = options.gravity || 9.81;
    const blockCoefficient = options.blockCoefficient || 0.55;
    const displacementVolume = mass / waterDensity;
    const rawDraft = displacementVolume / Math.max(length * beam * blockCoefficient, 0.001);
    const draft = clampNumber(rawDraft, height * 0.12, height * 0.85);
    const transDrag = options.transDrag || 1;
    const angularDrag = options.angularDrag || 1;

    const frontalArea = beam * draft;
    const lateralArea = length * draft;
    const waterplaneArea = length * beam;

    const surgeCd = options.surgeCd || 0.22;
    const swayCd = options.swayCd || 1.15;
    const heaveCd = options.heaveCd || 0.9;

    const linearDamping = new vec3(
        transDrag * (options.swayLinearDamping || 0.45),
        transDrag * (options.heaveLinearDamping || 1.35),
        transDrag * (options.surgeLinearDamping || 0.08)
    );

    const quadraticDamping = new vec3(
        0.5 * waterDensity * swayCd * lateralArea / mass,
        0.5 * waterDensity * heaveCd * waterplaneArea / mass,
        0.5 * waterDensity * surgeCd * frontalArea / mass
    );

    const inertia = new vec3(
        options.pitchInertiaFactor || 1.15,
        options.yawInertiaFactor || 1.25,
        options.rollInertiaFactor || 1.1
    );
    inertia.x *= mass * (height * height + length * length) / 12;
    inertia.y *= mass * (beam * beam + length * length) / 12;
    inertia.z *= mass * (beam * beam + height * height) / 12;

    const angularDamping = new vec3(
        angularDrag * (options.pitchAngularDamping || 4.0),
        angularDrag * (options.yawAngularDamping || 2.3),
        angularDrag * (options.rollAngularDamping || 4.3)
    );

    return {
        dimensions: new vec3(beam, height, length),
        mass,
        waterDensity,
        gravity,
        blockCoefficient,
        displacementVolume,
        draft,
        frontalArea,
        lateralArea,
        waterplaneArea,
        linearDamping,
        quadraticDamping,
        maxSubmergenceRatio: options.maxSubmergenceRatio || 2.4,
        maxDragAcceleration: new vec3(
            options.maxSwayDragAcceleration || 8,
            options.maxHeaveDragAcceleration || 14,
            options.maxSurgeDragAcceleration || 5
        ),
        maxHeaveSpeed: options.maxHeaveSpeed || 1.4,
        angularDamping,
        inertia
    };
}

export function clampNumber(value, min, max) {
    return Math.min(Math.max(value, min), max);
}

export class sensorConfig{
    constructor(sensorDict){
        if (sensorDict instanceof Map){
            this.sensors = Array.from(sensorDict.entries()).map(([id, sensor]) => {
                sensor.id = id;
                return sensor;
            });
        }
        else{
            this.sensors = Object.entries(sensorDict).map(([id, sensor]) => {
                sensor.id = id;
                return sensor;
            });
        }
    }
}

export class dayCamSensor{
    constructor(
        name,
        height,
        width,
        fov,
        hz,
        cost,
        mountPosition = new vec3(0, 0.65, 1.9),
        mountOrientation = new vec3(0, 0, 0)
    ){
        this.name = name;
        this.type = "dayCam";
        this.height = height;
        this.width = width;
        this.fov = fov;
        this.hz = hz;
        this.cost = cost;
        this.mountPosition = mountPosition;
        this.mountOrientation = mountOrientation;
    }
}

export class nightCamSensor{
    constructor(
        name,
        height,
        width,
        fov,
        hz,
        cost,
        mountPosition = new vec3(0, 0.65, 1.9),
        mountOrientation = new vec3(0, 0, 0)
    ){
        this.name = name;
        this.type = "nightCam";
        this.height = height;
        this.width = width;
        this.fov = fov;
        this.hz = hz;
        this.cost = cost;
        this.mountPosition = mountPosition;
        this.mountOrientation = mountOrientation;
    }
}

export class imuSensor{
    constructor(name, hz, cost, mountPosition = new vec3(0, 0.25, 0), mountOrientation = new vec3(0, 0, 0)){
        this.name=name;
        this.type = "imu";
        this.hz = hz;
        this.cost = cost;
        this.mountPosition = mountPosition;
        this.mountOrientation = mountOrientation;
    }
}

export class gpsSensor{
    constructor(name, hz, cost, mountPosition = new vec3(0, 0.8, -0.5), mountOrientation = new vec3(0, 0, 0)){
        this.name=name;
        this.type = "gps";
        this.hz = hz;
        this.cost = cost;
        this.mountPosition = mountPosition;
        this.mountOrientation = mountOrientation;
    }
}

export class exo2Sensor{
    constructor(name, hz, cost, mountPosition = new vec3(0, 0, 0), mountOrientation = new vec3(0, 0, 0)){
        this.name=name;
        this.type = "exo2";
        this.hz = hz;
        this.cost = cost;
        this.mountPosition = mountPosition;
        this.mountOrientation = mountOrientation;
    }
}

export class lidarSensor{
    constructor(
        name,
        horizontalRange,
        verticalRange,
        distance,
        angularRes,
        hz,
        cost,
        mountPosition = new vec3(0, 0.9, 0.4),
        mountOrientation = new vec3(0, 0, 0),
        includeWater = false
    ){
        this.name = name;
        this.type = "lidar";
        this.hRange = horizontalRange;
        this.vRange = verticalRange;
        this.dRange = distance;
        this.angularRes = angularRes;
        this.hz = hz;
        this.cost = cost;
        this.mountPosition = mountPosition;
        this.mountOrientation = mountOrientation;
        this.includeWater = includeWater;
    }
}

export class envConfig{
    constructor(width, height, obstacles, deniedZones, favoredZones, waterFieldConfig, visibility, timeOfDay, wind = null){
        this.bounds = {"width":width,"height":height};
        this.obstacles = obstacles;
        this.deniedZones = deniedZones;
        this.favoredZones = favoredZones;
        this.waterFieldConfig = waterFieldConfig;
        this.visibility = visibility;
        this.timeOfDay = timeOfDay;
        this.wind = wind;
    }
}

export class envSample {
    constructor(t, p, v, a, waterheight, vis, obstaclesHit, timeOfDay, hullWaterSamples = [], waterSample = null, submergedState = null) {
        this.t = t;
        this.p = p;
        this.waterA = a;
        this.waterV = v;
        this.waterH = waterheight;
        this.waterSample = waterSample;
        this.visibility = vis;
        this.obstaclesHit = obstaclesHit;
        this.timeOfDay = timeOfDay;
        this.hullWaterSamples = hullWaterSamples;
        this.submergedState = submergedState;
    }
}

export class envModel{
    constructor(envC){
        this.width = envC.bounds.width;
        this.height = envC.bounds.height
        this.obstacles = envC.obstacles;
        this.favoredZones = envC.favoredZones;
        this.deniedZones = envC.deniedZones;
        this.waterField = new waterFieldModel(envC.waterFieldConfig);
        
        this.visibility = envC.visibility;
        this.timeOfDay = envC.timeOfDay;
        this.wind = envC.wind || null;
    }

    incrementTime(time){
        if (time%(24*3600) > 21*3600 || time%(24*3600) < 3*3600){
            timeOfDay = "night";
        }
        if (time%(24*3600) > 3*3600 && time%(24*3600) < 9*3600){
            timeOfDay = "dawn";
        }
        if (time%(24*3600) > 9*3600 && time%(24*3600) < 15*3600){
            timeOfDay = "day";
        }
        if (time%(24*3600) > 15*3600 && time%(24*3600) < 21*3600){
            timeOfDay = "dusk";
        }
    }

    getLocalSample(state, vehicleParameters = null, physicsMode = PHYSICS_MODES.PLANAR3){
        const obstaclesHit = {};

        this.obstacles.forEach((obs, obsIdx) => {
            if (state.boat.pos.dist(obs.pos) < obs.r + (state.boat.hitBox || 0)){
                obstaclesHit[obsIdx] = obs;
            }
        });

        const hullOffsets = this.buildHullSampleOffsets(state.boat.dimensions);

        const hullWaterSamples = hullOffsets.map((localOffset) => {
            const offset = rotateBodyOffset(localOffset, state.boat.orientation);
            const samplePos = new vec3(
                state.boat.pos.x + offset.x,
                state.boat.pos.y + offset.y,
                state.boat.pos.z + offset.z
            );
            const waterSample = this.waterField.sampleAt(samplePos, state.time);

            return {
                offset,
                localOffset,
                samplePos,
                waterSample,
                waterH: waterSample.surfaceHeight,
                waterV: waterSample.velocity,
                waterA: waterSample.acceleration,
                waterNormal: waterSample.normal,
                depth: waterSample.depth,
                submerged: waterSample.submerged
            };
        });
        const centerWaterSample = this.waterField.sampleAt(state.boat.pos, state.time);
        const submergedState = physicsMode === PHYSICS_MODES.PLANAR3 && vehicleParameters?.hullPrimitives?.length
            ? computeSubmergedState(
                vehicleParameters.hullPrimitives,
                {
                    position: state.boat.pos,
                    orientation: state.boat.orientation
                },
                this.waterField,
                state.time
            )
            : null;

        const sample = new envSample(
            state.time,
            state.boat.pos,
            centerWaterSample.velocity,
            centerWaterSample.acceleration,
            centerWaterSample.surfaceHeight,
            this.visibility,
            obstaclesHit,
            state.timeOfDay,
            hullWaterSamples,
            centerWaterSample,
            submergedState
        );
        sample.wind = this.wind;
        return sample;
    }

    buildHullSampleOffsets(dimensions){
        const offsets = [];
        const beamStations = [-0.5, 0, 0.5];
        const lengthStations = [-0.5, -0.25, 0, 0.25, 0.5];
        const draft = Math.max(dimensions.draft || dimensions.y * 0.25, 0.001);

        lengthStations.forEach((zStation) => {
            beamStations.forEach((xStation) => {
                offsets.push(new vec3(
                    xStation * dimensions.x,
                    -draft,
                    zStation * dimensions.z
                ));
            });
        });

        return offsets;
    }

}

export class visibility{
    constructor(rain, fog){
        this.rain = rain;
        this.fog = fog;
        this.total = rain*fog;
    }
}

export class waterFieldConfig{
    constructor(waves, current, mode = "legacy", gravity = 9.81, depth = Infinity){
        this.waves = waves;
        this.current = current;
        this.mode = mode;
        this.gravity = gravity;
        this.depth = depth;
    }
}

export class waveConfig{
    constructor(heading, peakHeight, wavelength, speed, steepness, mode = "legacy", phase = 0){
        this.heading = heading;
        this.peakHeight = peakHeight;
        this.wavelength = wavelength;
        this.speed = speed;
        this.steepness = steepness;
        this.mode = mode;
        this.phase = phase;
    }
}

export class goalConfig{
    constructor(waypoints, tolerance){
        this.waypoints = waypoints;
        this.tolerance = tolerance;

    }
}

export class controlConfig{
    constructor(mode, controlHz, strategy, timeout, guidanceMode = "relative"){
        this.mode = mode;
        this.controlHz = controlHz;
        this.strategy = strategy;
        this.timeout = timeout;
        this.guidanceMode = guidanceMode;
    }
}

export function createInitialSimState(scenarioC){
    const startTime = startTimeFromTimeOfDay(scenarioC.envConfig.timeOfDay);
    const runtimeWaterField = new waterFieldModel(scenarioC.envConfig.waterFieldConfig);

    const initialEnvState = new envState(
        scenarioC.envConfig.bounds,
        scenarioC.envConfig.obstacles,
        scenarioC.envConfig.deniedZones,
        scenarioC.envConfig.favoredZones,
        scenarioC.envConfig.waterFieldConfig,
        scenarioC.envConfig.visibility,
        startTime,
        scenarioC.envConfig.timeOfDay
    );

    const startPos = new vec3(
        scenarioC.boatConfig.startPos.x,
        scenarioC.boatConfig.startPos.y,
        scenarioC.boatConfig.startPos.z
    );
    startPos.y = runtimeWaterField.sampleAt(startPos, startTime).surfaceHeight;

    let initialHeading = scenarioC.boatConfig.startOrientation.y;
    if (scenarioC.goalConfig.waypoints.length > 0){
        const firstWaypoint = scenarioC.goalConfig.waypoints[0];
        initialHeading = Math.atan2(firstWaypoint.x - startPos.x, firstWaypoint.z - startPos.z);
    }

    const initialBoatState = new boatState(
        startPos,
        new vec3(
            scenarioC.boatConfig.startOrientation.x,
            initialHeading,
            scenarioC.boatConfig.startOrientation.z
        ),
        scenarioC.boatConfig.dimensions,
        scenarioC.boatConfig.basePowerDraw
    );
    initialBoatState.draft = scenarioC.boatConfig.hydrodynamics.draft;
    initialBoatState.dimensions.draft = scenarioC.boatConfig.hydrodynamics.draft;
    initialBoatState.rigidBody = RigidBodyState.fromEuler(
        {N: initialBoatState.pos.z, E: initialBoatState.pos.x, D: -initialBoatState.pos.y},
        initialBoatState.orientation.z || 0,
        initialBoatState.orientation.x || 0,
        initialBoatState.heading || 0
    );
    const initialBoatBelief = copyBoatState(initialBoatState);

    const initialGoalState = new goalState(
        scenarioC.goalConfig.waypoints,
        scenarioC.goalConfig.tolerance
    );

    const initialSensorsState = new sensorsState(scenarioC.sensorConfig.sensors);
    const initialControlState = new controlState(scenarioC.controlConfig);
    const initialMetricState = new metricState();

    const initialState = new simState(
        startTime,
        initialBoatState,
        initialBoatBelief,
        initialGoalState,
        initialSensorsState,
        initialEnvState,
        initialControlState,
        initialMetricState
    );
    initialState.physicsMode = normalizePhysicsMode(scenarioC.simConfig.physicsMode || PHYSICS_MODES.COUPLED6);
    initialState.waveCoupling = scenarioC.simConfig.waveCoupling || "none";

    return initialState;
}

export class boatState{
    constructor(pos, orientation, dimensions, powerDraw){
        this.pos = pos; //vec3
        this.velocity = new vec3(0,0,0); //vec3
        this.acceleration = new vec3(0, 0, 0);
        this.environmentAcceleration = new vec3(0, 0, 0);
        this.guidanceAcceleration = new vec3(0, 0, 0);
        this.buoyancyAcceleration = new vec3(0, 0, 0);
        this.waterDragAcceleration = new vec3(0, 0, 0);
        this.waterVelocity = new vec3(0, 0, 0);
        this.waterHeight = pos.y;
        this.orientation = orientation;
        this.heading = orientation.y;
        this.angularVel = new vec3(0, 0, 0);
        this.angularAcceleration = new vec3(0, 0, 0);
        this.environmentAngularAcceleration = new vec3(0, 0, 0);
        this.guidanceAngularAcceleration = new vec3(0, 0, 0);
        this.restoringAngularAcceleration = new vec3(0, 0, 0);
        this.powerDraw = powerDraw;
        this.dimensions = dimensions;
        this.draft = dimensions.draft || dimensions.y * 0.25;
        this.hitBox = Math.max(dimensions.x, dimensions.z) / 2;
        this.rigidBody = null;
        this.lastActuatorCommand = null;
        this.lastDynamicsWrench = null;
        this.hydrostaticWrench = null;
        this.forceBreakdown = {};
        this.physicsMode = null;
    }
}

export class goalState{
    constructor(waypoints, tolerance){
        this.waypoints = waypoints;
        this.tolerance = tolerance;
        this.waypointIdx = 0;
        this.completed = false;
        this.failed = false;
        this.waypointHistory = []; //WaypointHitRecord[]
    }
}

export class waypointHitRecord{
    constructor(waypoint, boat, time){
        this.waypointIdx = waypoint.idx;
        this.hitTime = time;
        this.hitDist = horizontalDistance(boat.pos, waypoint.pos);
    }
}

export class sensorsState{
    constructor(sensors){
        this.sensors = sensors;
        this.activeSet = [];
        this.lastOutputs = {};
        this.sampleCounts = {};
        this.latencyQueue = {}; //Not going to be implemented yet
    }
}

export class envState{
    constructor(bounds, obstacles, deniedZones, favoredZones, waterFieldConfig, visibility, time, timeOfDay){
        this.bounds = bounds;
        this.obstacles = obstacles;
        this.deniedZones = deniedZones;
        this.favoredZones = favoredZones;
        this.waterFieldConfig = waterFieldConfig;
        this.visibility = visibility;
        this.time = time;
        this.timeOfDay = timeOfDay;
    }
}

export class controlState{
    constructor(config){
        this.mode = config.mode;
        this.strategy = config.strategy;
        this.timeout = config.timeout;
        this.guidanceMode = config.guidanceMode || "relative";
        this.lastTrajectory = {};
        this.pendingRequest = false;
        this.lastUpdateTime = 0;
        this.commandHistory = []; //controlCommand[]
    }
}

export class metricState{
    constructor(){
        this.totalEnergy = 0; //sensor+movement
        this.totalSensorCost = 0;
        this.movementCost = 0;
        this.strategicPenalty = 0;
        this.lastSensorCost = 0;
        this.lastMovementCost = 0;
        this.lastTotalCost = 0;
        this.lastSpeed = 0;
    }
}

export class vec3 {
    constructor(x,y,z){
        this.x = x;
        this.y = y;
        this.z = z;
    }

    dist(v){
        return ((this.x-v.x)**2 + (this.y-v.y)**2 + (this.z-v.z)**2)**0.5;
    }

    add(v){
        this.x += v.x;
        this.y += v.y;
        this.z += v.z;
        return this;
    }
    sub(v){
        this.x -= v.x;
        this.y -= v.y;
        this.z -= v.z;
        return this;
    }
    mult(v){
        this.x *= v.x;
        this.y *= v.y;
        this.z *= v.z;
        return this;
    }
    div(v){
        this.x /= v.x;
        this.y /= v.y;
        this.z /= v.z;
        return this;
    }
}

export class Obstacle{
    constructor(pos, r, collision){
        this.pos = pos;
        this.r = r;
        this.collision = collision;
    }

}

export class Zone{
    constructor(points, type, sensors){
        this.points = points;
        this.type = type
        this.sensors = sensors;
    } 
}

export class Wave{
    constructor(heading, peakHeight, wavelength, speed, steepness, options = {}){
        this.heading = heading;
        this.peakHeight = peakHeight;
        this.wavelength = wavelength;
        this.speed = speed;
        this.steepness = steepness;
        this.mode = options.mode || "legacy";
        this.phase = options.phase || 0;
        this.gravity = options.gravity || 9.81;
        this.depth = options.depth ?? Infinity;

        this.amplitude = peakHeight / 2;
        this.headingRad = heading * Math.PI / 180;
        this.direction = {
            x: Math.cos(this.headingRad),
            z: Math.sin(this.headingRad),
        };

        this.k = 2 * Math.PI / wavelength;
        this.omega = this.mode === "parityLinear"
            ? this.dispersionOmega()
            : this.k * speed;
        this.period = this.omega === 0 ? Infinity : 2 * Math.PI / this.omega;
        this.frequency = this.period === Infinity ? 0 : 1 / this.period;
        this.q = steepness;
    }

    dispersionOmega(){
        if (!Number.isFinite(this.depth)){
            return Math.sqrt(this.gravity * this.k);
        }
        return Math.sqrt(this.gravity * this.k * Math.tanh(this.k * Math.max(this.depth, 0)));
    }

    phaseAt(x, z, time){
        return this.k * (this.direction.x * x + this.direction.z * z) - this.omega * time + this.phase;
    }

    heightAt(x, z, time){
        const phase = this.phaseAt(x, z, time);
        if (this.mode === "parityLinear"){
            return this.amplitude * Math.cos(phase);
        }
        return this.amplitude * Math.sin(phase);
    }

    velocityAt(x, z, time){
        const phase = this.phaseAt(x, z, time);
        if (this.mode === "parityLinear"){
            const horizontalScale = this.amplitude * this.omega * Math.cos(phase);
            return new vec3(
                this.direction.x * horizontalScale,
                this.amplitude * this.omega * Math.sin(phase),
                this.direction.z * horizontalScale
            );
        }
        const horizontalScale = this.q * this.amplitude * this.omega * Math.sin(phase);

        return new vec3(
            this.direction.x * horizontalScale,
            -this.amplitude * this.omega * Math.cos(phase),
            this.direction.z * horizontalScale
        );
    }

    accelerationAt(x, z, time){
        const phase = this.phaseAt(x, z, time);
        if (this.mode === "parityLinear"){
            const horizontalScale = this.amplitude * this.omega * this.omega * Math.sin(phase);
            return new vec3(
                this.direction.x * horizontalScale,
                -this.amplitude * this.omega * this.omega * Math.cos(phase),
                this.direction.z * horizontalScale
            );
        }
        const horizontalScale = -this.q * this.amplitude * this.omega * this.omega * Math.cos(phase);

        return new vec3(
            this.direction.x * horizontalScale,
            -this.amplitude * this.omega * this.omega * Math.sin(phase),
            this.direction.z * horizontalScale
        );
    }

    gradientAt(x, z, time){
        const phase = this.phaseAt(x, z, time);
        if (this.mode === "parityLinear"){
            const slope = -this.amplitude * this.k * Math.sin(phase);
            return {
                dx: this.direction.x * slope,
                dz: this.direction.z * slope
            };
        }
        const slope = this.amplitude * this.k * Math.cos(phase);

        return {
            dx: this.direction.x * slope,
            dz: this.direction.z * slope
        };
    }
}

export class waterSample{
    constructor(pos, surfaceHeight, velocity, acceleration, normal){
        this.pos = pos;
        this.surfaceHeight = surfaceHeight;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.normal = normal;
        this.depth = surfaceHeight - pos.y;
        this.submerged = this.depth > 0;
    }
}

export class waterFieldModel{
    constructor(waterFieldC){
        this.mode = waterFieldC.mode || "legacy";
        this.gravity = waterFieldC.gravity || 9.81;
        this.depth = waterFieldC.depth ?? Infinity;
        this.waves = waterFieldC.waves.map((wave) => {
            if (wave instanceof Wave){
                return wave;
            }
            return new Wave(
                wave.heading,
                wave.peakHeight ?? wave.height,
                wave.wavelength,
                wave.speed,
                wave.steepness,
                {
                    mode: wave.mode || this.mode,
                    phase: wave.phase || 0,
                    gravity: this.gravity,
                    depth: this.depth
                }
            );
        });
        this.current = waterFieldC.current;
    }
    
    heightAt(x,z,time){
        let height = 0;
        this.waves.forEach((item) => {
            height += item.heightAt(x,z,time);
        });
        return height;
    }

    sampleAt(pos, time){
        const surfaceHeight = this.heightAt(pos.x, pos.z, time);

        return new waterSample(
            copyVec(pos),
            surfaceHeight,
            this.velocityAt(pos.x, pos.z, time),
            this.accelerationAt(pos.x, pos.z, time),
            this.normalAt(pos.x, pos.z, time)
        );
    }

    normalAt(x,z,time){
        let dhdx = 0;
        let dhdz = 0;

        this.waves.forEach((item) => {
            const gradient = item.gradientAt(x, z, time);
            dhdx += gradient.dx;
            dhdz += gradient.dz;
        });

        return this.normalizeVec(new vec3(-dhdx, 1, -dhdz));
    }

    velocityAt(x,z,time){
        const v = new vec3(0,0,0);
        this.waves.forEach((item) => {
            v.add(item.velocityAt(x,z,time));
        });
        v.add(this.current);
        return v;
    }

    accelerationAt(x,z,time){
        const v = new vec3(0,0,0);
        this.waves.forEach((item) => {
            v.add(item.accelerationAt(x,z,time));
        });
        return v;
    }

    normalizeVec(v){
        const mag = (v.x * v.x + v.y * v.y + v.z * v.z) ** 0.5;
        if (mag === 0){
            return new vec3(0, 1, 0);
        }
        return new vec3(v.x / mag, v.y / mag, v.z / mag);
    }
}
