import assert from "node:assert/strict";
import {resolveExperiment} from "../../../packages/experiment-schema/src/index.ts";
import {LegacyProductionEngine} from "./legacy-production-engine.ts";

const expectedInertia=[5.375132500000001,13.347308666666668,17.891219833333338];
const config=resolveExperiment({schema_version:1,experiment:{name:"surveyor-common-task-resolution",seed:73,timestep_s:.05,duration_s:2},backend:{type:"node"},vehicle:{preset:"searobotics-surveyor-m1.8",plant:"planar3"},initial_state:{position_ned_m:[10000,10000,0],attitude_rad:[0,0,.1]},mission:{type:"rl-common-waypoint-v1",waypoints:[{north_m:10020,east_m:10000}]},sensors:[]} as any),engine=new LegacyProductionEngine();
engine.reset(config);const result=engine.step({actuators:{portCommand:.3,starboardCommand:.4}}),path:any=result.info.vehicle_path;
assert.equal(path.vehicle_id,"searobotics-surveyor-m1.8");assert.equal(path.model_id,"searobotics-surveyor-m1.8-common-task-v1");assert.equal(path.mass_kg,52.3);assert.deepEqual(path.geometry_m,{length:1.83,beam:.91,draft:.17});assert.deepEqual(path.inertia_diagonal_kg_m2,expectedInertia);assert.deepEqual(path.thruster_positions_body_m,[[-.65,-.33,0],[-.65,.33,0]]);assert.equal(path.actuator_lag.time_constant_s,.35);assert.match(path.actuator_lag.provenance,/borrowed generic Vehicle A fallback/);assert.ok(result.info.vehicle_diagnostics);
const projected=engine.step({actuators:{desiredWrench:[70,0,0,0,0,-78]}}),commands:any=projected.info.vehicle_diagnostics.applied_command;
assert.ok(Math.abs(commands.port.thrust)<=70&&Math.abs(commands.starboard.thrust)<=70);assert.ok(Math.abs(commands.port.thrust+commands.starboard.thrust)<=70,"yaw-first projection must reserve force before surge");
assert.ok(Math.abs(.33*commands.port.thrust-.33*commands.starboard.thrust)<=46.2+1e-9);
const truth:any=engine.getGroundTruth();assert.ok(truth.position_ned_m.every(Number.isFinite));assert.ok(truth.velocity_body_mps.every(Number.isFinite));engine.dispose();console.log("Surveyor common-task Node resolution passed.");
