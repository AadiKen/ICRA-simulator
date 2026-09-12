#!/usr/bin/env node
import {McpServer} from "@modelcontextprotocol/sdk/server/mcp.js";
import {StdioServerTransport} from "@modelcontextprotocol/sdk/server/stdio.js";
import {z} from "zod";
import {constructScenario,getRunMetrics,listSensorPlugins,listVehiclePresets,runScenario,validateScenario} from "./tools.ts";

const server=new McpServer({name:"bcod-sim",version:"0.1.0"});
const waypoint=z.object({north_m:z.number(),east_m:z.number()});
const geographicPoint=z.object({lat:z.number(),lon:z.number(),heading_deg:z.number().optional()});
const mission=z.union([z.object({type:z.literal("waypoint"),waypoints:z.array(waypoint).min(1)}).passthrough(),z.object({type:z.literal("surveyor-waypoint"),origin:geographicPoint,erp:geographicPoint,waypoints:z.array(geographicPoint).min(1),max_thrust_command:z.number().int().optional()}).passthrough()]);
const vector3=z.tuple([z.number(),z.number(),z.number()]),credentials=z.union([z.object({source:z.literal("environment"),env_var:z.string()}),z.object({source:z.literal("file"),path:z.string()})]);
const environment=z.object({current_mps:vector3.optional(),wind_mps:vector3.optional(),regular_wave:z.object({amplitude_m:z.number(),period_s:z.number(),phase_rad:z.number().optional(),direction_rad:z.number(),water_depth_m:z.number().optional()}).optional(),data_sources:z.object({mode:z.enum(["realtime_forecast","historical_replay"]),ndbc:z.object({enabled:z.boolean()}).optional(),coops:z.object({enabled:z.boolean()}).optional(),nws:z.object({enabled:z.boolean(),user_agent:z.object({application:z.string(),contact:z.string()}).optional()}).optional(),rtofs:z.object({enabled:z.boolean()}).optional(),era5:z.object({enabled:z.boolean(),credentials:credentials.optional()}).optional()}).optional()});
const initialState=z.object({position_ned_m:vector3.optional(),attitude_rad:vector3.optional(),body_velocity_mps:vector3.optional(),angular_rate_body_rad_s:vector3.optional()}),sensorArtifacts=z.object({mode:z.enum(["summary","selected-raw","all-raw"]),raw_plugins:z.array(z.string()).optional(),max_bytes_per_run:z.number().int().optional()});
const vehicle=z.object({preset:z.string(),plant:z.enum(["planar3","coupled6"]),hydrodynamic_reference_speed_mps:z.number().optional(),hydrodynamics:z.object({artifact_checksum_sha256:z.string(),frequency_grid_rad_s:z.array(z.number()),extrapolation_policy:z.literal("reject")}).optional()});
const experiment=z.object({schema_version:z.literal(1),experiment:z.object({name:z.string(),seed:z.number().int(),timestep_s:z.number(),duration_s:z.number()}),backend:z.object({type:z.enum(["browser","node","tensor-cpu","tensor-mps","tensor-cuda"]),parallel_environments:z.number().int().optional()}),vehicle,environment:environment.optional(),initial_state:initialState.optional(),integrator:z.enum(["rk4","semi-implicit-euler"]).optional(),sensors:z.array(z.object({plugin:z.string(),enabled:z.boolean().optional(),oracle:z.boolean().optional()})).optional(),mission,metrics:z.array(z.string()).optional(),outputs:z.object({directory:z.string().optional(),state_hz:z.number().optional(),sensor_artifacts:sensorArtifacts.optional()}).optional()});
const reply=(value:unknown)=>({content:[{type:"text" as const,text:JSON.stringify(value)}]});
server.registerTool("list_vehicle_presets",{description:"List vehicle presets",inputSchema:{}},async()=>reply(listVehiclePresets()));
server.registerTool("list_sensor_plugins",{description:"List built-in sensor plugins",inputSchema:{}},async()=>reply(listSensorPlugins()));
server.registerTool("validate_scenario",{description:"Validate an ExperimentV1 scenario",inputSchema:experiment},async input=>reply(validateScenario(input as any)));
server.registerTool("construct_scenario",{description:"Validate, resolve, and retain a scenario",inputSchema:experiment},async input=>reply(constructScenario(input as any)));
server.registerTool("run_scenario",{description:"Run a retained scenario. Returned metrics.propulsion_energy is propulsion-only energy; total_energy also includes sensor cost.",inputSchema:{scenario_id:z.string()}},async input=>reply(await runScenario(input)));
server.registerTool("get_run_metrics",{description:"Read a completed run's persisted metrics. metrics.propulsion_energy is propulsion-only energy; total_energy also includes sensor cost.",inputSchema:{run_id:z.string()}},async input=>reply(await getRunMetrics(input)));
await server.connect(new StdioServerTransport());
