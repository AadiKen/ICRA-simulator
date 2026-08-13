import {
    scenarioConfig,
    simConfig,
    boatConfig,
    sensorConfig,
    dayCamSensor,
    lidarSensor,
    envConfig,
    visibility,
    waterFieldConfig,
    waveConfig,
    goalConfig,
    controlConfig,
    createInitialSimState,
    simulator,
    Obstacle,
    Zone,
    vec3,
} from "./schema.js";

export function initializeSimulationState(scenarioC){
    return createInitialSimState(scenarioC);
}

const wait = (ms) => new Promise(resolve => setTimeout(resolve, ms));

export async function simulation(){
    //-------------------------------------------------------------------------------------------------------------------------------
    //P L A C E H O L D E R   C O N F I G
    //-------------------------------------------------------------------------------------------------------------------------------
    const sensorDict = {
        sensor_1: new dayCamSensor("dayCam_1", 200, 200, 45, 1, 10),
        sensor_2: new lidarSensor("lidarSensor_1", 0.1, 0.1, 10, 0.01, 1, 40),
    };

    const obstacles = [new Obstacle(new vec3(0, 0, 0), 4, true)];
    const favoredZones = [new Zone([new vec3(0, 0, 0), new vec3(1, 0, 0), new vec3(0, 0, 1)], "favored", ["sensor_1"])];
    const deniedZones = [new Zone([new vec3(0, 0, 0), new vec3(1, 0, 0), new vec3(0, 0, 1)], "denied", ["sensor_2"])];
    const waves = [new waveConfig(320, 1, 3, 0.5, 1)];
    const waterfieldC = new waterFieldConfig(waves, new vec3(1, 0, 0.5));

    const vis = new visibility(0.2, 0.2);

    const waypoints = [new vec3(1, 0, 2), new vec3(4, 0, 5), new vec3(3, 0, 1)];

    const simC = new simConfig(1, 10, 0, true, "planar3");
    const boatC = new boatConfig(1, 1, 1, 1, 0, 0);
    const sensorC = new sensorConfig(sensorDict);
    const envC = new envConfig(20, 20, obstacles, deniedZones, favoredZones, waterfieldC, vis, "day");
    const goalC = new goalConfig(waypoints, 0.1);
    const controlC = new controlConfig("local", 0.125, "heuristic", 100);

    const scenarioC = new scenarioConfig(simC, boatC, sensorC, envC, goalC, controlC);
    //-------------------------------------------------------------------------------------------------------------------------------
    
    const sim = new simulator(scenarioC);
    const stepTime = 1000 / scenarioC.simConfig.simHz;
    let isSimulating = true;
    while (isSimulating){
        const stepStartTime = Date.now();

        sim.step();

        if (!sim.simStatus()){
            isSimulating = false;
            break;
        }

        const stepEndTime = Date.now();
        if (stepEndTime-stepStartTime < stepTime){
            await wait(stepTime - (stepEndTime-stepStartTime));

        }
        else{
            console.log("WARNING: Allotted Sim Timestep Exceeded");
        }
    }
}
