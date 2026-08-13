import {
    scenarioConfig,
    simConfig,
    boatConfig,
    sensorConfig,
    gpsSensor,
    imuSensor,
    dayCamSensor,
    lidarSensor,
    envConfig,
    visibility,
    waterFieldConfig,
    waveConfig,
    goalConfig,
    controlConfig,
    Obstacle,
    vec3
} from "./schema.js?v=25";

export function createDemoScenario(options = {}) {
    const sensorDict = {
        gps: new gpsSensor("GPS", 2, 0.25, new vec3(0, 0.8, -0.5), new vec3(0, 0, 0)),
        imu: new imuSensor("IMU", 10, 0.1, new vec3(0, 0.25, 0), new vec3(0, 0, 0)),
        dayCam: new dayCamSensor("Day Camera", 320, 240, 60, 1, 2, new vec3(0, 0.75, 2.05), new vec3(0, 0, 0)),
        lidar: new lidarSensor("Lidar", 90, 30, 25, 1, 1, 3, new vec3(0, 1.05, 0.35), new vec3(0, 0, 0))
    };

    const waves = [
        new waveConfig(35, 0.16, 11, 1.34, 0.45),
        new waveConfig(120, 0.16, 6, 1.24, 0.25)
    ];

    const boundsWidth = 90;
    const boundsHeight = 70;

    return new scenarioConfig(
        new simConfig(12, 90, 42, true, options.physicsMode || "coupled6", {
            waveCoupling: "none",
            logEvery: options.logEvery || 1
        }),
        new boatConfig(
            2.5,
            0.9,
            0.9,
            0.7,
            0.08,
            0.12,
            new vec3(12, 0, 12),
            new vec3(0, 0, 0),
            new vec3(2.4, 1.0, 4.8),
            1000,
            0.32,
            0.35,
            3.2,
            0.32,
            0.08,
            3.4,
            3.8,
            2.8,
            23.5,
            1.2,
            new vec3(0.24, 0.22, 0.26),
            0.16,
            new vec3(0.45, 1.2, 0.45),
            0.24,
            0.22
        ),
        new sensorConfig(sensorDict),
        new envConfig(
            boundsWidth,
            boundsHeight,
            [
                new Obstacle(new vec3(38, 0, 25), 3, true),
                new Obstacle(new vec3(58, 0, 47), 4, true)
            ],
            [],
            [],
            new waterFieldConfig(waves, new vec3(0.08, 0, 0.02)),
            new visibility(0.95, 0.9),
            "day"
        ),
        new goalConfig(
            [
                new vec3(22, 0, 22),
                new vec3(25, 0, 50),
                new vec3(72, 0, 65)
            ],
            1
        ),
        new controlConfig("local", 2, "heuristic", 100, "relative")
    );
}
