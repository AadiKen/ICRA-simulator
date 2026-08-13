import {
    scenarioConfig,
    simConfig,
    boatConfig,
    sensorConfig,
    gpsSensor,
    imuSensor,
    dayCamSensor,
    nightCamSensor,
    lidarSensor,
    exo2Sensor,
    envConfig,
    visibility,
    waterFieldConfig,
    waveConfig,
    goalConfig,
    controlConfig,
    Obstacle,
    Zone,
    simulator,
    vec3
} from "./schema.js?v=26";
import {DemoRenderer} from "./demoRenderer.js?v=20";
import {ThreeSensorProvider} from "./threeSensorProvider.js?v=27";
import {SensorStreamPublisher} from "./sensorStreamPublisher.js?v=26";

const STEPS = [
    {id: "vehicle", eyebrow: "Step 1", title: "Vehicle"},
    {id: "obstacles", eyebrow: "Step 2", title: "Operating Area"},
    {id: "environment", eyebrow: "Step 3", title: "Water Conditions"},
    {id: "route", eyebrow: "Step 4", title: "Route"},
    {id: "run", eyebrow: "Live", title: "Mission Run"},
    {id: "results", eyebrow: "Summary", title: "Results"}
];

const SENSOR_TYPES = [
    {type: "gps", label: "GPS · legacy browser"},
    {type: "imu", label: "IMU · legacy browser"},
    {type: "lidar", label: "LiDAR · legacy ThreeSensorProvider"},
    {type: "dayCam", label: "Day camera · legacy ThreeSensorProvider"},
    {type: "nightCam", label: "Night camera · legacy ThreeSensorProvider"}
];

const SENSOR_ICON_KEYS = {
    gps: "gps",
    imu: "imu",
    lidar: "lidar",
    dayCam: "day-camera",
    nightCam: "night-camera",
    exo2: "exo2-water-sensor"
};

const WEATHER_PRESETS = {
    clear: {label: "Clear", rain: 1.0, fog: 1.0, description: "High visibility and clean sensor returns."},
    rainy: {label: "Rainy", rain: 0.55, fog: 0.85, description: "Lower visibility, noisy cameras, and degraded LiDAR returns."},
    foggy: {label: "Foggy", rain: 0.95, fog: 0.35, description: "Strong visual haze and reduced camera contrast."}
};

function sensorConfigItem(type, name, mountPosition, mountOrientation, details = {}) {
    return {
        type,
        name,
        mountPosition,
        mountOrientation,
        details: {...details}
    };
}

function smoothWaves() {
    return [
        {height: 0.10, heading: 35, speed: 0.82, wavelength: 14, steepness: 0.25},
        {height: 0.06, heading: 120, speed: 0.62, wavelength: 9, steepness: 0.18}
    ];
}

function choppyWaves() {
    return [
        {height: 0.24, heading: 30, speed: 1.35, wavelength: 10, steepness: 0.45},
        {height: 0.18, heading: 125, speed: 1.18, wavelength: 6, steepness: 0.32}
    ];
}

const config = {
    renderMode: "cinematic",
    physicsMode: new URLSearchParams(window.location.search).get("physicsMode") === "planar3" ? "planar3" : "coupled6",
    boatPreset: "survey",
    boatMode: "preset",
    boat: {
        maxSpeed: 2.5,
        maxAcceleration: 0.9,
        maxDeceleration: 0.9,
        maxTurn: 0.7,
        mass: 1000,
        dimensions: new vec3(2.4, 1.0, 4.8),
        startPos: new vec3(12, 0, 12)
    },
    sensors: [
        sensorConfigItem("gps", "GPS", new vec3(0, 0.8, -0.5), new vec3(0, 0, 0)),
        sensorConfigItem("imu", "IMU", new vec3(0, 0.25, 0), new vec3(0, 0, 0)),
        sensorConfigItem("dayCam", "Day Camera", new vec3(0, 0.75, 2.05), new vec3(0, 0, 0), {width: 240, height: 180, fov: 60, hz: 1}),
        sensorConfigItem("lidar", "Lidar", new vec3(0, 1.05, 0.35), new vec3(0, 0, 0), {hRange: 90, vRange: 28, dRange: 25, angularRes: 2, hz: 1})
    ],
    obstacles: [
        {x: 38, z: 25, r: 3, type: "rock"},
        {x: 58, z: 47, r: 4, type: "fountain"}
    ],
    zones: [],
    zoneDraft: [],
    zonePreview: null,
    weather: "clear",
    waterPreset: "smooth",
    currentSpeed: 0.08,
    currentHeading: 76,
    waves: smoothWaves(),
    waypoints: [
        {x: 22, z: 22},
        {x: 25, z: 50},
        {x: 72, z: 65}
    ],
    goalTolerance: 1,
    streamSensors: true
};

const app = {
    renderer: null,
    sim: null,
    sensorProvider: null,
    sensorPublisher: null,
    stepIdx: 0,
    running: false,
    accumulator: 0,
    speedMultiplier: 1,
    loopStarted: false,
    lastFrameMs: 0,
    selectedPlacement: null,
    autoTimer: 0,
    lastFeedSignature: "",
    livePowerSamples: [],
    showRunStats: false,
    pendingZone: null,
    editingSensorIdx: null,
    addingSensorType: "dayCam",
    editingWaveIdx: null
};

const els = {};

function init() {
    cacheElements();
    exposeLaunchControls();
    bindControls();
    if (ensureRenderer()) {
        renderStep();
        startLoop();
    }
}

function exposeLaunchControls() {
    window.bcodOpenRenderModeModal = openRenderModeModal;
    window.bcodLaunchWithRenderMode = launchWithRenderMode;
}

function ensureRenderer() {
    if (app.renderer) return true;
    try {
        app.renderer = new DemoRenderer(els.viewport, config);
        return true;
    }
    catch (error) {
        console.error("Failed to initialize demo renderer.", error);
        return false;
    }
}

function startLoop() {
    if (app.loopStarted) return;
    app.loopStarted = true;
    requestAnimationFrame(loop);
}

function cacheElements() {
    els.viewport = document.getElementById("demoViewport");
    els.hero = document.getElementById("heroOverlay");
    els.renderModeModal = document.getElementById("renderModeModal");
    els.panel = document.getElementById("demoPanel");
    els.launch = document.getElementById("launchDemo");
    els.eyebrow = document.getElementById("stepEyebrow");
    els.title = document.getElementById("stepTitle");
    els.content = document.getElementById("stepContent");
    els.back = document.getElementById("backStep");
    els.next = document.getElementById("nextStep");
    els.showSensors = document.getElementById("showSensors");
    els.sensorBadges = document.getElementById("sensorBadges");
    els.stats = document.getElementById("missionStats");
    els.sensorFeedDock = document.getElementById("sensorFeedDock");
}

function bindControls() {
    els.launch.addEventListener("click", (event) => {
        event.preventDefault();
        openRenderModeModal();
    });
    els.renderModeModal.querySelectorAll("[data-render-mode]").forEach((button) => {
        button.addEventListener("click", (event) => {
            event.preventDefault();
            launchWithRenderMode(button.dataset.renderMode);
        });
    });
    els.next.addEventListener("click", () => nextStep());
    els.back.addEventListener("click", () => previousStep());
    els.showSensors.addEventListener("change", () => {
        if (!app.renderer) return;
        app.renderer.setShowSensors(els.showSensors.checked);
        updateSensorFeedVisibility();
    });
    els.viewport.addEventListener("dragover", (event) => event.preventDefault());
    els.viewport.addEventListener("drop", handleViewportDrop);
    els.viewport.addEventListener("click", handleViewportClick);
    els.viewport.addEventListener("pointerdown", handleViewportPointerDown);
    window.addEventListener("pointermove", handleViewportPointerMove);
    window.addEventListener("pointerup", handleViewportPointerUp);
}

function openRenderModeModal() {
    if (!els.renderModeModal) return;
    els.renderModeModal.style.display = "grid";
    els.renderModeModal.classList.remove("hidden");
    els.renderModeModal.querySelectorAll("[data-render-mode]").forEach((button) => {
        button.classList.toggle("selected", button.dataset.renderMode === config.renderMode);
    });
}

function launchWithRenderMode(renderMode) {
    config.renderMode = renderMode === "cinematic" ? "cinematic" : "efficient";
    if (!ensureRenderer()) return;
    els.renderModeModal.classList.add("hidden");
    els.renderModeModal.style.display = "";
    els.hero.classList.add("hidden");
    els.panel.classList.add("show");
    app.renderer.setConfig(config);
    app.renderer.setPhase("boat");
    renderStep();
    startLoop();
}

function nextStep() {
    if (currentStep().id === "run") {
        app.running = false;
        setStep("results");
        return;
    }
    if (app.stepIdx < STEPS.length - 1) {
        app.stepIdx += 1;
        renderStep();
    }
}

function previousStep() {
    if (app.stepIdx > 0) {
        app.stepIdx -= 1;
        renderStep();
    }
}

function setStep(stepId) {
    const idx = STEPS.findIndex((step) => step.id === stepId);
    if (idx >= 0) {
        app.stepIdx = idx;
        renderStep();
    }
}

function currentStep() {
    return STEPS[app.stepIdx];
}

function renderStep() {
    const step = currentStep();
    els.panel.classList.toggle("results-mode", step.id === "results");
    config.editingSensorId = step.id === "vehicle" && app.editingSensorIdx !== null
        ? `${config.sensors[app.editingSensorIdx].type}_${app.editingSensorIdx}`
        : null;
    els.eyebrow.textContent = step.eyebrow;
    els.title.textContent = step.title;
    app.renderer.setPhase(step.id === "vehicle" ? "boat" : step.id);
    app.running = step.id === "run";
    els.back.disabled = app.stepIdx === 0;
    els.next.textContent = step.id === "route" ? "Run" : step.id === "run" ? "Results" : "Next";

    if (step.id === "run" && !app.sim) {
        startSimulation();
    }

    const renderers = {
        vehicle: renderVehicleStep,
        obstacles: renderObstacleStep,
        environment: renderEnvironmentStep,
        route: renderRouteStep,
        run: renderRunStep,
        results: renderResultsStep
    };
    renderers[step.id]();
    updatePanelVisibility();
    updateStats();
    updateSensorBadges();
    updateSensorFeedVisibility();
}

function invalidateSimulation() {
    app.sensorPublisher?.dispose();
    app.sensorProvider?.dispose();
    app.sim = null;
    app.sensorProvider = null;
    app.sensorPublisher = null;
    app.lastFeedSignature = "";
    app.livePowerSamples = [];
    app.renderer.attachSimulator(null);
    updateSensorFeedVisibility();
}

function updatePanelVisibility() {
    const step = currentStep().id;
    const showRunInfo = step === "run";
    const showSensorConfig = step === "vehicle" || showRunInfo;
    els.stats.classList.toggle("hidden-section", !showRunInfo);
    document.querySelector(".sensor-strip").classList.toggle("hidden-section", !showSensorConfig);
    document.querySelector(".panel-actions").classList.toggle("hidden-section", step === "results");
}

function renderVehicleStep() {
    els.content.innerHTML = `
        <p class="step-copy">Choose a vehicle profile and configure this legacy browser sensor suite. These controls do not select Sensor SDK typed plugins; use typed plugin IDs in a resolved production experiment. Sensor markers appear on the boat with short forward-facing rays.</p>
        <label class="range-row"><span>Physics model</span>
            <select id="demoPhysicsMode">
                <option value="coupled6" ${config.physicsMode === "coupled6" ? "selected" : ""}>Coupled 6-DoF</option>
                <option value="planar3" ${config.physicsMode === "planar3" ? "selected" : ""}>Planar 3-DoF</option>
            </select>
        </label>
        <div class="option-grid">
            <button class="option-card preset-card ${config.boatMode === "preset" ? "selected" : ""}" data-boat-mode="preset" type="button">
                <strong>Use Survey ASV Preset</strong><span>Balanced acceleration, turning, mass, and hull dimensions.</span>
            </button>
            <button class="option-card ${config.boatMode === "custom" ? "selected" : ""}" data-boat-mode="custom" type="button">
                <strong>Create Custom Boat</strong><span>Expose handling, mass, and hull sliders.</span>
            </button>
        </div>
        <div class="${config.boatMode === "custom" ? "" : "hidden-section"}">
            ${range("Max Speed", "maxSpeed", config.boat.maxSpeed, 1.2, 4.8, 0.1, "m/s")}
            ${range("Max Acceleration", "maxAcceleration", config.boat.maxAcceleration, 0.2, 2.0, 0.05, "m/s²")}
            ${range("Max Deceleration", "maxDeceleration", config.boat.maxDeceleration, 0.2, 2.0, 0.05, "m/s²")}
            ${range("Max Angular Acceleration", "maxTurn", config.boat.maxTurn, 0.2, 1.8, 0.05, "rad/s²")}
            ${range("Mass", "mass", config.boat.mass, 300, 3500, 50, "kg")}
            ${range("Beam", "beam", config.boat.dimensions.x, 1.2, 4.0, 0.1, "m")}
            ${range("Hull Height", "height", config.boat.dimensions.y, 0.5, 2.0, 0.05, "m")}
            ${range("Length", "length", config.boat.dimensions.z, 2.0, 8.0, 0.1, "m")}
        </div>
        <div class="compact-list">
            ${sensorDisplayItems().map((item, idx) => sensorRow(item.sensor, idx, item.label)).join("")}
        </div>
        <div class="add-sensor-panel">
            <div class="add-sensor-grid">
                ${SENSOR_TYPES.map((item) => `
                    <button class="sensor-type-button ${item.type === app.addingSensorType ? "selected" : ""}" data-add-sensor-type="${item.type}" type="button">
                        ${sensorIcon(item.type, true)}
                        <span>${item.label}</span>
                    </button>
                `).join("")}
            </div>
            <button id="addSensor" class="tool-button add-sensor-action" type="button">Add ${SENSOR_TYPES.find((item) => item.type === app.addingSensorType)?.label || "Sensor"}</button>
        </div>
        ${app.editingSensorIdx !== null ? sensorEditor(config.sensors[app.editingSensorIdx], app.editingSensorIdx) : ""}
    `;
    els.content.querySelectorAll("[data-boat-mode]").forEach((button) => {
        button.addEventListener("click", () => {
            config.boatMode = button.dataset.boatMode;
            if (config.boatMode === "preset") applySurveyPreset();
            invalidateSimulation();
            renderStep();
        });
    });
    document.getElementById("demoPhysicsMode").addEventListener("change", (event) => {
        config.physicsMode = event.target.value;
        invalidateSimulation();
        renderStep();
    });
    els.content.querySelectorAll("[data-add-sensor-type]").forEach((button) => {
        button.addEventListener("click", () => {
            app.addingSensorType = button.dataset.addSensorType;
            renderStep();
        });
    });
    document.getElementById("addSensor").addEventListener("click", () => {
        config.sensors.push(defaultSensorConfig(app.addingSensorType));
        app.editingSensorIdx = config.sensors.length - 1;
        invalidateSimulation();
        app.renderer.setConfig(config);
        renderStep();
    });
    bindSensorRows();
    bindSensorEditor();
    bindRanges();
    app.renderer.setConfig(config);
}

function applySurveyPreset() {
    config.boat.maxSpeed = 2.5;
    config.boat.maxAcceleration = 0.9;
    config.boat.maxDeceleration = 0.9;
    config.boat.maxTurn = 0.7;
    config.boat.mass = 1000;
    config.boat.dimensions = new vec3(2.4, 1.0, 4.8);
}

function defaultSensorConfig(type) {
    const label = SENSOR_TYPES.find((item) => item.type === type)?.label || type;
    const defaults = {
        gps: sensorConfigItem("gps", "GPS", new vec3(0, 0.8, -0.5), new vec3(0, 0, 0)),
        imu: sensorConfigItem("imu", "IMU", new vec3(0, 0.25, 0), new vec3(0, 0, 0)),
        dayCam: sensorConfigItem("dayCam", "Day Camera", new vec3(0, 0.75, 2.05), new vec3(0, 0, 0), {width: 240, height: 180, fov: 60, hz: 1}),
        nightCam: sensorConfigItem("nightCam", "Night Camera", new vec3(0.28, 0.75, 1.8), new vec3(0, 0, 0), {width: 240, height: 180, fov: 60, hz: 0.5}),
        lidar: sensorConfigItem("lidar", "Lidar", new vec3(0, 1.05, 0.35), new vec3(0, 0, 0), {hRange: 90, vRange: 28, dRange: 25, angularRes: 2, hz: 1})
    };
    return defaults[type] || sensorConfigItem(type, label, new vec3(0, 0.5, 0), new vec3(0, 0, 0));
}

function sensorRow(sensor, idx, label) {
    return `
        <div class="compact-row">
            <div class="sensor-row-title">${sensorIcon(sensor.type, true)}<div><strong>${label}</strong><span>legacy browser path · ${sensor.type} · pos ${fmtVec(sensor.mountPosition)} · orient ${fmtVec(sensor.mountOrientation)}</span></div></div>
            <div class="mini-actions">
                <button class="icon-button" data-edit-sensor="${idx}" type="button">Edit</button>
                <button class="icon-button" data-remove-sensor="${idx}" type="button">×</button>
            </div>
        </div>
    `;
}

function fmtVec(v) {
    return `${Number(v.x).toFixed(2)}, ${Number(v.y).toFixed(2)}, ${Number(v.z).toFixed(2)}`;
}

function sensorEditor(sensor, idx) {
    const display = sensorDisplayItems()[idx];
    const details = sensor.details || {};
    const cameraDetails = sensor.type === "dayCam" || sensor.type === "nightCam";
    const lidarDetails = sensor.type === "lidar";
    return `
        <div class="modal-panel">
            <h4 class="modal-title-with-icon">${sensorIcon(sensor.type, true)}<span>Edit ${display ? display.label : sensor.name}</span></h4>
            <div class="field-grid">
                ${numberField("x", "pos-x", sensor.mountPosition.x)}
                ${numberField("y", "pos-y", sensor.mountPosition.y)}
                ${numberField("z", "pos-z", sensor.mountPosition.z)}
                ${numberField("pitch", "rot-x", sensor.mountOrientation.x)}
                ${numberField("yaw", "rot-y", sensor.mountOrientation.y)}
                ${numberField("roll", "rot-z", sensor.mountOrientation.z)}
            </div>
            ${cameraDetails ? `
                <div class="field-grid">
                    ${numberField("width", "detail-width", details.width || 240, 1)}
                    ${numberField("height", "detail-height", details.height || 180, 1)}
                    ${numberField("FOV", "detail-fov", details.fov || 60, 1)}
                    ${numberField("Hz", "detail-hz", details.hz || 1, 0.1)}
                </div>
            ` : ""}
            ${lidarDetails ? `
                <div class="field-grid">
                    ${numberField("h FOV", "detail-hRange", details.hRange || 90, 1)}
                    ${numberField("v FOV", "detail-vRange", details.vRange || 28, 1)}
                    ${numberField("range", "detail-dRange", details.dRange || 25, 1)}
                    ${numberField("res", "detail-angularRes", details.angularRes || 2, 0.5)}
                    ${numberField("Hz", "detail-hz", details.hz || 1, 0.1)}
                </div>
            ` : ""}
            <div class="modal-actions">
                <button id="saveSensor" class="tool-button" data-save-sensor="${idx}" type="button">Save</button>
                <button id="cancelSensor" class="tool-button" type="button">Done</button>
            </div>
        </div>
    `;
}

function numberField(label, key, value, step = 0.05) {
    return `<label>${label}<input data-sensor-field="${key}" type="number" value="${Number(value).toFixed(step >= 1 ? 0 : 2)}" step="${step}"></label>`;
}

function bindSensorRows() {
    els.content.querySelectorAll("[data-edit-sensor]").forEach((button) => {
        button.addEventListener("click", () => {
            app.editingSensorIdx = Number(button.dataset.editSensor);
            renderStep();
        });
    });
    els.content.querySelectorAll("[data-remove-sensor]").forEach((button) => {
        button.addEventListener("click", () => {
            config.sensors.splice(Number(button.dataset.removeSensor), 1);
            app.editingSensorIdx = null;
            invalidateSimulation();
            app.renderer.setConfig(config);
            renderStep();
        });
    });
}

function bindSensorEditor() {
    const save = document.getElementById("saveSensor");
    const cancel = document.getElementById("cancelSensor");
    if (!save || app.editingSensorIdx === null) return;
    save.addEventListener("click", () => {
        const sensor = config.sensors[app.editingSensorIdx];
        const values = {};
        els.content.querySelectorAll("[data-sensor-field]").forEach((input) => {
            values[input.dataset.sensorField] = Number(input.value);
        });
        sensor.mountPosition = new vec3(values["pos-x"], values["pos-y"], values["pos-z"]);
        sensor.mountOrientation = new vec3(values["rot-x"], values["rot-y"], values["rot-z"]);
        Object.entries(values).forEach(([key, value]) => {
            if (key.startsWith("detail-")) {
                sensor.details[key.replace("detail-", "")] = value;
            }
        });
        invalidateSimulation();
        app.renderer.setConfig(config);
        renderStep();
    });
    cancel.addEventListener("click", () => {
        app.editingSensorIdx = null;
        renderStep();
    });
}

function renderObstacleStep() {
    els.content.innerHTML = `
        <p class="step-copy">Drag or select obstacles, then click the map. Favored and denied zones are polygon regions: click vertices, then click near the first vertex to close the zone.</p>
        <div class="option-grid">
            ${dragItem("rock", "Rock", "Hard collision object")}
            ${dragItem("fountain", "Fountain", "Larger obstacle / visual marker")}
            ${dragItem("deniedZone", "Denied Zone", "Region where selected sensors should be avoided")}
            ${dragItem("favoredZone", "Favored Zone", "Region where selected sensors are preferred")}
        </div>
        <p class="placement-hint">${obstacleSummaryText()}</p>
        <div class="compact-list">
            ${config.zones.map((zone, idx) => zoneRow(zone, idx)).join("")}
        </div>
        <button id="clearObstacles" class="tool-button" type="button">Clear Obstacles</button>
        ${app.pendingZone ? zoneSensorEditor(app.pendingZone) : ""}
    `;
    bindPlacementTools();
    bindZoneRows();
    bindZoneEditor();
    document.getElementById("clearObstacles").addEventListener("click", () => {
        config.obstacles = [];
        config.zones = [];
        config.zoneDraft = [];
        config.zonePreview = null;
        app.pendingZone = null;
        invalidateSimulation();
        app.renderer.setConfig(config);
        renderStep();
    });
}

function obstacleSummaryText() {
    if (config.zoneDraft.length) {
        return `${config.zoneDraft.length} zone vertices set. Click near the first vertex after at least three points to close.`;
    }
    return `${config.obstacles.length} collision obstacles and ${config.zones.length} sensor zones placed.`;
}

function zoneRow(zone, idx) {
    const label = zone.type === "favoredZone" ? "Favored Zone" : "Denied Zone";
    return `
        <div class="compact-row">
            <div><strong>${label} ${idx + 1}</strong><span>${zone.points.length} points</span><div class="zone-icon-row">${zoneSensorIcons(zone)}</div></div>
            <div class="mini-actions">
                <button class="icon-button" data-remove-zone="${idx}" type="button">×</button>
            </div>
        </div>
    `;
}

function zoneSensorIcons(zone) {
    const items = new Map(sensorDisplayItems().map((item) => [item.id, item]));
    if (!zone.sensors.length) return `<span>no sensors selected</span>`;
    return zone.sensors.map((id) => {
        const item = items.get(id);
        return item ? `<span class="mini-sensor-chip">${sensorIcon(item.type, true)}${item.label}</span>` : "";
    }).join("");
}

function bindZoneRows() {
    els.content.querySelectorAll("[data-remove-zone]").forEach((button) => {
        button.addEventListener("click", () => {
            config.zones.splice(Number(button.dataset.removeZone), 1);
            invalidateSimulation();
            app.renderer.setConfig(config);
            renderStep();
        });
    });
}

function zoneSensorEditor(zone) {
    const deny = zone.type === "deniedZone";
    const items = sensorDisplayItems();
    const selected = new Set(zone.sensors);
    return `
        <div class="modal-panel">
            <h4>${deny ? "Denied" : "Favored"} Zone Sensors</h4>
            <p class="step-copy">${deny ? "Select sensors to deny inside this zone." : "Select sensors to favor inside this zone."}</p>
            <div class="zone-sensor-grid">
                ${items.map((item) => `
                    <label class="sensor-choice ${selected.has(item.id) ? "selected" : ""}">
                        <input data-zone-sensor="${item.id}" type="checkbox" ${selected.has(item.id) ? "checked" : ""}>
                        ${sensorIcon(item.type, selected.has(item.id))}
                        <span>${item.label}</span>
                    </label>
                `).join("")}
            </div>
            <div class="modal-actions">
                <button id="saveZone" class="tool-button" type="button">Save Zone</button>
                <button id="cancelZone" class="tool-button" type="button">Cancel</button>
            </div>
        </div>
    `;
}

function bindZoneEditor() {
    const save = document.getElementById("saveZone");
    const cancel = document.getElementById("cancelZone");
    if (!app.pendingZone || !save || !cancel) return;
    els.content.querySelectorAll("[data-zone-sensor]").forEach((input) => {
        input.addEventListener("change", () => {
            const label = input.closest(".sensor-choice");
            const item = sensorDisplayItems().find((sensor) => sensor.id === input.dataset.zoneSensor);
            label.classList.toggle("selected", input.checked);
            const img = label.querySelector("img");
            if (img && item) {
                const icon = SENSOR_ICON_KEYS[item.type] || SENSOR_ICON_KEYS.gps;
                img.src = `./sensor-icons/${icon}${input.checked ? "" : "-inactive"}.svg`;
            }
        });
    });
    save.addEventListener("click", () => {
        app.pendingZone.sensors = [...els.content.querySelectorAll("[data-zone-sensor]:checked")]
            .map((input) => input.dataset.zoneSensor);
        config.zones.push(app.pendingZone);
        app.pendingZone = null;
        config.zonePreview = null;
        invalidateSimulation();
        app.renderer.setConfig(config);
        renderStep();
    });
    cancel.addEventListener("click", () => {
        app.pendingZone = null;
        config.zonePreview = null;
        invalidateSimulation();
        app.renderer.setConfig(config);
        renderStep();
    });
}

function renderEnvironmentStep() {
    els.content.innerHTML = `
        <p class="step-copy">Start from a two-wave preset or build a custom sea state. Each wave feeds the same water model used by the simulator physics.</p>
        <div class="weather-row">
            ${Object.entries(WEATHER_PRESETS).map(([key, item]) => `
                <button class="option-card weather-card ${config.weather === key ? "selected" : ""}" data-weather="${key}" type="button">
                    <strong>${item.label}</strong><span>${item.description}</span>
                </button>
            `).join("")}
        </div>
        <div class="wave-preset-row">
            <button class="option-card ${config.waterPreset === "smooth" ? "selected" : ""}" data-wave-preset="smooth" type="button">
                <strong>Smooth</strong><span>Lower waves and slower motion.</span>
            </button>
            <button class="option-card ${config.waterPreset === "choppy" ? "selected" : ""}" data-wave-preset="choppy" type="button">
                <strong>Choppy</strong><span>Higher, faster, mixed headings.</span>
            </button>
        </div>
        <div class="range-row">
            <label><span>Current Speed</span><strong id="currentSpeedValue">${config.currentSpeed.toFixed(2)} m/s</strong></label>
            <input id="currentSpeedSlider" type="range" min="0" max="1.5" step="0.01" value="${config.currentSpeed}">
        </div>
        <div class="range-row">
            <label><span>Current Heading</span><strong id="currentHeadingValue">${config.currentHeading.toFixed(0)}°</strong></label>
            <input id="currentHeadingSlider" type="range" min="0" max="359" step="1" value="${config.currentHeading}">
        </div>
        <div class="compact-list">
            ${config.waves.map((wave, idx) => waveRow(wave, idx)).join("")}
        </div>
        <button id="addWave" class="tool-button" type="button">Add Wave</button>
        ${app.editingWaveIdx !== null ? waveEditor(config.waves[app.editingWaveIdx], app.editingWaveIdx) : ""}
    `;
    els.content.querySelectorAll("[data-weather]").forEach((button) => {
        button.addEventListener("click", () => {
            config.weather = button.dataset.weather;
            invalidateSimulation();
            app.renderer.setConfig(config);
            renderStep();
        });
    });
    els.content.querySelectorAll("[data-wave-preset]").forEach((button) => {
        button.addEventListener("click", () => {
            config.waterPreset = button.dataset.wavePreset;
            config.waves = config.waterPreset === "choppy" ? choppyWaves() : smoothWaves();
            app.editingWaveIdx = null;
            invalidateSimulation();
            app.renderer.setConfig(config);
            renderStep();
        });
    });
    document.getElementById("addWave").addEventListener("click", () => {
        config.waterPreset = "custom";
        config.waves.push({height: 0.12, heading: 60, speed: 1.0, wavelength: 9, steepness: 0.28});
        app.editingWaveIdx = config.waves.length - 1;
        invalidateSimulation();
        app.renderer.setConfig(config);
        renderStep();
    });
    bindCurrentControls();
    bindWaveRows();
    bindWaveEditor();
}

function bindCurrentControls() {
    const speed = document.getElementById("currentSpeedSlider");
    const heading = document.getElementById("currentHeadingSlider");
    speed.addEventListener("input", () => {
        config.currentSpeed = Number(speed.value);
        document.getElementById("currentSpeedValue").textContent = `${config.currentSpeed.toFixed(2)} m/s`;
        invalidateSimulation();
        app.renderer.setConfig(config);
    });
    heading.addEventListener("input", () => {
        config.currentHeading = Number(heading.value);
        document.getElementById("currentHeadingValue").textContent = `${config.currentHeading.toFixed(0)}°`;
        invalidateSimulation();
        app.renderer.setConfig(config);
    });
}

function waveRow(wave, idx) {
    return `
        <div class="wave-chip">
            <div><strong>Wave ${idx + 1} </strong><span>${wave.height.toFixed(2)} m · ${wave.heading.toFixed(0)}° · ${wave.speed.toFixed(2)} m/s</span></div>
            <div class="mini-actions">
                <button class="icon-button" data-edit-wave="${idx}" type="button">Edit</button>
                <button class="icon-button" data-remove-wave="${idx}" type="button">×</button>
            </div>
        </div>
    `;
}

function waveEditor(wave, idx) {
    return `
        <div class="modal-panel">
            <h4>Edit Wave ${idx + 1} </h4>
            <div class="field-grid">
                ${numberField("height", "wave-height", wave.height, 0.02)}
                ${numberField("heading", "wave-heading", wave.heading, 1)}
                ${numberField("speed", "wave-speed", wave.speed, 0.05)}
                ${numberField("length", "wave-wavelength", wave.wavelength || 9, 0.5)}
                ${numberField("steep", "wave-steepness", wave.steepness || 0.25, 0.05)}
            </div>
            <div class="modal-actions">
                <button id="saveWave" class="tool-button" type="button">Save</button>
                <button id="doneWave" class="tool-button" type="button">Done</button>
            </div>
        </div>
    `;
}

function bindWaveRows() {
    els.content.querySelectorAll("[data-edit-wave]").forEach((button) => {
        button.addEventListener("click", () => {
            app.editingWaveIdx = Number(button.dataset.editWave);
            renderStep();
        });
    });
    els.content.querySelectorAll("[data-remove-wave]").forEach((button) => {
        button.addEventListener("click", () => {
            config.waterPreset = "custom";
            config.waves.splice(Number(button.dataset.removeWave), 1);
            app.editingWaveIdx = null;
            invalidateSimulation();
            app.renderer.setConfig(config);
            renderStep();
        });
    });
}

function bindWaveEditor() {
    const save = document.getElementById("saveWave");
    const done = document.getElementById("doneWave");
    if (!save || app.editingWaveIdx === null) return;
    save.addEventListener("click", () => {
        const values = {};
        els.content.querySelectorAll("[data-sensor-field^='wave-']").forEach((input) => {
            values[input.dataset.sensorField.replace("wave-", "")] = Number(input.value);
        });
        config.waves[app.editingWaveIdx] = {
            height: values.height,
            heading: values.heading,
            speed: values.speed,
            wavelength: values.wavelength,
            steepness: values.steepness
        };
        config.waterPreset = "custom";
        invalidateSimulation();
        app.renderer.setConfig(config);
        renderStep();
    });
    done.addEventListener("click", () => {
        app.editingWaveIdx = null;
        renderStep();
    });
}

function renderRouteStep() {
    els.content.innerHTML = `
        <p class="step-copy">Click the water to add waypoints. The route becomes the simulator goal sequence.</p>
        <p class="placement-hint">${config.waypoints.length} waypoints set. Need at least one waypoint to run.</p>
        <button id="clearRoute" class="tool-button" type="button">Clear Route</button>
    `;
    document.getElementById("clearRoute").addEventListener("click", () => {
        config.waypoints = [];
        invalidateSimulation();
        app.renderer.setConfig(config);
        renderStep();
    });
}

function renderRunStep() {
    config.streamSensors = shouldStreamSensors();
    els.content.innerHTML = `
        <p class="step-copy">The mission is running from the live simulator. External sensor streaming is ${config.streamSensors ? "enabled" : "off"}.</p>
        <div class="range-row">
            <label><span>Simulation Speed</span><strong id="simSpeedValue">${app.speedMultiplier.toFixed(1)}x</strong></label>
            <input id="simSpeedSlider" type="range" min="0.25" max="4" step="0.25" value="${app.speedMultiplier}">
        </div>
        <button id="toggleRunStats" class="tool-button" type="button">${app.showRunStats ? "Hide" : "Show"} Detailed Statistics</button>
        <div id="runStatsPanel" class="run-stats-panel ${app.showRunStats ? "" : "hidden-section"}"></div>
        <div class="graph-card">
            <div class="graph-title"><span>Live Power</span><strong>Current vs max sensor draw</strong></div>
            <canvas id="livePowerGraph"></canvas>
        </div>
        <button id="restartMission" class="tool-button" type="button">Restart Mission</button>
    `;
    document.getElementById("toggleRunStats").addEventListener("click", () => {
        app.showRunStats = !app.showRunStats;
        renderStep();
    });
    document.getElementById("simSpeedSlider").addEventListener("input", (event) => {
        app.speedMultiplier = Number(event.target.value);
        document.getElementById("simSpeedValue").textContent = `${app.speedMultiplier.toFixed(1)}x`;
    });
    document.getElementById("restartMission").addEventListener("click", startSimulation);
    updateRunStatsPanel();
}

function shouldStreamSensors() {
    const params = new URLSearchParams(window.location.search);
    return params.get("streamSensors") !== "0" &&
        params.get("stream") !== "0" &&
        window.localStorage.getItem("bcodStreamSensors") !== "0";
}

function renderResultsStep() {
    app.running = false;
    const state = app.sim ? app.sim.state : null;
    const completed = state && state.goal.completed;
    els.content.innerHTML = `
        <p class="step-copy">${completed ? "Mission completed." : "Mission is not complete yet."} Review the run and adjust configuration if needed.</p>
        <div class="results-grid">
            <div class="graph-card wide-graph">
                <div class="graph-title"><span>Power</span><strong>Current vs max sensor draw</strong></div>
                <canvas id="finalPowerGraph"></canvas>
            </div>
            <div class="graph-card">
                <div class="graph-title"><span>Sensor Timeline</span><strong>Activation intervals</strong></div>
                <canvas id="finalSensorGraph"></canvas>
            </div>
            <div class="graph-card">
                <div class="graph-title"><span>Energy</span><strong>Usage by sensor</strong></div>
                <canvas id="finalEnergyGraph"></canvas>
            </div>
        </div>
        <div class="result-summary">
            ${metric("Energy Used", `${resultEnergyUsed().toFixed(2)}`)}
            ${metric("Energy Reduction", `${resultEnergyReduction().toFixed(1)}%`)}
        </div>
        <button id="rerunMission" class="tool-button" type="button">Run Again</button>
    `;
    document.getElementById("rerunMission").addEventListener("click", () => {
        startSimulation();
        setStep("run");
    });
    requestAnimationFrame(drawResultsGraphs);
}

function range(label, key, value, min, max, step, unit) {
    return `
        <div class="range-row">
            <label><span>${label}</span><strong>${Number(value).toFixed(step < 1 ? 2 : 0)} ${unit}</strong></label>
            <input data-range="${key}" type="range" min="${min}" max="${max}" value="${value}" step="${step}">
        </div>
    `;
}

function dragItem(type, label, description) {
    return `
        <div class="drag-item ${app.selectedPlacement === type ? "selected" : ""}" draggable="true" data-place="${type}">
            <strong>${label}</strong><span>${description}</span>
        </div>
    `;
}

function bindRanges() {
    els.content.querySelectorAll("[data-range]").forEach((input) => {
        input.addEventListener("input", () => {
            const key = input.dataset.range;
            const value = Number(input.value);
            if (key in config.boat) config.boat[key] = value;
            if (key === "beam") config.boat.dimensions.x = value;
            if (key === "height") config.boat.dimensions.y = value;
            if (key === "length") config.boat.dimensions.z = value;
            if (key in config.waves) config.waves[key] = value;
            invalidateSimulation();
            app.renderer.setConfig(config);
            renderStep();
        });
    });
}

function bindPlacementTools() {
    els.content.querySelectorAll("[data-place]").forEach((item) => {
        item.addEventListener("click", () => {
            app.selectedPlacement = app.selectedPlacement === item.dataset.place ? null : item.dataset.place;
            renderStep();
        });
        item.addEventListener("dragstart", (event) => {
            app.selectedPlacement = item.dataset.place;
            event.dataTransfer.setData("text/plain", item.dataset.place);
        });
    });
}

function handleViewportDrop(event) {
    event.preventDefault();
    if (currentStep().id !== "obstacles") return;
    const type = event.dataTransfer.getData("text/plain") || app.selectedPlacement;
    if (type === "deniedZone" || type === "favoredZone") {
        addZonePoint(type, event.clientX, event.clientY);
    }
    else {
        placeObstacle(type, event.clientX, event.clientY);
    }
}

function handleViewportClick(event) {
    const step = currentStep().id;
    if (app.vehicleCameraDragged) {
        app.vehicleCameraDragged = false;
        return;
    }
    if (step === "obstacles" && app.selectedPlacement) {
        if (app.selectedPlacement === "deniedZone" || app.selectedPlacement === "favoredZone") {
            addZonePoint(app.selectedPlacement, event.clientX, event.clientY);
        }
        else {
            placeObstacle(app.selectedPlacement, event.clientX, event.clientY);
        }
    }
    if (step === "route") {
        const point = app.renderer.screenToWorld(event.clientX, event.clientY);
        if (!point) return;
        config.waypoints.push({x: point.x, z: point.z});
        invalidateSimulation();
        app.renderer.setConfig(config);
        renderStep();
    }
}

function handleViewportPointerDown(event) {
    const step = currentStep().id;
    if ((step !== "vehicle" && step !== "run") || event.button !== 0) return;
    app.vehicleCameraDrag = {
        x: event.clientX,
        y: event.clientY,
        moved: false,
        mode: step
    };
    els.viewport.setPointerCapture?.(event.pointerId);
}

function handleViewportPointerMove(event) {
    if (!app.vehicleCameraDrag) return;
    const dx = event.clientX - app.vehicleCameraDrag.x;
    const dy = event.clientY - app.vehicleCameraDrag.y;
    app.vehicleCameraDrag.x = event.clientX;
    app.vehicleCameraDrag.y = event.clientY;
    if (Math.abs(dx) + Math.abs(dy) > 0) {
        app.vehicleCameraDrag.moved = true;
        if (app.vehicleCameraDrag.mode === "run") {
            app.renderer.panRunCamera(dx, dy);
        }
        else {
            app.renderer.panVehicleCamera(dx, dy);
        }
    }
}

function handleViewportPointerUp() {
    if (!app.vehicleCameraDrag) return;
    app.vehicleCameraDragged = app.vehicleCameraDrag.moved;
    app.vehicleCameraDrag = null;
}

function placeObstacle(type, clientX, clientY) {
    if (!type) return;
    const point = app.renderer.screenToWorld(clientX, clientY);
    if (!point) return;
    config.obstacles.push({
        x: point.x,
        z: point.z,
        r: type === "fountain" ? 4 : 3,
        type
    });
    invalidateSimulation();
    app.renderer.setConfig(config);
    renderStep();
}

function addZonePoint(type, clientX, clientY) {
    const point = app.renderer.screenToWorld(clientX, clientY);
    if (!point) return;
    const draft = config.zoneDraft;
    const first = draft[0];
    const closesZone = draft.length >= 3 && first && Math.hypot(point.x - first.x, point.z - first.z) <= 2.25;
    if (closesZone) {
        app.pendingZone = {
            type,
            points: draft.map((item) => ({x: item.x, z: item.z})),
            sensors: type === "deniedZone" ? sensorDisplayItems().map((item) => item.id) : []
        };
        config.zonePreview = app.pendingZone;
        config.zoneDraft = [];
    }
    else {
        config.zoneDraft.push({x: point.x, z: point.z, type});
    }
    invalidateSimulation();
    app.renderer.setConfig(config);
    renderStep();
}

function startSimulation() {
    app.sensorPublisher?.dispose();
    app.sensorProvider?.dispose();
    const scenario = createScenarioFromConfig();
    app.sim = new simulator(scenario);
    app.renderer.attachSimulator(app.sim);
    app.sensorProvider = new ThreeSensorProvider(app.renderer, {
        getTargets: () => app.renderer.getSensorTargets(),
        syncScene: () => app.renderer.syncToSimulation(),
        getWeather: () => config.weather
    });
    app.sim.setSensorProvider(app.sensorProvider);
    config.streamSensors = shouldStreamSensors();
    app.sensorPublisher = new SensorStreamPublisher({enabled: config.streamSensors});
    app.accumulator = 0;
    app.lastFeedSignature = "";
    app.livePowerSamples = [];
    app.running = true;
    updateSensorFeedVisibility();
}

function createScenarioFromConfig() {
    const sensorDict = {};
    config.sensors.forEach((item, idx) => {
        sensorDict[`${item.type}_${idx}`] = buildSensor(item);
    });

    const obstacles = config.obstacles.map((obs) => new Obstacle(new vec3(obs.x, 0, obs.z), obs.r, true));
    const deniedZones = config.zones
        .filter((zone) => zone.type === "deniedZone")
        .map((zone) => new Zone(zone.points.map((point) => new vec3(point.x, 0, point.z)), "deniedZone", zone.sensors));
    const favoredZones = config.zones
        .filter((zone) => zone.type === "favoredZone")
        .map((zone) => new Zone(zone.points.map((point) => new vec3(point.x, 0, point.z)), "favoredZone", zone.sensors));
    const waypoints = config.waypoints.length > 0
        ? config.waypoints.map((wp) => new vec3(wp.x, 0, wp.z))
        : [new vec3(72, 0, 65)];
    const current = currentVector();

    return new scenarioConfig(
        new simConfig(12, 120, 42, true, config.physicsMode, {waveCoupling: "none"}),
        new boatConfig(
            config.boat.maxSpeed,
            config.boat.maxAcceleration,
            config.boat.maxDeceleration,
            config.boat.maxTurn,
            0.08,
            0.12,
            config.boat.startPos,
            new vec3(0, 0, 0),
            config.boat.dimensions,
            config.boat.mass,
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
            90,
            70,
            obstacles,
            deniedZones,
            favoredZones,
            new waterFieldConfig(
                [
                    ...config.waves.map((wave) => new waveConfig(
                        wave.heading,
                        wave.height,
                        wave.wavelength || 9,
                        wave.speed || 1,
                        wave.steepness || 0.25
                    ))
                ],
                current
            ),
            visibilityFromWeather(config.weather),
            "day"
        ),
        new goalConfig(waypoints, config.goalTolerance),
        new controlConfig("local", 2, "heuristic", 100, "relative")
    );
}

function currentVector() {
    const heading = (config.currentHeading * Math.PI) / 180;
    return new vec3(
        Math.sin(heading) * config.currentSpeed,
        0,
        Math.cos(heading) * config.currentSpeed
    );
}

function visibilityFromWeather(weather) {
    const preset = WEATHER_PRESETS[weather] || WEATHER_PRESETS.clear;
    return new visibility(preset.rain, preset.fog);
}

function buildSensor(item) {
    const d = item.details || {};
    if (item.type === "gps") {
        return new gpsSensor(item.name, 2, 0.25, item.mountPosition, item.mountOrientation);
    }
    if (item.type === "imu") {
        return new imuSensor(item.name, 10, 0.1, item.mountPosition, item.mountOrientation);
    }
    if (item.type === "dayCam") {
        return new dayCamSensor(item.name, d.height || 180, d.width || 240, d.fov || 60, d.hz || 1, 2, item.mountPosition, item.mountOrientation);
    }
    if (item.type === "nightCam") {
        return new nightCamSensor(item.name, d.height || 180, d.width || 240, d.fov || 60, d.hz || 0.5, 2.5, item.mountPosition, item.mountOrientation);
    }
    if (item.type === "lidar") {
        return new lidarSensor(item.name, d.hRange || 90, d.vRange || 28, d.dRange || 25, d.angularRes || 2, d.hz || 1, 3, item.mountPosition, item.mountOrientation);
    }
    if (item.type === "exo2") {
        return new exo2Sensor(item.name, 1, 1, item.mountPosition, item.mountOrientation);
    }
    return new gpsSensor(item.name, 1, 1, item.mountPosition, item.mountOrientation);
}

function loop(frameMs) {
    if (!app.lastFrameMs) app.lastFrameMs = frameMs;
    const dt = Math.min((frameMs - app.lastFrameMs) / 1000, 0.25);
    app.lastFrameMs = frameMs;

    if (app.running && app.sim && app.sim.simStatus()) {
        app.accumulator += dt * app.speedMultiplier;
        while (app.accumulator >= app.sim.stepTime && app.sim.simStatus()) {
            app.sim.step();
            app.accumulator -= app.sim.stepTime;
        }
        if (!app.sim.simStatus()) {
            app.running = false;
        }
    }

    app.renderer.update(frameMs / 1000);
    updateStats();
    updateRunStatsPanel();
    updateSensorBadges();
    updateSensorFeeds();
    requestAnimationFrame(loop);
}

function updateStats() {
    const state = app.sim ? app.sim.state : null;
    const metrics = state ? state.metrics : null;
    const speed = state ? Math.hypot(state.boat.velocity.x, state.boat.velocity.z) : 0;
    const waypoint = state ? `${state.goal.waypointIdx}/${state.goal.waypoints.length}` : `0/${config.waypoints.length}`;
    els.stats.innerHTML = `
        ${metric("Waypoint", waypoint)}
        ${metric("Speed", `${speed.toFixed(2)} m/s`)}
        ${metric("Energy", `${(metrics ? metrics.totalEnergy : 0).toFixed(2)}`)}
        ${metric("Steps", `${state ? state.steps : 0}`)}
    `;
}

function metric(label, value) {
    return `<div class="metric-card"><span>${label}</span><strong>${value}</strong></div>`;
}

function updateRunStatsPanel() {
    if (currentStep().id !== "run" || !app.showRunStats) return;
    const panel = document.getElementById("runStatsPanel");
    if (!panel || !app.sim) return;
    const state = app.sim.state;
    const boat = state.boat;
    const belief = state.boatBelief;
    const localEnv = state.localEnv;
    const posError = belief ? boat.pos.dist(belief.pos) : 0;
    const orientError = belief ? orientationDifference(boat.orientation, belief.orientation) : {x: 0, y: 0, z: 0, mag: 0};
    const activeSensors = Array.isArray(state.activeSensors) ? state.activeSensors : Object.values(state.activeSensors || {});
    const activeNames = activeSensors.map((id) => sensorDisplayItems().find((item) => item.id === id)?.label || id);

    panel.innerHTML = `
        ${statRow("True Position", fmtVec(boat.pos))}
        ${statRow("Believed Position", fmtVec(belief.pos))}
        ${statRow("Position Error", `${posError.toFixed(3)} m`)}
        ${statRow("True Orientation", fmtVecRad(boat.orientation))}
        ${statRow("Believed Orientation", fmtVecRad(belief.orientation))}
        ${statRow("Orientation Error", `${orientError.mag.toFixed(3)} rad`)}
        ${statRow("Velocity", `${fmtVec(boat.velocity)} m/s`)}
        ${statRow("Angular Velocity", `${fmtVecRad(boat.angularVel)} rad/s`)}
        ${statRow("Weather", WEATHER_PRESETS[config.weather]?.label || config.weather)}
        ${statRow("Current Visibility", visibilityText(localEnv))}
        ${statRow("Rain Visibility", localEnv?.visibility ? Number(localEnv.visibility.rain).toFixed(3) : "n/a")}
        ${statRow("Fog Visibility", localEnv?.visibility ? Number(localEnv.visibility.fog).toFixed(3) : "n/a")}
        ${statRow("Water Height", `${Number(boat.waterHeight || 0).toFixed(3)} m`)}
        ${statRow("Draft", `${Number(boat.draft || 0).toFixed(3)} m`)}
        ${statRow("Active Sensors", activeNames.length ? activeNames.join(", ") : "none")}
        ${statRow("Last Sensor Cost", Number(state.metrics.lastSensorCost || 0).toFixed(3))}
        ${statRow("Last Movement Cost", Number(state.metrics.lastMovementCost || 0).toFixed(3))}
    `;
}

function statRow(label, value) {
    return `<div class="stat-row"><span>${label}</span><strong>${value}</strong></div>`;
}

function fmtVecRad(v) {
    return `${Number(v.x).toFixed(3)}, ${Number(v.y).toFixed(3)}, ${Number(v.z).toFixed(3)}`;
}

function orientationDifference(actual, believed) {
    const x = normalizeAngle(actual.x - believed.x);
    const y = normalizeAngle(actual.y - believed.y);
    const z = normalizeAngle(actual.z - believed.z);
    return {x, y, z, mag: Math.hypot(x, y, z)};
}

function normalizeAngle(angle) {
    return Math.atan2(Math.sin(angle), Math.cos(angle));
}

function visibilityText(localEnv) {
    if (!localEnv || !localEnv.visibility) return "n/a";
    const visibilityValue = localEnv.visibility.total ?? localEnv.visibility.value ?? localEnv.visibility.range ?? localEnv.visibility;
    if (typeof visibilityValue === "number") return visibilityValue.toFixed(3);
    if (typeof visibilityValue === "object") {
        return Object.entries(visibilityValue)
            .map(([key, value]) => `${key}: ${typeof value === "number" ? value.toFixed(2) : value}`)
            .join(", ");
    }
    return String(visibilityValue);
}

function updateSensorBadges() {
    const feeds = app.sim ? app.sim.getSensorFeeds() : [];
    const labels = sensorDisplayItems();
    els.sensorBadges.innerHTML = labels.map((item) => {
        const feed = feeds.find((feedItem) => feedItem.id === item.id);
        const active = feed && feed.active && feed.status === "live";
        return `<div class="sensor-badge ${active ? "active" : ""}">${sensorIcon(item.type, active)}<span>${item.label.replace(" Camera", "Cam")}</span></div>`;
    }).join("");
}

function updateSensorFeedVisibility() {
    const show = els.showSensors.checked && currentStep().id === "run" && app.running && app.sim;
    els.sensorFeedDock.classList.toggle("show", Boolean(show));
    if (!show) {
        els.sensorFeedDock.innerHTML = "";
    }
}

function updateSensorFeeds() {
    if (!app.sim) {
        updateSensorFeedVisibility();
        return;
    }
    const feeds = app.sim.getSensorFeeds();
    const signature = feeds.map((feed) => `${feed.id}:${feed.t}:${feed.summary}`).join("|");
    updateSensorFeedVisibility();
    updateLivePowerGraph();
    if (signature === app.lastFeedSignature) return;
    app.lastFeedSignature = signature;

    if (config.streamSensors && app.sensorPublisher) {
        app.sensorPublisher.publish(feeds);
    }

    renderSensorFeedCards(feeds);
}

function sensorDisplayItems() {
    const totals = new Map();
    config.sensors.forEach((sensor) => totals.set(sensor.type, (totals.get(sensor.type) || 0) + 1));
    const seen = new Map();
    return config.sensors.map((sensor, idx) => {
        const count = (seen.get(sensor.type) || 0) + 1;
        seen.set(sensor.type, count);
        const base = SENSOR_TYPES.find((item) => item.type === sensor.type)?.label || sensor.name || sensor.type;
        return {
            id: `${sensor.type}_${idx}`,
            type: sensor.type,
            label: totals.get(sensor.type) > 1 ? `${base} ${count}` : base,
            sensor
        };
    });
}

function sensorIcon(type, active = true) {
    const icon = SENSOR_ICON_KEYS[type] || SENSOR_ICON_KEYS.gps;
    return `<img class="sensor-icon" src="./sensor-icons/${icon}${active ? "" : "-inactive"}.svg" alt="">`;
}

function renderSensorFeedCards(feeds) {
    if (!els.sensorFeedDock.classList.contains("show")) return;
    const names = new Map(sensorDisplayItems().map((item) => [item.id, item.label]));
    const existingIds = new Set([...els.sensorFeedDock.querySelectorAll("[data-feed-id]")].map((card) => card.dataset.feedId));
    feeds.forEach((feed) => {
        if (!existingIds.has(feed.id)) {
            els.sensorFeedDock.insertAdjacentHTML("beforeend", sensorFeedCardMarkup(feed, names.get(feed.id) || feed.name));
        }
        const card = els.sensorFeedDock.querySelector(`[data-feed-id="${cssEscape(feed.id)}"]`);
        if (!card) return;
        card.querySelector("h3").textContent = names.get(feed.id) || feed.name;
        card.querySelector(".feed-status").textContent = feed.status === "live" ? feed.summary : feed.summary || feed.status;
        const body = card.querySelector(".feed-body");
        drawSensorFeedBody(body, feed);
    });
    [...els.sensorFeedDock.querySelectorAll("[data-feed-id]")].forEach((card) => {
        if (!feeds.some((feed) => feed.id === card.dataset.feedId)) card.remove();
    });
}

function sensorFeedCardMarkup(feed, label) {
    return `
        <article class="feed-card show" data-feed-id="${feed.id}">
            <h3>${label}</h3>
            <div class="feed-body"></div>
            <p class="feed-status">${feed.summary || feed.status}</p>
        </article>
    `;
}

function drawSensorFeedBody(body, feed) {
    if (feed.displayType === "image" && feed.data && feed.data.imageDataUrl) {
        if (!body.querySelector("img")) body.innerHTML = `<img alt="${feed.name} output">`;
        body.querySelector("img").src = feed.data.imageDataUrl;
        return;
    }
    if (!body.querySelector("canvas")) body.innerHTML = "<canvas></canvas>";
    const canvas = body.querySelector("canvas");
    if (feed.displayType === "pointCloud" && feed.data && feed.data.ranges) {
        drawLidarMini(canvas, feed.data);
        return;
    }
    if (feed.displayType === "motion" && feed.data) {
        drawImuMini(canvas, feed.data);
        return;
    }
    if (feed.displayType === "position" && feed.data) {
        drawGpsMini(canvas, feed.data);
        return;
    }
    drawTextMini(canvas, feed.summary || "waiting");
}

function drawLidarMini(canvas, data) {
    canvas.width = data.width;
    canvas.height = data.height;
    const ctx = canvas.getContext("2d");
    const image = ctx.createImageData(data.width, data.height);
    const maxDistance = data.maxDistance || 1;
    data.ranges.forEach((range, idx) => {
        const hit = range < maxDistance;
        const near = 1 - Math.min(range / maxDistance, 1);
        image.data[idx * 4] = hit ? 50 + near * 205 : 18;
        image.data[idx * 4 + 1] = hit ? 140 + near * 95 : 31;
        image.data[idx * 4 + 2] = hit ? 230 - near * 105 : 42;
        image.data[idx * 4 + 3] = 255;
    });
    ctx.putImageData(image, 0, 0);
}

function drawGpsMini(canvas, data) {
    canvas.width = 240;
    canvas.height = 180;
    const ctx = canvas.getContext("2d");
    const pos = data.pos || {x: 0, y: 0, z: 0};
    const velocity = data.velocity || {x: 0, y: 0, z: 0};
    const x = THREEClamp((pos.x / 90) * canvas.width, 14, canvas.width - 14);
    const z = THREEClamp((pos.z / 70) * canvas.height, 14, canvas.height - 14);
    ctx.fillStyle = "#071116";
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    ctx.strokeStyle = "rgba(142, 221, 244, 0.22)";
    ctx.lineWidth = 1;
    for (let gx = 20; gx < canvas.width; gx += 40) {
        ctx.beginPath();
        ctx.moveTo(gx, 0);
        ctx.lineTo(gx, canvas.height);
        ctx.stroke();
    }
    for (let gy = 20; gy < canvas.height; gy += 40) {
        ctx.beginPath();
        ctx.moveTo(0, gy);
        ctx.lineTo(canvas.width, gy);
        ctx.stroke();
    }
    ctx.fillStyle = "#42e4ae";
    ctx.beginPath();
    ctx.arc(x, z, 6, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = "rgba(255,255,255,0.9)";
    ctx.font = "11px system-ui, sans-serif";
    ctx.fillText(`x ${pos.x.toFixed(2)} z ${pos.z.toFixed(2)}`, 12, 22);
    ctx.fillText(`speed ${Math.hypot(velocity.x || 0, velocity.z || 0).toFixed(2)} m/s`, 12, 40);
}

function drawTextMini(canvas, text) {
    canvas.width = 240;
    canvas.height = 180;
    const ctx = canvas.getContext("2d");
    ctx.fillStyle = "#071116";
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = "rgba(255,255,255,0.88)";
    ctx.font = "13px system-ui, sans-serif";
    wrapCanvasText(ctx, text || "waiting", 14, 28, canvas.width - 28, 18);
}

function drawImuMini(canvas, data) {
    canvas.width = 240;
    canvas.height = 180;
    const ctx = canvas.getContext("2d");
    const roll = data.orientation?.z || 0;
    const pitch = data.orientation?.x || 0;
    const yaw = data.orientation?.y || 0;
    const accel = data.acceleration || {x: 0, y: 0, z: 0};
    const angularVel = data.angularVel || {x: 0, y: 0, z: 0};
    const cx = canvas.width / 2;
    const cy = canvas.height / 2;
    const pitchOffset = THREEClamp(pitch, -0.65, 0.65) * 72;

    ctx.fillStyle = "#0b1720";
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(-roll);
    ctx.translate(0, pitchOffset);
    ctx.fillStyle = "#1f6f93";
    ctx.fillRect(-canvas.width, -canvas.height * 2, canvas.width * 2, canvas.height * 2);
    ctx.fillStyle = "#184b37";
    ctx.fillRect(-canvas.width, 0, canvas.width * 2, canvas.height * 2);
    ctx.strokeStyle = "#dff7ff";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(-canvas.width, 0);
    ctx.lineTo(canvas.width, 0);
    ctx.stroke();
    ctx.restore();

    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(cx - 34, cy);
    ctx.lineTo(cx - 8, cy);
    ctx.moveTo(cx + 8, cy);
    ctx.lineTo(cx + 34, cy);
    ctx.moveTo(cx, cy - 6);
    ctx.lineTo(cx, cy + 6);
    ctx.stroke();

    const accelMag = Math.hypot(accel.x || 0, accel.y || 0, accel.z || 0);
    const yawRate = angularVel.y || 0;
    ctx.fillStyle = "rgba(255,255,255,0.88)";
    ctx.font = "11px system-ui, sans-serif";
    ctx.fillText(`pitch ${pitch.toFixed(2)} roll ${roll.toFixed(2)}`, 12, 22);
    ctx.fillText(`yaw ${yaw.toFixed(2)} yaw rate ${yawRate.toFixed(2)}`, 12, 40);
    ctx.fillText(`accel ${accelMag.toFixed(2)} m/s2`, 12, 158);
}

function THREEClamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
}

function cssEscape(value) {
    if (window.CSS && CSS.escape) return CSS.escape(value);
    return String(value).replace(/"/g, '\\"');
}

function updateLivePowerGraph() {
    if (!app.sim || currentStep().id !== "run") return;
    const canvas = document.getElementById("livePowerGraph");
    const t = app.sim.state.time - app.sim.state.startTime;
    if (app.livePowerSamples.length && app.livePowerSamples[app.livePowerSamples.length - 1].t === t) {
        if (canvas) drawPowerGraph(canvas, app.livePowerSamples);
        return;
    }
    app.livePowerSamples.push({
        t,
        current: currentSensorPower(),
        max: maxSensorPower()
    });
    if (app.livePowerSamples.length > 240) app.livePowerSamples.shift();
    if (canvas) drawPowerGraph(canvas, app.livePowerSamples);
}

function currentSensorPower() {
    if (!app.sim) return 0;
    const active = new Set(Array.isArray(app.sim.state.activeSensors)
        ? app.sim.state.activeSensors
        : Object.values(app.sim.state.activeSensors || {}));
    return app.sim.state.sensors.sensors.reduce((total, sensor) => {
        return active.has(sensor.id) || active.has(sensor.name) ? total + sensor.cost : total;
    }, 0);
}

function maxSensorPower() {
    if (app.sim) {
        return app.sim.state.sensors.sensors.reduce((total, sensor) => total + sensor.cost, 0);
    }
    return config.sensors.reduce((total, sensor) => total + sensorCostEstimate(sensor.type), 0);
}

function sensorCostEstimate(type) {
    return {
        gps: 0.25,
        imu: 0.1,
        dayCam: 2,
        nightCam: 2.5,
        lidar: 3,
        exo2: 1
    }[type] || 1;
}

function drawResultsGraphs() {
    drawPowerGraph(document.getElementById("finalPowerGraph"), finalPowerSamples());
    drawSensorTimeline(document.getElementById("finalSensorGraph"));
    drawEnergyGraph(document.getElementById("finalEnergyGraph"));
}

function finalPowerSamples() {
    if (!app.sim) return [];
    return app.sim.logs.sensorActivations.map((entry) => ({
        t: entry.t - app.sim.state.startTime,
        current: sensorPowerForActive(entry.activeSensors),
        max: maxSensorPower()
    }));
}

function sensorPowerForActive(activeSensors) {
    if (!app.sim) return 0;
    const active = new Set(activeSensors || []);
    return app.sim.state.sensors.sensors.reduce((total, sensor) => {
        return active.has(sensor.id) || active.has(sensor.name) ? total + sensor.cost : total;
    }, 0);
}

function drawPowerGraph(canvas, samples) {
    if (!canvas) return;
    setupCanvas(canvas);
    const ctx = canvas.getContext("2d");
    clearGraph(ctx, canvas);
    drawGraphAxes(ctx, canvas, "s", "power");
    const maxY = Math.max(1, ...samples.map((sample) => sample.max), ...samples.map((sample) => sample.current));
    const maxT = Math.max(1, ...samples.map((sample) => sample.t));
    drawLineSeries(ctx, canvas, samples, "max", maxT, maxY, "#2c95c9");
    drawLineSeries(ctx, canvas, samples, "current", maxT, maxY, "#ff6b5f");
    drawLegend(ctx, [["Current", "#ff6b5f"], ["Max", "#2c95c9"]]);
}

function drawSensorTimeline(canvas) {
    if (!canvas || !app.sim) return;
    setupCanvas(canvas);
    const ctx = canvas.getContext("2d");
    clearGraph(ctx, canvas);
    const items = sensorDisplayItems();
    const maxT = Math.max(1, app.sim.state.time - app.sim.state.startTime);
    const rowH = (canvas.height - 52) / Math.max(items.length, 1);
    ctx.fillStyle = "rgba(255,255,255,0.8)";
    ctx.font = "11px system-ui, sans-serif";
    items.forEach((item, row) => {
        const y = 30 + row * rowH;
        ctx.fillText(item.label, 12, y + 11);
        ctx.fillStyle = "rgba(142, 221, 244, 0.18)";
        ctx.fillRect(105, y, canvas.width - 125, Math.max(7, rowH * 0.46));
        ctx.fillStyle = sensorColor(item.type);
        activeIntervals(item.id).forEach((interval) => {
            const x = 105 + (interval.start / maxT) * (canvas.width - 125);
            const w = Math.max(2, ((interval.end - interval.start) / maxT) * (canvas.width - 125));
            ctx.fillRect(x, y, w, Math.max(7, rowH * 0.46));
        });
        ctx.fillStyle = "rgba(255,255,255,0.8)";
    });
}

function drawEnergyGraph(canvas) {
    if (!canvas || !app.sim) return;
    setupCanvas(canvas);
    const ctx = canvas.getContext("2d");
    clearGraph(ctx, canvas);
    const usage = sensorEnergyUsage();
    const maxValue = Math.max(1, ...usage.map((item) => item.energy));
    const rowH = (canvas.height - 46) / Math.max(usage.length, 1);
    ctx.font = "11px system-ui, sans-serif";
    usage.forEach((item, idx) => {
        const y = 26 + idx * rowH;
        const w = ((canvas.width - 130) * item.energy) / maxValue;
        ctx.fillStyle = "rgba(255,255,255,0.82)";
        ctx.fillText(item.label, 12, y + 12);
        ctx.fillStyle = sensorColor(item.type);
        ctx.fillRect(105, y, w, Math.max(8, rowH * 0.46));
        ctx.fillStyle = "rgba(255,255,255,0.72)";
        ctx.fillText(item.energy.toFixed(2), 110 + w, y + 12);
    });
}

function activeIntervals(sensorId) {
    if (!app.sim) return [];
    const logs = app.sim.logs.sensorActivations;
    const intervals = [];
    let open = null;
    logs.forEach((entry, idx) => {
        const active = (entry.activeSensors || []).includes(sensorId);
        const t = entry.t - app.sim.state.startTime;
        if (active && open === null) open = t;
        if ((!active || idx === logs.length - 1) && open !== null) {
            intervals.push({start: open, end: active && idx === logs.length - 1 ? t + app.sim.stepTime : t});
            open = null;
        }
    });
    return intervals;
}

function sensorEnergyUsage() {
    if (!app.sim) return [];
    return sensorDisplayItems().map((item) => {
        const sensor = app.sim.state.sensors.sensors.find((candidate) => candidate.id === item.id);
        const duration = activeIntervals(item.id).reduce((total, interval) => total + interval.end - interval.start, 0);
        return {
            ...item,
            energy: duration * (sensor ? sensor.cost : sensorCostEstimate(item.type))
        };
    });
}

function resultEnergyUsed() {
    return sensorEnergyUsage().reduce((total, item) => total + item.energy, 0);
}

function resultEnergyReduction() {
    if (!app.sim) return 0;
    const elapsed = Math.max(app.sim.state.time - app.sim.state.startTime, app.sim.stepTime);
    const maxEnergy = maxSensorPower() * elapsed;
    if (maxEnergy <= 0) return 0;
    return ((maxEnergy - resultEnergyUsed()) / maxEnergy) * 100;
}

function setupCanvas(canvas) {
    const rect = canvas.getBoundingClientRect();
    canvas.width = Math.max(240, Math.floor(rect.width || 320));
    canvas.height = Math.max(180, Math.floor(rect.height || 220));
}

function clearGraph(ctx, canvas) {
    ctx.fillStyle = "#071116";
    ctx.fillRect(0, 0, canvas.width, canvas.height);
}

function drawGraphAxes(ctx, canvas, xLabel, yLabel) {
    ctx.strokeStyle = "rgba(255,255,255,0.22)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(40, 16);
    ctx.lineTo(40, canvas.height - 30);
    ctx.lineTo(canvas.width - 14, canvas.height - 30);
    ctx.stroke();
    ctx.fillStyle = "rgba(255,255,255,0.58)";
    ctx.font = "11px system-ui, sans-serif";
    ctx.fillText(xLabel, canvas.width - 24, canvas.height - 10);
    ctx.fillText(yLabel, 10, 16);
}

function drawLineSeries(ctx, canvas, samples, key, maxT, maxY, color) {
    if (!samples.length) return;
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    samples.forEach((sample, idx) => {
        const x = 40 + (sample.t / maxT) * (canvas.width - 54);
        const y = (canvas.height - 30) - (sample[key] / maxY) * (canvas.height - 48);
        if (idx === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
    });
    ctx.stroke();
}

function drawLegend(ctx, items) {
    ctx.font = "11px system-ui, sans-serif";
    items.forEach((item, idx) => {
        const x = 48 + idx * 76;
        ctx.fillStyle = item[1];
        ctx.fillRect(x, 18, 10, 10);
        ctx.fillStyle = "rgba(255,255,255,0.76)";
        ctx.fillText(item[0], x + 15, 27);
    });
}

function sensorColor(type) {
    return {
        gps: "#42e4ae",
        imu: "#ff7a90",
        lidar: "#ffd166",
        dayCam: "#8eddf4",
        nightCam: "#9d8cff",
        exo2: "#7ee081"
    }[type] || "#8eddf4";
}

function wrapCanvasText(ctx, text, x, y, maxWidth, lineHeight) {
    const words = String(text).split(/\s+/);
    let line = "";
    words.forEach((word) => {
        const nextLine = line ? `${line} ${word}` : word;
        if (ctx.measureText(nextLine).width > maxWidth && line) {
            ctx.fillText(line, x, y);
            y += lineHeight;
            line = word;
        }
        else {
            line = nextLine;
        }
    });
    if (line) ctx.fillText(line, x, y);
}

init();
