import {simulator} from "./schema.js?v=25";
import {createDemoScenario} from "./scenarioPresets.js?v=25";
import {ThreeStateRenderer} from "./threeStateRenderer.js?v=29";
import {ThreeSensorProvider} from "./threeSensorProvider.js?v=25";
import {SensorStreamPublisher} from "./sensorStreamPublisher.js?v=26";

const state = {
    sim: null,
    renderer: null,
    running: false,
    accumulator: 0,
    lastFrameMs: 0,
    speedMultiplier: 1,
    lastPanelUpdateMs: 0,
    lastSensorFeedSignature: "",
    sensorPublisher: null
};

const els = {};

function init() {
    cacheElements();
    resetSimulation();
    bindControls();
    requestAnimationFrame(loop);
}

function cacheElements() {
    els.viewport = document.getElementById("viewport");
    els.startPause = document.getElementById("startPause");
    els.step = document.getElementById("stepOnce");
    els.reset = document.getElementById("reset");
    els.runUntilDone = document.getElementById("runUntilDone");
    els.waterMode = document.getElementById("waterMode");
    els.physicsMode = document.getElementById("physicsMode");
    els.speed = document.getElementById("speed");
    els.debug = document.getElementById("debug");
    els.sensorFeeds = document.getElementById("sensorFeeds");
    els.status = document.getElementById("status");
    els.exportCSV = document.getElementById("exportCSV");
}

function bindControls() {
    els.startPause.addEventListener("click", () => {
        state.running = !state.running;
        updateControlLabels();
    });

    els.step.addEventListener("click", () => {
        state.sim.step();
        state.renderer.update();
        updateDebug();
        updateSensorFeeds(true);
    });

    els.reset.addEventListener("click", () => {
        resetSimulation();
    });

    els.runUntilDone.addEventListener("click", () => {
        state.sim.runUntilDone(5000);
        state.renderer.update();
        updateDebug();
        updateSensorFeeds(true);
        state.running = false;
        updateControlLabels();
    });

    els.waterMode.addEventListener("change", () => {
        state.renderer.setMode(els.waterMode.value);
        state.renderer.update();
        updateSensorFeeds(true);
    });
    els.physicsMode.addEventListener("change", resetSimulation);

    els.speed.addEventListener("input", () => {
        state.speedMultiplier = Number(els.speed.value);
        document.getElementById("speedValue").textContent = `${state.speedMultiplier.toFixed(1)}x`;
    });

    els.exportCSV.addEventListener("click", () => {
        exportLogsToCSV(state.sim);
    });
}

function resetSimulation() {
    const queryMode = new URLSearchParams(window.location.search).get("physicsMode");
    const physicsMode = queryMode === "planar3" || queryMode === "coupled6"
        ? queryMode
        : (els.physicsMode?.value || "coupled6");
    if (els.physicsMode) els.physicsMode.value = physicsMode;
    const scenario = createDemoScenario({physicsMode});
    state.sim = new simulator(scenario);
    state.running = false;
    state.accumulator = 0;
    state.lastPanelUpdateMs = 0;
    state.lastSensorFeedSignature = "";

    if (state.renderer) {
        state.sensorPublisher?.dispose();
        state.sensorProvider?.dispose();
        state.renderer.dispose();
        els.viewport.replaceChildren();
    }

    state.renderer = new ThreeStateRenderer(els.viewport, state.sim, {
        mode: els.waterMode ? els.waterMode.value : "physics",
        waterResolution: 64
    });
    state.sensorProvider = new ThreeSensorProvider(state.renderer, {
        getTargets: () => state.renderer.getSensorTargets(),
        syncScene: () => state.renderer.syncToSimulation()
    });
    state.sim.setSensorProvider(state.sensorProvider);
    state.sensorPublisher = new SensorStreamPublisher({enabled: shouldStreamSensors()});
    state.renderer.update();
    updateControlLabels();
    updatePanels(true);
}

function loop(frameMs) {
    if (!state.lastFrameMs) {
        state.lastFrameMs = frameMs;
    }
    const realDeltaSec = Math.min((frameMs - state.lastFrameMs) / 1000, 0.25);
    state.lastFrameMs = frameMs;

    if (state.running && state.sim.simStatus()) {
        state.accumulator += realDeltaSec * state.speedMultiplier;
        while (state.accumulator >= state.sim.stepTime && state.sim.simStatus()) {
            state.sim.step();
            state.accumulator -= state.sim.stepTime;
        }
        if (!state.sim.simStatus()) {
            state.running = false;
            updateControlLabels();
        }
    }

    state.renderer.update();
    updatePanels(false, frameMs);
    requestAnimationFrame(loop);
}

function updatePanels(force = false, frameMs = performance.now()) {
    if (!force && frameMs - state.lastPanelUpdateMs < 140) {
        return;
    }
    state.lastPanelUpdateMs = frameMs;
    updateDebug();
    updateSensorFeeds(force);
}

function updateControlLabels() {
    els.startPause.textContent = state.running ? "Pause" : "Start";
    els.status.textContent = state.sim.simStatus()
        ? state.running ? "Running" : "Paused"
        : `Stopped: ${state.sim.state.stopReason || "unknown"}`;
}

function shouldStreamSensors() {
    const params = new URLSearchParams(window.location.search);
    return params.get("streamSensors") === "1" ||
        params.get("stream") === "1" ||
        window.localStorage.getItem("bcodStreamSensors") === "1";
}

// ─── CSV Export ───────────────────────────────────────────────────────────────
//
// Axis remapping: your JS sim uses a right-handed Y-up frame where:
//   pos.z  = north (forward / surge axis)
//   pos.x  = east  (lateral / sway axis)
//   pos.y  = up    (heave axis)
//   heading = yaw around Y (radians)
//
// This matches MSS NED convention when exported as (north=z, east=x).
//
// Note: snapshotBoatState() does not record guidanceAcceleration separately —
// only the total acceleration is available in logs. The MSS benchmark script
// uses total_accel_n (forward component) as a proxy for the thrust input,
// which is accurate because environmental drag is small vs. guidance thrust
// during active manoeuvring. For a cleaner signal, see the note in the
// benchmark README about deriving tau from consecutive velocity differences.
//
function exportLogsToCSV(sim) {
    const boatLogs = sim.logs.boatStates;
    const metricLogs = sim.logs.metrics;

    if (!boatLogs || boatLogs.length === 0) {
        alert("No log data — run the simulation first.");
        return;
    }

    // Build a metrics lookup by index so we can join energy data per step.
    // boatStates and metrics are logged together in logger.log(), so indices align.
    const header = [
        "time",           // simulation time (s)
        "north",          // pos.z  — forward axis (m)
        "east",           // pos.x  — lateral axis (m)
        "height",         // pos.y  — vertical position (m), useful for heave validation
        "surge",          // velocity.z — forward speed (m/s)
        "sway",           // velocity.x — lateral speed (m/s)
        "heave",          // velocity.y — vertical speed (m/s)
        "heading",        // yaw angle (rad), positive = clockwise from north
        "roll",           // orientation.z (rad)
        "pitch",          // orientation.x (rad)
        "yaw_rate",       // angularVel.y (rad/s)
        "roll_rate",      // angularVel.z (rad/s)
        "pitch_rate",     // angularVel.x (rad/s)
        "total_accel_n",  // acceleration.z — total forward acceleration (m/s²)
        "total_accel_e",  // acceleration.x — total lateral acceleration (m/s²)
        "total_accel_up", // acceleration.y — total vertical acceleration (m/s²)
        "yaw_accel",      // angularAcceleration.y (rad/s²)
        "speed",          // scalar horizontal speed (m/s), convenience column
        "total_energy",   // cumulative energy cost from metrics log
        "last_total_cost",
        "down",
        "quat_w", "quat_x", "quat_y", "quat_z",
        "body_w", "roll_body_rate", "pitch_body_rate",
        "body_w_dot", "roll_body_accel", "pitch_body_accel",
        "physics_mode"
    ].join(",");

    const rows = boatLogs.map((s, i) => {
        const m = metricLogs[i] || {};
        const speed = Math.sqrt(
            (s.velocity.x || 0) ** 2 +
            (s.velocity.z || 0) ** 2
        );
        return [
            s.t.toFixed(4),
            (s.pos.z).toFixed(4),
            (s.pos.x).toFixed(4),
            (s.pos.y).toFixed(4),
            (s.velocity.z).toFixed(4),
            (s.velocity.x).toFixed(4),
            (s.velocity.y).toFixed(4),
            (s.heading).toFixed(6),
            (s.orientation.z).toFixed(6),
            (s.orientation.x).toFixed(6),
            (s.angularVel.y).toFixed(6),
            (s.angularVel.z).toFixed(6),
            (s.angularVel.x).toFixed(6),
            (s.acceleration.z).toFixed(6),
            (s.acceleration.x).toFixed(6),
            (s.acceleration.y).toFixed(6),
            (s.angularAcceleration.y).toFixed(6),
            speed.toFixed(4),
            (m.totalEnergy || 0).toFixed(4),
            (m.lastTotalCost || 0).toFixed(6),
            (s.rigidBody?.position.D || 0).toFixed(6),
            (s.quaternion?.w || 0).toFixed(9),
            (s.quaternion?.x || 0).toFixed(9),
            (s.quaternion?.y || 0).toFixed(9),
            (s.quaternion?.z || 0).toFixed(9),
            (s.bodyVelocity?.w || 0).toFixed(6),
            (s.bodyAngularRate?.p || 0).toFixed(6),
            (s.bodyAngularRate?.q || 0).toFixed(6),
            (s.bodyAcceleration?.wDot || 0).toFixed(6),
            (s.bodyAngularAccel?.pDot || 0).toFixed(6),
            (s.bodyAngularAccel?.qDot || 0).toFixed(6),
            s.physicsMode || sim.physicsMode
        ].join(",");
    });

    const stopReason = sim.state.stopReason || "unknown";
    const finalPos = boatLogs[boatLogs.length - 1];
    const summary = [
        `# BCOD Simulator export`,
        `# Steps: ${boatLogs.length}`,
        `# Stop reason: ${stopReason}`,
        `# Final position (north, east): ${finalPos.pos.z.toFixed(2)}, ${finalPos.pos.x.toFixed(2)}`,
        `# Sim Hz: ${Math.round(1 / sim.stepTime)}`,
        `# Axis convention: north=pos.z, east=pos.x, up=pos.y`
    ].join("\n");

    const blob = new Blob(
        [summary + "\n" + header + "\n" + rows.join("\n")],
        { type: "text/csv" }
    );
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `bcod_sim_${stopReason}_${boatLogs.length}steps.csv`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);

    console.log(
        `[Export] ${boatLogs.length} steps exported. ` +
        `Stop: ${stopReason}. ` +
        `Final pos: (${finalPos.pos.z.toFixed(1)} N, ${finalPos.pos.x.toFixed(1)} E)`
    );
}
// ─────────────────────────────────────────────────────────────────────────────

function updateDebug() {
    const simState = state.sim.state;
    const boat = simState.boat;
    const belief = simState.boatBelief;
    const metric = state.sim.logs.metrics[state.sim.logs.metrics.length - 1] || simState.metrics;
    const localEnv = simState.localEnv;
    const water = localEnv ? localEnv.waterSample : null;
    const activeSensors = Array.isArray(simState.activeSensors)
        ? simState.activeSensors
        : Object.values(simState.activeSensors || {});

    els.debug.innerHTML = `
        <div class="debug-grid">
            <span>time</span><strong>${simState.time.toFixed(2)}</strong>
            <span>time of day</span><strong>${simState.timeOfDay} ${fmtClock(simState.time)}</strong>
            <span>tick / steps</span><strong>${simState.tick} / ${simState.steps}</strong>
            <span>status</span><strong>${simState.isSimulating ? "active" : simState.stopReason}</strong>
            <span>guidance mode</span><strong>${simState.control.guidanceMode}</strong>
            <span>physics mode</span><strong>${simState.physicsMode} · waves ${simState.waveCoupling}</strong>
            <span>goal</span><strong>${simState.goal.waypointIdx}/${simState.goal.waypoints.length}</strong>
            <span>abs position</span><strong>${fmtVec(boat.pos)}</strong>
            <span>belief position</span><strong>${fmtVec(belief.pos)}</strong>
            <span>draft</span><strong>${boat.draft.toFixed(3)}</strong>
            <span>vel</span><strong>${fmtVec(boat.velocity)}</strong>
            <span>acc</span><strong>${fmtVec(boat.acceleration)}</strong>
            <span>abs orientation</span><strong>${fmtVec(boat.orientation)}</strong>
            <span>belief orientation</span><strong>${fmtVec(belief.orientation)}</strong>
            <span>abs / belief heading</span><strong>${boat.heading.toFixed(2)} / ${belief.heading.toFixed(2)}</strong>
            <span>angular vel</span><strong>${fmtVec(boat.angularVel)}</strong>
            <span>angular speed</span><strong>${vecMag(boat.angularVel).toFixed(3)}</strong>
            <span>env angular acc</span><strong>${fmtVec(boat.environmentAngularAcceleration)}</strong>
            <span>guidance angular acc</span><strong>${fmtVec(boat.guidanceAngularAcceleration)}</strong>
            <span>restoring angular acc</span><strong>${fmtVec(boat.restoringAngularAcceleration)}</strong>
            <span>total angular acc</span><strong>${fmtVec(boat.angularAcceleration)}</strong>
            <span>water height</span><strong>${water ? water.surfaceHeight.toFixed(3) : "n/a"}</strong>
            <span>water normal</span><strong>${water ? fmtVec(water.normal) : "n/a"}</strong>
            <span>active sensors</span><strong>${activeSensors.join(", ") || "none"}</strong>
            <span>energy</span><strong>${metric.totalEnergy.toFixed(3)}</strong>
            <span>last cost</span><strong>${metric.lastTotalCost.toFixed(3)}</strong>
            <span>logs</span><strong>${state.sim.logs.boatStates.length}</strong>
        </div>
    `;
}

function updateSensorFeeds(force = false) {
    const feeds = state.sim.getSensorFeeds();
    if (state.sensorPublisher) {
        state.sensorPublisher.publish(feeds);
    }
    const signature = feeds.map((feed) => {
        return `${feed.id}:${feed.status}:${feed.t}:${feed.summary}`;
    }).join("|");
    if (!force && signature === state.lastSensorFeedSignature) {
        return;
    }
    state.lastSensorFeedSignature = signature;
    els.sensorFeeds.replaceChildren(...feeds.map((feed) => renderSensorFeed(feed)));
}

function renderSensorFeed(feed) {
    const card = document.createElement("article");
    card.className = `sensor-card ${feed.status}`;

    const header = document.createElement("header");
    const title = document.createElement("strong");
    title.textContent = feed.name;
    const meta = document.createElement("span");
    meta.textContent = `${feed.type} · ${feed.status} · ${feed.hz || 0} Hz`;
    header.append(title, meta);
    card.appendChild(header);

    const summary = document.createElement("p");
    summary.textContent = feed.summary;
    card.appendChild(summary);

    if (feed.displayType === "position" && feed.data) {
        card.appendChild(renderKeyValueGrid([
            ["pos", fmtVec(feed.data.pos)],
            ["vel", fmtVec(feed.data.velocity)]
        ]));
    }
    else if (feed.displayType === "motion" && feed.data) {
        card.appendChild(renderKeyValueGrid([
            ["acc", fmtVec(feed.data.acceleration)],
            ["ang vel", fmtVec(feed.data.angularVel)],
            ["orient", fmtVec(feed.data.orientation)]
        ]));
    }
    else if (feed.displayType === "image" && feed.data && feed.data.imageDataUrl) {
        const img = document.createElement("img");
        img.className = "sensor-image";
        img.alt = `${feed.name} output`;
        img.src = feed.data.imageDataUrl;
        card.appendChild(img);
    }
    else if (feed.displayType === "pointCloud" && feed.data) {
        const canvas = document.createElement("canvas");
        canvas.className = "lidar-range";
        canvas.width = feed.data.width;
        canvas.height = feed.data.height;
        drawLidarRange(canvas, feed.data);
        card.appendChild(canvas);
        card.appendChild(renderKeyValueGrid([
            ["rays", String(feed.data.width * feed.data.height)],
            ["hits", String(feed.data.hitCount || 0)],
            ["min", feed.data.minRange ? `${feed.data.minRange.toFixed(2)} m` : "n/a"]
        ]));
    }

    return card;
}

function renderKeyValueGrid(rows) {
    const grid = document.createElement("div");
    grid.className = "sensor-kv";
    rows.forEach(([label, value]) => {
        const key = document.createElement("span");
        key.textContent = label;
        const val = document.createElement("strong");
        val.textContent = value;
        grid.append(key, val);
    });
    return grid;
}

function drawLidarRange(canvas, data) {
    const ctx = canvas.getContext("2d");
    const image = ctx.createImageData(data.width, data.height);
    const maxDistance = data.maxDistance || 1;

    data.ranges.forEach((range, idx) => {
        const hit = range < maxDistance;
        const near = 1 - Math.min(range / maxDistance, 1);
        image.data[idx * 4] = hit ? 45 + near * 210 : 16;
        image.data[idx * 4 + 1] = hit ? 120 + near * 120 : 32;
        image.data[idx * 4 + 2] = hit ? 230 - near * 120 : 45;
        image.data[idx * 4 + 3] = 255;
    });

    ctx.putImageData(image, 0, 0);
}

function fmtVec(v) {
    return `${v.x.toFixed(2)}, ${v.y.toFixed(2)}, ${v.z.toFixed(2)}`;
}

function vecMag(v) {
    return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

function fmtClock(time) {
    const daySeconds = ((Math.floor(time) % 86400) + 86400) % 86400;
    const hours = Math.floor(daySeconds / 3600);
    const minutes = Math.floor((daySeconds % 3600) / 60);
    const seconds = daySeconds % 60;
    return `${pad2(hours)}:${pad2(minutes)}:${pad2(seconds)}`;
}

function pad2(value) {
    return String(value).padStart(2, "0");
}

init();
