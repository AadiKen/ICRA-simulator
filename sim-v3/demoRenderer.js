import * as THREE from "./vendor/three.module.js";
import {sensorWorldPose} from "./schema.js?v=26";

const CAMERA_PHASES = {
    landing: {pos: new THREE.Vector3(55, 54, 94), target: new THREE.Vector3(34, 0, 28)},
    boat: {pos: new THREE.Vector3(22, 14, 30), target: new THREE.Vector3(12, 1, 12)},
    obstacles: {pos: new THREE.Vector3(45, 76, 54), target: new THREE.Vector3(45, 0, 35)},
    environment: {pos: new THREE.Vector3(78, 44, 84), target: new THREE.Vector3(45, 0, 35)},
    route: {pos: new THREE.Vector3(45, 84, 35), target: new THREE.Vector3(45, 0, 35)},
    run: {pos: new THREE.Vector3(-13, 11, -18), target: new THREE.Vector3(0, 0, 0)},
    results: {pos: new THREE.Vector3(45, 78, 78), target: new THREE.Vector3(45, 0, 35)}
};

function makeGradientCanvas(top, bottom, middle = null) {
    const canvas = document.createElement("canvas");
    canvas.width = 16;
    canvas.height = 16;
    const ctx = canvas.getContext("2d");
    const gradient = ctx.createLinearGradient(0, 0, 0, canvas.height);
    gradient.addColorStop(0, top);
    if (middle) gradient.addColorStop(0.58, middle);
    gradient.addColorStop(1, bottom);
    ctx.fillStyle = gradient;
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    return canvas;
}

function makeSkyEnvironment() {
    const side = makeGradientCanvas("#78b8ff", "#d7efff", "#a8d8ff");
    const top = makeGradientCanvas("#4f96f0", "#8fc8ff");
    const bottom = makeGradientCanvas("#0a3544", "#061a22");
    const texture = new THREE.CubeTexture([side, side, top, bottom, side, side]);
    texture.colorSpace = THREE.SRGBColorSpace;
    texture.needsUpdate = true;
    return texture;
}

export class DemoRenderer {
    constructor(container, config) {
        this.container = container;
        this.config = config;
        this.sim = null;
        this.phase = "landing";
        this.showSensors = true;
        this.renderMode = null;
        this.waterResolution = this.isCinematicMode() ? 96 : 48;
        this.tmpColor = new THREE.Color();
        this.sensorMarkers = new Map();
        this.sensorLines = new Map();
        this.vehicleOrbit = {azimuth: 0, elevation: 0, radius: 0};
        this.runOrbit = {azimuthOffset: 0, elevation: 0.58, radius: 15.5};

        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x071116);
        this.scene.fog = new THREE.Fog(0x071116, 90, 210);

        this.camera = new THREE.PerspectiveCamera(46, 1, 0.1, 650);
        this.camera.position.copy(CAMERA_PHASES.landing.pos);
        this.cameraTarget = CAMERA_PHASES.landing.target.clone();

        this.renderer = new THREE.WebGLRenderer({antialias: true});
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        this.renderer.outputColorSpace = THREE.SRGBColorSpace;
        this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
        this.renderer.toneMappingExposure = 1.08;
        this.container.appendChild(this.renderer.domElement);

        this.buildLights();
        this.buildSky();
        this.buildWorld();
        this.applyRenderMode(true);
        this.resize();
        this.resizeHandler = () => this.resize();
        window.addEventListener("resize", this.resizeHandler);
    }

    dispose() {
        window.removeEventListener("resize", this.resizeHandler);
        this.scene.traverse((object) => {
            object.geometry?.dispose();
            const materials = Array.isArray(object.material) ? object.material : [object.material];
            materials.filter(Boolean).forEach((material) => {
                Object.values(material).forEach((value) => {
                    if (value?.isTexture) value.dispose();
                });
                material.dispose();
            });
        });
        if (this.scene.background?.isTexture) this.scene.background.dispose();
        if (this.scene.environment?.isTexture) this.scene.environment.dispose();
        this.renderer.dispose();
        this.renderer.forceContextLoss();
    }

    buildLights() {
        this.ambientLight = new THREE.HemisphereLight(0xa7dfff, 0x153324, 1.8);
        this.scene.add(this.ambientLight);

        this.sunLight = new THREE.DirectionalLight(0xffffff, 2.4);
        this.sunLight.position.set(32, 72, 18);
        this.sunLight.castShadow = true;
        this.sunLight.shadow.mapSize.set(1536, 1536);
        this.sunLight.shadow.camera.near = 0.5;
        this.sunLight.shadow.camera.far = 220;
        this.sunLight.shadow.camera.left = -85;
        this.sunLight.shadow.camera.right = 85;
        this.sunLight.shadow.camera.top = 85;
        this.sunLight.shadow.camera.bottom = -85;
        this.scene.add(this.sunLight);
        this.scene.add(this.sunLight.target);
    }

    buildSky() {
        const geometry = new THREE.SphereGeometry(480, 32, 16);
        this.skyMaterial = new THREE.ShaderMaterial({
            side: THREE.BackSide,
            depthWrite: false,
            depthTest: false,
            uniforms: {
                topColor: {value: new THREE.Color(0x5da6f5)},
                horizonColor: {value: new THREE.Color(0xc8e9ff)},
                bottomColor: {value: new THREE.Color(0x061a22)}
            },
            vertexShader: `
                varying vec3 vWorldPosition;
                void main() {
                    vec4 worldPosition = modelMatrix * vec4(position, 1.0);
                    vWorldPosition = worldPosition.xyz;
                    gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
                }
            `,
            fragmentShader: `
                varying vec3 vWorldPosition;
                uniform vec3 topColor;
                uniform vec3 horizonColor;
                uniform vec3 bottomColor;
                void main() {
                    float h = normalize(vWorldPosition).y;
                    vec3 upper = mix(horizonColor, topColor, smoothstep(-0.04, 0.78, h));
                    vec3 color = mix(bottomColor, upper, smoothstep(-0.5, 0.08, h));
                    gl_FragColor = vec4(color, 1.0);
                }
            `
        });
        this.skyDome = new THREE.Mesh(geometry, this.skyMaterial);
        this.skyDome.renderOrder = -1000;
        this.scene.add(this.skyDome);
        try {
            this.skyEnvironment = makeSkyEnvironment();
        }
        catch (error) {
            console.warn("Sky reflection environment failed; continuing with visible sky only.", error);
            this.skyEnvironment = null;
        }
    }

    buildWorld() {
        this.bounds = {width: 90, height: 70};
        this.buildWater();
        this.buildBounds();
        this.buildBoat();
        this.obstacleGroup = new THREE.Group();
        this.scene.add(this.obstacleGroup);
        this.zoneGroup = new THREE.Group();
        this.scene.add(this.zoneGroup);
        this.waypointGroup = new THREE.Group();
        this.scene.add(this.waypointGroup);
        this.sensorGroup = new THREE.Group();
        this.scene.add(this.sensorGroup);
        this.trajectoryLine = null;
        this.pathLine = null;
        this.refreshConfigVisuals();
    }

    buildWater() {
        const geometry = new THREE.PlaneGeometry(
            this.bounds.width,
            this.bounds.height,
            this.waterResolution,
            this.waterResolution
        );
        geometry.rotateX(-Math.PI / 2);
        geometry.translate(this.bounds.width / 2, 0, this.bounds.height / 2);
        const colors = new Float32Array(geometry.attributes.position.count * 3);
        geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
        this.waterMesh = new THREE.Mesh(geometry, this.createWaterMaterial());
        this.waterMesh.receiveShadow = true;
        this.waterMesh.userData.lidarTarget = true;
        this.waterMesh.userData.type = "water";
        this.waterMesh.userData.label = "water";
        this.scene.add(this.waterMesh);
    }

    createWaterMaterial() {
        if (!this.isCinematicMode()) {
            return new THREE.MeshStandardMaterial({
                vertexColors: true,
                transparent: true,
                opacity: 0.86,
                roughness: 0.64,
                metalness: 0,
                side: THREE.DoubleSide
            });
        }

        if (!this.waterNormalMap) {
            this.waterNormalMap = new THREE.TextureLoader().load("./vendor/waternormals.jpg");
            this.waterNormalMap.wrapS = THREE.RepeatWrapping;
            this.waterNormalMap.wrapT = THREE.RepeatWrapping;
            this.waterNormalMap.repeat.set(7.5, 7.5);
        }

        return new THREE.MeshPhysicalMaterial({
            vertexColors: true,
            color: 0x0b5f73,
            transparent: true,
            opacity: 0.88,
            roughness: 0.18,
            metalness: 0,
            clearcoat: 1,
            clearcoatRoughness: 0.08,
            reflectivity: 0.72,
            envMapIntensity: 0.8,
            normalMap: this.waterNormalMap,
            normalScale: new THREE.Vector2(0.18, 0.18),
            side: THREE.DoubleSide
        });
    }

    buildBounds() {
        const shape = [
            new THREE.Vector3(0, 0.08, 0),
            new THREE.Vector3(this.bounds.width, 0.08, 0),
            new THREE.Vector3(this.bounds.width, 0.08, this.bounds.height),
            new THREE.Vector3(0, 0.08, this.bounds.height),
            new THREE.Vector3(0, 0.08, 0)
        ];
        this.boundsLine = new THREE.Line(
            new THREE.BufferGeometry().setFromPoints(shape),
            new THREE.LineBasicMaterial({color: 0x9be7ff, transparent: true, opacity: 0.7})
        );
        this.scene.add(this.boundsLine);
    }

    buildBoat() {
        const dims = this.config.boat.dimensions;
        const group = new THREE.Group();
        const hull = new THREE.Mesh(
            new THREE.BoxGeometry(dims.x, dims.y, dims.z),
            new THREE.MeshStandardMaterial({color: 0xf4f7f8, roughness: 0.45})
        );
        hull.castShadow = true;
        group.add(hull);

        const marker = new THREE.Mesh(
            new THREE.BoxGeometry(dims.x * 0.12, dims.y * 0.1, dims.z * 0.72),
            new THREE.MeshStandardMaterial({color: 0xff453a, roughness: 0.36})
        );
        marker.position.set(0, dims.y * 0.58, dims.z * 0.18);
        group.add(marker);

        const mast = new THREE.Mesh(
            new THREE.CylinderGeometry(0.045, 0.045, dims.y * 2.7, 12),
            new THREE.MeshStandardMaterial({color: 0x101a20, roughness: 0.5})
        );
        mast.position.y = dims.y * 1.25;
        group.add(mast);

        this.boat = group;
        this.scene.add(group);
    }

    attachSimulator(sim) {
        this.sim = sim;
        this.rebuildSensorMarkers();
        this.updateSensors();
    }

    setPhase(phase) {
        this.phase = phase;
    }

    setShowSensors(showSensors) {
        this.showSensors = showSensors;
    }

    setConfig(config) {
        this.config = config;
        this.applyRenderMode();
        this.refreshConfigVisuals();
        this.rebuildSensorMarkers();
    }

    isCinematicMode() {
        return this.config && this.config.renderMode === "cinematic";
    }

    applyRenderMode(force = false) {
        const nextMode = this.isCinematicMode() ? "cinematic" : "efficient";
        if (!force && this.renderMode === nextMode) return;
        this.renderMode = nextMode;

        const desiredResolution = nextMode === "cinematic" ? 96 : 48;
        const rebuildGeometry = this.waterMesh && this.waterResolution !== desiredResolution;
        this.waterResolution = desiredResolution;

        if (this.skyDome) {
            this.skyDome.visible = nextMode === "cinematic";
        }
        this.scene.environment = nextMode === "cinematic" && this.skyEnvironment ? this.skyEnvironment : null;

        if (rebuildGeometry) {
            this.scene.remove(this.waterMesh);
            this.waterMesh.geometry.dispose();
            this.waterMesh.material.dispose();
            this.buildWater();
        }
        else if (this.waterMesh) {
            this.waterMesh.material.dispose();
            this.waterMesh.material = this.createWaterMaterial();
        }
    }

    refreshConfigVisuals() {
        this.refreshObstacles();
        this.refreshZones();
        this.refreshWaypoints();
        this.updateVisibilityForPhase();
    }

    refreshObstacles() {
        if (!this.obstacleGroup) return;
        this.obstacleGroup.clear();
        this.config.obstacles.forEach((obs, idx) => {
            const mesh = new THREE.Mesh(
                new THREE.CylinderGeometry(obs.r, obs.r, 1.5, 28),
                new THREE.MeshStandardMaterial({
                    color: obs.type === "fountain" ? 0x6fd6ff : 0xd35444,
                    roughness: 0.58
                })
            );
            mesh.position.set(obs.x, 0.75, obs.z);
            mesh.castShadow = true;
            mesh.receiveShadow = true;
            mesh.userData.lidarTarget = true;
            mesh.userData.type = "obstacle";
            mesh.userData.label = `obstacle_${idx}`;
            this.obstacleGroup.add(mesh);
        });
    }

    refreshZones() {
        if (!this.zoneGroup) return;
        this.zoneGroup.clear();
        const zones = [...(this.config.zones || [])];
        if (this.config.zonePreview) zones.push(this.config.zonePreview);
        zones.forEach((zone, idx) => {
            const group = this.buildZoneMesh(zone, idx);
            if (group) this.zoneGroup.add(group);
        });
        const draft = this.config.zoneDraft || [];
        if (draft.length) {
            const color = draft[0].type === "favoredZone" ? 0x42e4ae : 0xff6b5f;
            const points = draft.map((point) => new THREE.Vector3(point.x, 1.18, point.z));
            const line = new THREE.Line(
                new THREE.BufferGeometry().setFromPoints(points),
                new THREE.LineBasicMaterial({color, transparent: true, opacity: 0.92})
            );
            this.zoneGroup.add(line);
            draft.forEach((point, idx) => {
                const dot = new THREE.Mesh(
                    new THREE.SphereGeometry(idx === 0 ? 0.34 : 0.24, 16, 10),
                    new THREE.MeshBasicMaterial({color})
                );
                dot.position.set(point.x, 1.2, point.z);
                this.zoneGroup.add(dot);
            });
        }
    }

    buildZoneMesh(zone, idx) {
        if (!zone.points || zone.points.length < 3) return null;
        const color = zone.type === "favoredZone" ? 0x42e4ae : 0xff6b5f;
        const center = zone.points.reduce((sum, point) => ({
            x: sum.x + point.x / zone.points.length,
            z: sum.z + point.z / zone.points.length
        }), {x: 0, z: 0});
        const vertices = [];
        for (let i = 0; i < zone.points.length; i += 1) {
            const a = zone.points[i];
            const b = zone.points[(i + 1) % zone.points.length];
            vertices.push(center.x, 1.05 + idx * 0.02, center.z, a.x, 1.05 + idx * 0.02, a.z, b.x, 1.05 + idx * 0.02, b.z);
        }
        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute("position", new THREE.Float32BufferAttribute(vertices, 3));
        geometry.computeVertexNormals();
        const fill = new THREE.Mesh(
            geometry,
            new THREE.MeshBasicMaterial({color, transparent: true, opacity: 0.26, side: THREE.DoubleSide})
        );
        const borderPoints = zone.points.map((point) => new THREE.Vector3(point.x, 1.12 + idx * 0.02, point.z));
        borderPoints.push(borderPoints[0].clone());
        const border = new THREE.Line(
            new THREE.BufferGeometry().setFromPoints(borderPoints),
            new THREE.LineBasicMaterial({color, transparent: true, opacity: 0.9})
        );
        const group = new THREE.Group();
        group.add(fill, border);
        group.userData.type = zone.type;
        return group;
    }

    refreshWaypoints() {
        if (!this.waypointGroup) return;
        this.waypointGroup.clear();
        this.config.waypoints.forEach((wp, idx) => {
            const group = new THREE.Group();
            const dot = new THREE.Mesh(
                new THREE.SphereGeometry(0.58, 18, 12),
                new THREE.MeshStandardMaterial({color: 0x42e4ae, emissive: 0x0a2a1e})
            );
            const ring = new THREE.Mesh(
                new THREE.TorusGeometry(this.config.goalTolerance, 0.035, 8, 56),
                new THREE.MeshBasicMaterial({color: 0x42e4ae, transparent: true, opacity: 0.72})
            );
            ring.rotation.x = Math.PI / 2;
            group.add(dot, ring);
            group.position.set(wp.x, 0.42 + idx * 0.02, wp.z);
            this.waypointGroup.add(group);
        });
        this.updatePathLine();
    }

    updatePathLine() {
        if (this.pathLine) {
            this.scene.remove(this.pathLine);
            this.pathLine.geometry.dispose();
        }
        if (this.config.waypoints.length < 2) {
            this.pathLine = null;
            return;
        }
        const points = this.config.waypoints.map((wp) => new THREE.Vector3(wp.x, 0.32, wp.z));
        this.pathLine = new THREE.Line(
            new THREE.BufferGeometry().setFromPoints(points),
            new THREE.LineBasicMaterial({color: 0x42e4ae, transparent: true, opacity: 0.84})
        );
        this.scene.add(this.pathLine);
    }

    rebuildSensorMarkers() {
        this.sensorGroup.clear();
        this.sensorMarkers.clear();
        this.sensorLines.clear();
        const markerGeometry = new THREE.SphereGeometry(0.16, 12, 8);
        this.currentSensors().forEach((sensor, idx) => {
            const key = this.sensorKey(sensor, idx);
            const color = sensor.type === "lidar"
                ? 0xffd166
                : sensor.type === "dayCam" || sensor.type === "nightCam"
                    ? 0x8eddf4
                    : sensor.type === "imu"
                        ? 0xff7a90
                        : 0xb5f48e;
            const marker = new THREE.Mesh(markerGeometry, new THREE.MeshBasicMaterial({color}));
            const line = new THREE.Line(
                new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
                new THREE.LineBasicMaterial({color})
            );
            this.sensorGroup.add(marker, line);
            this.sensorMarkers.set(key, marker);
            this.sensorLines.set(key, line);
        });
    }

    currentSensors() {
        if (this.sim) return this.sim.state.sensors.sensors;
        return (this.config.sensors || []).map((sensor, idx) => ({
            ...sensor,
            id: `${sensor.type}_${idx}`
        }));
    }

    sensorKey(sensor, idx = 0) {
        return sensor.id || sensor.name || `${sensor.type}_${idx}`;
    }

    update(time = 0) {
        this.syncToSimulation(time);
        this.updateCamera();
        this.renderer.render(this.scene, this.camera);
    }

    syncToSimulation(time = 0) {
        const simTime = this.sim ? this.sim.state.time : 43200 + time;
        this.updateLighting(simTime);
        this.updateWater(simTime);

        if (this.sim) {
            this.updateBoatFromState(this.sim.state.boat);
            this.updateWaypointState();
            this.updateTrajectory();
            this.updateSensors();
        }
        else {
            this.boat.position.set(this.config.boat.startPos.x, 0.42, this.config.boat.startPos.z);
            this.boat.rotation.set(0, 0, 0);
            this.updateVisibilityForPhase();
            this.updateSensors();
        }
    }

    updateLighting(time) {
        const weather = this.config.weather || "clear";
        const daySeconds = ((time % 86400) + 86400) % 86400;
        const dayFraction = daySeconds / 86400;
        const sunAngle = dayFraction * Math.PI * 2 - Math.PI / 2;
        const altitude = Math.sin(sunAngle);
        const daylight = THREE.MathUtils.clamp((altitude + 0.12) / 0.55, 0, 1);
        const weatherLight = weather === "rainy" ? 0.62 : weather === "foggy" ? 0.78 : 1;
        const fogNear = weather === "foggy" ? 32 : weather === "rainy" ? 62 : 90;
        const fogFar = weather === "foggy" ? 118 : weather === "rainy" ? 155 : 210;
        const fogColor = weather === "foggy" ? 0xaebec2 : weather === "rainy" ? 0x51666d : 0x071116;
        this.scene.fog.color.setHex(fogColor);
        this.scene.fog.near = fogNear;
        this.scene.fog.far = fogFar;
        this.scene.background.setHex(fogColor);
        this.sunLight.position.set(34 + Math.cos(sunAngle) * 72, Math.max(altitude, 0.08) * 62, 18);
        this.sunLight.target.position.set(45, 0, 35);
        this.sunLight.target.updateMatrixWorld();
        this.sunLight.intensity = (0.18 + daylight * 2.3) * weatherLight;
        this.sunLight.color.copy(new THREE.Color(0xffffff).lerp(new THREE.Color(0xffc889), (1 - daylight) * 0.42));
        this.ambientLight.intensity = (0.52 + daylight * 1.16) * (weather === "foggy" ? 1.05 : weatherLight);
        this.renderer.toneMappingExposure = weather === "foggy" ? 1.04 : 0.98 + daylight * 0.24;
        if (this.skyMaterial && this.isCinematicMode()) {
            const nightTop = new THREE.Color(0x06111e);
            const dayTop = weather === "rainy" ? new THREE.Color(0x627d89) : weather === "foggy" ? new THREE.Color(0xb4c6cc) : new THREE.Color(0x5ca7f7);
            const dayHorizon = weather === "rainy" ? new THREE.Color(0x879aa1) : weather === "foggy" ? new THREE.Color(0xd3dcdd) : new THREE.Color(0xd1edff);
            const duskHorizon = new THREE.Color(0xffbf7a);
            const bottom = weather === "rainy" ? new THREE.Color(0x152b31) : weather === "foggy" ? new THREE.Color(0x879497) : new THREE.Color(0x06202a);
            const twilight = THREE.MathUtils.clamp((altitude + 0.22) / 0.34, 0, 1) * (1 - daylight);
            this.skyMaterial.uniforms.topColor.value.copy(nightTop).lerp(dayTop, daylight);
            this.skyMaterial.uniforms.horizonColor.value.copy(dayHorizon).lerp(duskHorizon, twilight * 0.65);
            this.skyMaterial.uniforms.bottomColor.value.copy(bottom);
        }
        if (this.waterMesh && this.waterMesh.material) {
            this.waterMesh.material.envMapIntensity = weather === "rainy" ? 0.72 : weather === "foggy" ? 0.48 : 1.05;
        }
    }

    updateWater(time) {
        const waterField = this.sim ? this.sim.envModel.waterField : null;
        const posAttr = this.waterMesh.geometry.attributes.position;
        const colorAttr = this.waterMesh.geometry.attributes.color;
        if (this.waterNormalMap) {
            this.waterNormalMap.offset.set((time * 0.012) % 1, (time * 0.018) % 1);
        }
        for (let i = 0; i < posAttr.count; i += 1) {
            const x = posAttr.getX(i);
            const z = posAttr.getZ(i);
            const surfaceHeight = waterField
                ? waterField.sampleAt({x, y: 0, z}, time).surfaceHeight
                : Math.sin(x * 0.28 + time * 0.8) * 0.08 + Math.cos(z * 0.22 - time * 0.6) * 0.06;
            posAttr.setY(i, surfaceHeight);
            const h = THREE.MathUtils.clamp((surfaceHeight + 0.55) / 1.1, 0, 1);
            if (this.isCinematicMode()) {
                const glint = Math.pow(h, 2.4);
                this.tmpColor.setRGB(0.015 + h * 0.07 + glint * 0.1, 0.23 + h * 0.24 + glint * 0.16, 0.34 + h * 0.33 + glint * 0.23);
            }
            else {
                this.tmpColor.setRGB(0.05 + h * 0.1, 0.32 + h * 0.2, 0.46 + h * 0.26);
            }
            colorAttr.setXYZ(i, this.tmpColor.r, this.tmpColor.g, this.tmpColor.b);
        }
        posAttr.needsUpdate = true;
        colorAttr.needsUpdate = true;
        this.waterMesh.geometry.computeVertexNormals();
    }

    updateBoatFromState(boatState) {
        const draft = Math.min(Math.max(boatState.draft || boatState.dimensions.y * 0.25, 0), boatState.dimensions.y);
        const hullCenterAboveWaterline = boatState.dimensions.y * 0.5 - draft;
        this.boat.position.set(boatState.pos.x, boatState.pos.y + hullCenterAboveWaterline, boatState.pos.z);
        this.boat.rotation.set(boatState.orientation.x, boatState.orientation.y, boatState.orientation.z);
    }

    updateWaypointState() {
        this.updateVisibilityForPhase();
        const activeIdx = this.sim.state.goal.waypointIdx;
        this.waypointGroup.children.forEach((group, idx) => {
            group.visible = idx >= activeIdx;
            group.children.forEach((child) => {
                if (child.material && child.material.color) {
                    child.material.color.set(idx === activeIdx ? 0x42e4ae : 0x2c95c9);
                }
            });
        });
    }

    updateVisibilityForPhase() {
        const showRoute = this.phase === "route" || this.phase === "run" || this.phase === "results";
        if (this.waypointGroup) this.waypointGroup.visible = showRoute;
        if (this.pathLine) this.pathLine.visible = showRoute;
        if (this.trajectoryLine) this.trajectoryLine.visible = this.phase === "run" || this.phase === "results";
    }

    updateTrajectory() {
        const logs = this.sim.logs.boatStates;
        if (logs.length < 2) return;
        const stride = Math.max(1, Math.floor(logs.length / 260));
        const points = [];
        for (let i = 0; i < logs.length; i += stride) {
            const p = logs[i].pos;
            points.push(new THREE.Vector3(p.x, p.y + 0.12, p.z));
        }
        if (!this.trajectoryLine) {
            this.trajectoryLine = new THREE.Line(
                new THREE.BufferGeometry(),
                new THREE.LineBasicMaterial({color: 0xffd166, transparent: true, opacity: 0.9})
            );
            this.scene.add(this.trajectoryLine);
        }
        const oldGeometry = this.trajectoryLine.geometry;
        this.trajectoryLine.geometry = new THREE.BufferGeometry().setFromPoints(points);
        oldGeometry.dispose();
    }

    updateSensors() {
        const visiblePhase = this.phase === "boat" || this.phase === "run" || this.phase === "results";
        this.sensorGroup.visible = this.showSensors && visiblePhase;
        const boatState = this.sim
            ? this.sim.state.boat
            : {
                pos: this.config.boat.startPos,
                orientation: {x: 0, y: this.boat.rotation.y, z: 0}
            };
        const feeds = this.sim ? this.sim.getSensorFeeds() : [];
        const liveById = new Map(feeds.map((feed) => [feed.id, feed]));
        this.currentSensors().forEach((sensor, idx) => {
            const key = this.sensorKey(sensor, idx);
            const marker = this.sensorMarkers.get(key);
            const line = this.sensorLines.get(key);
            if (!marker || !line) return;
            const liveFeed = liveById.get(key);
            const pose = liveFeed && liveFeed.pose ? liveFeed.pose : sensorWorldPose(sensor, boatState);
            const editingSensorId = this.phase === "boat" ? this.config.editingSensorId : null;
            const visible = this.showSensors && visiblePhase && pose && (!editingSensorId || key === editingSensorId) && (!this.sim || (liveFeed && liveFeed.active));
            marker.visible = visible;
            line.visible = visible;
            if (!visible) return;
            const pos = pose.position;
            const forward = pose.forward;
            const start = new THREE.Vector3(pos.x, pos.y, pos.z);
            const end = new THREE.Vector3(pos.x + forward.x * 2.4, pos.y + forward.y * 2.4, pos.z + forward.z * 2.4);
            marker.position.copy(start);
            line.geometry.setFromPoints([start, end]);
        });
    }

    updateCamera() {
        const phaseTarget = CAMERA_PHASES[this.phase] || CAMERA_PHASES.landing;
        let desiredPos = phaseTarget.pos;
        let desiredTarget = phaseTarget.target;
        if (this.phase === "boat") {
            desiredTarget = new THREE.Vector3(this.config.boat.startPos.x, 1, this.config.boat.startPos.z);
            if (this.vehicleOrbit.radius <= 0) {
                const offset = CAMERA_PHASES.boat.pos.clone().sub(CAMERA_PHASES.boat.target);
                this.vehicleOrbit.radius = offset.length();
                this.vehicleOrbit.azimuth = Math.atan2(offset.x, offset.z);
                this.vehicleOrbit.elevation = Math.asin(offset.y / this.vehicleOrbit.radius);
            }
            desiredPos = desiredTarget.clone().add(new THREE.Vector3(
                Math.sin(this.vehicleOrbit.azimuth) * Math.cos(this.vehicleOrbit.elevation) * this.vehicleOrbit.radius,
                Math.sin(this.vehicleOrbit.elevation) * this.vehicleOrbit.radius,
                Math.cos(this.vehicleOrbit.azimuth) * Math.cos(this.vehicleOrbit.elevation) * this.vehicleOrbit.radius
            ));
        }
        if (this.phase === "run" && this.sim) {
            const boat = this.boat.position;
            const heading = this.sim.state.boat.heading || 0;
            desiredTarget = boat.clone().add(new THREE.Vector3(0, 1.2, 0));
            const azimuth = heading + Math.PI + this.runOrbit.azimuthOffset;
            desiredPos = desiredTarget.clone().add(new THREE.Vector3(
                Math.sin(azimuth) * Math.cos(this.runOrbit.elevation) * this.runOrbit.radius,
                Math.sin(this.runOrbit.elevation) * this.runOrbit.radius,
                Math.cos(azimuth) * Math.cos(this.runOrbit.elevation) * this.runOrbit.radius
            ));
        }
        this.camera.position.lerp(desiredPos, 0.045);
        this.cameraTarget.lerp(desiredTarget, 0.055);
        this.camera.lookAt(this.cameraTarget);
    }

    panVehicleCamera(dx, dy) {
        if (this.phase !== "boat") return;
        this.vehicleOrbit.azimuth -= dx * 0.008;
        this.vehicleOrbit.elevation = THREE.MathUtils.clamp(this.vehicleOrbit.elevation + dy * 0.006, 0.12, 1.12);
    }

    panRunCamera(dx, dy) {
        if (this.phase !== "run") return;
        this.runOrbit.azimuthOffset -= dx * 0.008;
        this.runOrbit.elevation = THREE.MathUtils.clamp(this.runOrbit.elevation + dy * 0.006, 0.12, 1.18);
    }

    resize() {
        const rect = this.container.getBoundingClientRect();
        this.camera.aspect = rect.width / Math.max(rect.height, 1);
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(rect.width, rect.height, false);
    }

    screenToWorld(clientX, clientY) {
        const rect = this.renderer.domElement.getBoundingClientRect();
        const x = ((clientX - rect.left) / rect.width) * 2 - 1;
        const y = -((clientY - rect.top) / rect.height) * 2 + 1;
        const raycaster = new THREE.Raycaster();
        raycaster.setFromCamera(new THREE.Vector2(x, y), this.camera);
        const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
        const point = new THREE.Vector3();
        raycaster.ray.intersectPlane(plane, point);
        if (!Number.isFinite(point.x) || !Number.isFinite(point.z)) return null;
        return {
            x: THREE.MathUtils.clamp(point.x, 2, this.bounds.width - 2),
            z: THREE.MathUtils.clamp(point.z, 2, this.bounds.height - 2)
        };
    }

    getSensorTargets() {
        return [
            ...this.obstacleGroup.children
        ].filter(Boolean);
    }
}
