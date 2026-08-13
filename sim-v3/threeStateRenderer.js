import * as THREE from "./vendor/three.module.js";

const WATER_MODES = new Set(["presentation", "height", "velocity", "normal", "hull", "physics"]);

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

export class ThreeStateRenderer {
    constructor(container, sim, options = {}) {
        this.container = container;
        this.sim = sim;
        this.mode = options.mode || "physics";
        this.waterResolution = options.waterResolution || 64;
        this.clock = new THREE.Clock();
        this.tmpColor = new THREE.Color();
        this.hullMarkers = [];
        this.hullLines = [];
        this.flowArrows = [];
        this.pathLine = null;
        this.trajectoryLine = null;
        this.daySkyColor = new THREE.Color(0x8fc9ff);
        this.duskSkyColor = new THREE.Color(0x1d3140);
        this.nightSkyColor = new THREE.Color(0x071016);

        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x071016);
        this.scene.fog = new THREE.Fog(0x071016, 80, 180);

        this.camera = new THREE.PerspectiveCamera(48, 1, 0.1, 500);
        this.camera.position.set(48, 54, 82);
        this.camera.lookAt(45, 0, 35);

        this.renderer = new THREE.WebGLRenderer({antialias: true});
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        this.renderer.outputColorSpace = THREE.SRGBColorSpace;
        this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
        this.renderer.toneMappingExposure = 1.08;
        this.container.appendChild(this.renderer.domElement);

        this.raycaster = new THREE.Raycaster();
        this.pointer = new THREE.Vector2();
        this.dragging = false;
        this.lastPointer = {x: 0, y: 0};
        this.cameraTarget = new THREE.Vector3(45, 0, 35);
        this.cameraSpherical = new THREE.Spherical(110, Math.PI * 0.32, Math.PI * 0.22);

        this.buildLights();
        this.buildSky();
        this.buildWorld();
        this.bindCameraControls();
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
        this.ambientLight = new THREE.HemisphereLight(0x7fb7ff, 0x102018, 1.8);
        this.scene.add(this.ambientLight);

        this.sunLight = new THREE.DirectionalLight(0xffffff, 2.2);
        this.sunLight.position.set(30, 55, 20);
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
        const geometry = new THREE.SphereGeometry(520, 32, 16);
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
        this.scene.environment = makeSkyEnvironment();
    }

    buildWorld() {
        const bounds = this.sim.state.env.bounds;
        this.buildWater(bounds);
        this.buildBounds(bounds);
        this.buildBoat();
        this.buildObstacles();
        this.buildWaypoints();
        this.buildHullDebug();
        this.buildFlowVectors(bounds);
        this.buildSensorDebug();
    }

    buildWater(bounds) {
        const geometry = new THREE.PlaneGeometry(
            bounds.width,
            bounds.height,
            this.waterResolution,
            this.waterResolution
        );
        geometry.rotateX(-Math.PI / 2);
        geometry.translate(bounds.width / 2, 0, bounds.height / 2);

        const colorCount = geometry.attributes.position.count;
        const colors = new Float32Array(colorCount * 3);
        geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));

        this.waterNormalMap = new THREE.TextureLoader().load("./vendor/waternormals.jpg");
        this.waterNormalMap.wrapS = THREE.RepeatWrapping;
        this.waterNormalMap.wrapT = THREE.RepeatWrapping;
        this.waterNormalMap.repeat.set(7.5, 7.5);

        const material = new THREE.MeshPhysicalMaterial({
            vertexColors: true,
            color: 0x0b5f73,
            transparent: true,
            opacity: 0.88,
            roughness: 0.18,
            metalness: 0.0,
            clearcoat: 1,
            clearcoatRoughness: 0.08,
            reflectivity: 0.72,
            envMapIntensity: 0.8,
            normalMap: this.waterNormalMap,
            normalScale: new THREE.Vector2(0.18, 0.18),
            side: THREE.DoubleSide
        });

        this.waterMesh = new THREE.Mesh(geometry, material);
        this.waterMesh.receiveShadow = true;
        this.waterMesh.userData.lidarTarget = true;
        this.waterMesh.userData.label = "water";
        this.waterMesh.userData.type = "water";
        this.scene.add(this.waterMesh);
    }

    buildBounds(bounds) {
        const points = [
            new THREE.Vector3(0, 0.08, 0),
            new THREE.Vector3(bounds.width, 0.08, 0),
            new THREE.Vector3(bounds.width, 0.08, bounds.height),
            new THREE.Vector3(0, 0.08, bounds.height),
            new THREE.Vector3(0, 0.08, 0)
        ];
        const geometry = new THREE.BufferGeometry().setFromPoints(points);
        const material = new THREE.LineBasicMaterial({color: 0x9be7ff});
        this.boundsLine = new THREE.Line(geometry, material);
        this.scene.add(this.boundsLine);

    }

    buildBoat() {
        const dims = this.sim.state.boat.dimensions;
        const group = new THREE.Group();
        const hull = new THREE.Mesh(
            new THREE.BoxGeometry(dims.x, dims.y, dims.z),
            new THREE.MeshStandardMaterial({color: 0xf2f4f7, roughness: 0.48})
        );
        hull.castShadow = true;
        hull.receiveShadow = true;
        group.add(hull);

        const bow = new THREE.Mesh(
            new THREE.ConeGeometry(dims.x * 0.5, dims.z * 0.35, 4),
            new THREE.MeshStandardMaterial({color: 0x2f8cff, roughness: 0.45})
        );
        bow.rotation.x = Math.PI / 2;
        bow.rotation.z = Math.PI / 4;
        bow.position.z = dims.z * 0.64;
        bow.castShadow = true;
        group.add(bow);

        const frontMarker = new THREE.Mesh(
            new THREE.ConeGeometry(dims.x * 0.22, dims.z * 0.5, 18),
            new THREE.MeshStandardMaterial({color: 0xff3b30, roughness: 0.35})
        );
        frontMarker.rotation.x = Math.PI / 2;
        frontMarker.position.set(0, dims.y * 0.62, dims.z * 0.78);
        frontMarker.castShadow = true;
        group.add(frontMarker);

        const centerline = new THREE.Mesh(
            new THREE.BoxGeometry(dims.x * 0.08, dims.y * 0.08, dims.z * 0.78),
            new THREE.MeshStandardMaterial({color: 0xff3b30, roughness: 0.42})
        );
        centerline.position.set(0, dims.y * 0.58, dims.z * 0.18);
        group.add(centerline);

        const mast = new THREE.Mesh(
            new THREE.CylinderGeometry(0.04, 0.04, dims.y * 2.6, 10),
            new THREE.MeshStandardMaterial({color: 0x111820})
        );
        mast.position.y = dims.y * 1.2;
        group.add(mast);

        this.boat = group;
        this.scene.add(group);
    }

    buildObstacles() {
        this.obstacles = this.sim.state.env.obstacles.map((obs, idx) => {
            const geometry = new THREE.CylinderGeometry(obs.r, obs.r, 1.4, 24);
            const material = new THREE.MeshStandardMaterial({
                color: obs.collision ? 0xff4b4b : 0xffb84b,
                roughness: 0.6
            });
            const mesh = new THREE.Mesh(geometry, material);
            mesh.position.set(obs.pos.x, 0.7, obs.pos.z);
            mesh.castShadow = true;
            mesh.receiveShadow = true;
            mesh.userData.lidarTarget = true;
            mesh.userData.label = `obstacle_${idx}`;
            mesh.userData.type = "obstacle";
            this.scene.add(mesh);
            return mesh;
        });
    }

    buildWaypoints() {
        this.waypoints = this.sim.state.goal.waypoints.map((wp, idx) => {
            const group = new THREE.Group();
            const sphere = new THREE.Mesh(
                new THREE.SphereGeometry(0.75, 18, 12),
                new THREE.MeshStandardMaterial({color: 0x56f39a, emissive: 0x102814})
            );
            const ring = new THREE.Mesh(
                new THREE.TorusGeometry(this.sim.state.goal.tolerance, 0.04, 8, 64),
                new THREE.MeshBasicMaterial({color: 0x56f39a})
            );
            ring.rotation.x = Math.PI / 2;
            group.add(sphere, ring);
            group.position.set(wp.x, 0.3 + idx * 0.02, wp.z);
            this.scene.add(group);
            return group;
        });
        this.updatePathLine();
    }

    buildHullDebug() {
        const markerGeometry = new THREE.SphereGeometry(0.25, 12, 8);
        const lineMaterial = new THREE.LineBasicMaterial({color: 0xffffff});

        for (let i = 0; i < 15; i++) {
            const marker = new THREE.Mesh(
                markerGeometry,
                new THREE.MeshBasicMaterial({color: 0xffffff})
            );
            const line = new THREE.Line(
                new THREE.BufferGeometry().setFromPoints([
                    new THREE.Vector3(),
                    new THREE.Vector3()
                ]),
                lineMaterial.clone()
            );
            this.scene.add(marker, line);
            this.hullMarkers.push(marker);
            this.hullLines.push(line);
        }
    }

    buildFlowVectors(bounds) {
        const material = new THREE.LineBasicMaterial({color: 0xa9ecff, transparent: true, opacity: 0.65});
        const spacing = Math.max(bounds.width, bounds.height) / 9;

        for (let x = spacing; x < bounds.width; x += spacing) {
            for (let z = spacing; z < bounds.height; z += spacing) {
                const line = new THREE.Line(
                    new THREE.BufferGeometry().setFromPoints([
                        new THREE.Vector3(),
                        new THREE.Vector3()
                    ]),
                    material.clone()
                );
                line.userData.base = new THREE.Vector3(x, 0, z);
                this.scene.add(line);
                this.flowArrows.push(line);
            }
        }
    }

    buildSensorDebug() {
        this.sensorMarkers = new Map();
        this.sensorLines = new Map();
        const markerGeometry = new THREE.SphereGeometry(0.14, 10, 8);

        this.sim.state.sensors.sensors.forEach((sensor) => {
            const marker = new THREE.Mesh(
                markerGeometry,
                new THREE.MeshBasicMaterial({color: sensor.type === "lidar" ? 0xffd166 : 0xa9ecff})
            );
            const line = new THREE.Line(
                new THREE.BufferGeometry().setFromPoints([
                    new THREE.Vector3(),
                    new THREE.Vector3()
                ]),
                new THREE.LineBasicMaterial({color: sensor.type === "lidar" ? 0xffd166 : 0xa9ecff})
            );
            marker.visible = false;
            line.visible = false;
            this.scene.add(marker, line);
            this.sensorMarkers.set(sensor.id || sensor.name, marker);
            this.sensorLines.set(sensor.id || sensor.name, line);
        });
    }

    bindCameraControls() {
        const canvas = this.renderer.domElement;

        canvas.addEventListener("pointerdown", (event) => {
            this.dragging = true;
            this.lastPointer.x = event.clientX;
            this.lastPointer.y = event.clientY;
            canvas.setPointerCapture(event.pointerId);
        });

        canvas.addEventListener("pointermove", (event) => {
            if (!this.dragging) return;
            const dx = event.clientX - this.lastPointer.x;
            const dy = event.clientY - this.lastPointer.y;
            this.lastPointer.x = event.clientX;
            this.lastPointer.y = event.clientY;

            this.cameraSpherical.theta -= dx * 0.006;
            this.cameraSpherical.phi = THREE.MathUtils.clamp(
                this.cameraSpherical.phi + dy * 0.004,
                0.18,
                Math.PI * 0.48
            );
            this.updateCamera();
        });

        canvas.addEventListener("pointerup", (event) => {
            this.dragging = false;
            canvas.releasePointerCapture(event.pointerId);
        });

        canvas.addEventListener("wheel", (event) => {
            event.preventDefault();
            this.cameraSpherical.radius = THREE.MathUtils.clamp(
                this.cameraSpherical.radius + event.deltaY * 0.08,
                35,
                180
            );
            this.updateCamera();
        }, {passive: false});

        this.updateCamera();
    }

    updateCamera() {
        const pos = new THREE.Vector3().setFromSpherical(this.cameraSpherical).add(this.cameraTarget);
        this.camera.position.copy(pos);
        this.camera.lookAt(this.cameraTarget);
    }

    resize() {
        const rect = this.container.getBoundingClientRect();
        this.camera.aspect = rect.width / Math.max(rect.height, 1);
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(rect.width, rect.height, false);
    }

    setMode(mode) {
        if (WATER_MODES.has(mode)) {
            this.mode = mode;
        }
    }

    update() {
        this.syncToSimulation();
        this.renderer.render(this.scene, this.camera);
    }

    syncToSimulation() {
        const state = this.sim.state;
        this.updateTimeLighting(state.time);
        this.updateWaterMesh(state.time);
        this.updateBoat(state.boat);
        this.updateWaypoints();
        this.updateTrajectoryLine();
        this.updateHullDebug(state.localEnv);
        this.updateFlowVectors(state.time);
        this.updateSensorDebug();
    }

    updateTimeLighting(time) {
        const bounds = this.sim.state.env.bounds;
        const centerX = bounds.width / 2;
        const centerZ = bounds.height / 2;
        const daySeconds = ((time % 86400) + 86400) % 86400;
        const dayFraction = daySeconds / 86400;
        const sunAngle = dayFraction * Math.PI * 2 - Math.PI / 2;
        const altitude = Math.sin(sunAngle);
        const daylight = THREE.MathUtils.clamp((altitude + 0.12) / 0.55, 0, 1);
        const dawnDusk = 1 - Math.abs(dayFraction - 0.5) * 2;
        const orbitRadius = Math.max(bounds.width, bounds.height) * 0.9;
        const y = Math.max(altitude, -0.18) * orbitRadius * 0.78;
        const x = centerX + Math.cos(sunAngle) * orbitRadius;
        const z = centerZ - orbitRadius * 0.24;
        const lightColor = this.lightColorForAltitude(altitude);

        this.sunLight.position.set(x, y, z);
        this.sunLight.target.position.set(centerX, 0, centerZ);
        this.sunLight.target.updateMatrixWorld();
        this.sunLight.intensity = 0.05 + daylight * 2.55;
        this.sunLight.color.copy(lightColor);

        this.ambientLight.intensity = 0.45 + daylight * 1.35;
        const skyColor = this.skyColorForLight(altitude, dawnDusk);
        this.scene.background.copy(skyColor);
        this.scene.fog.color.copy(skyColor);
        this.renderer.toneMappingExposure = 0.98 + daylight * 0.24;
        if (this.skyMaterial) {
            const twilight = THREE.MathUtils.clamp((altitude + 0.22) / 0.34, 0, 1) * (1 - daylight);
            this.skyMaterial.uniforms.topColor.value.copy(this.nightSkyColor).lerp(this.daySkyColor, daylight);
            this.skyMaterial.uniforms.horizonColor.value.copy(new THREE.Color(0xd1edff)).lerp(new THREE.Color(0xffbf7a), twilight * 0.65);
            this.skyMaterial.uniforms.bottomColor.value.setHex(0x06202a);
        }
    }

    lightColorForAltitude(altitude) {
        const horizonWarmth = 1 - THREE.MathUtils.clamp((altitude - 0.05) / 0.6, 0, 1);
        return new THREE.Color(0xffffff).lerp(new THREE.Color(0xffb25f), horizonWarmth * 0.7);
    }

    skyColorForLight(altitude, dawnDusk) {
        const daylight = THREE.MathUtils.clamp((altitude + 0.1) / 0.8, 0, 1);
        const twilight = THREE.MathUtils.clamp((altitude + 0.22) / 0.28, 0, 1) * (1 - daylight);
        const color = this.nightSkyColor.clone().lerp(this.duskSkyColor, twilight * (0.35 + dawnDusk * 0.65));
        return color.lerp(this.daySkyColor, daylight);
    }

    updateWaterMesh(time) {
        const geometry = this.waterMesh.geometry;
        const posAttr = geometry.attributes.position;
        const colorAttr = geometry.attributes.color;
        const waterField = this.sim.envModel.waterField;
        if (this.waterNormalMap) {
            this.waterNormalMap.offset.set((time * 0.012) % 1, (time * 0.018) % 1);
        }

        for (let i = 0; i < posAttr.count; i++) {
            const x = posAttr.getX(i);
            const z = posAttr.getZ(i);
            const sample = waterField.sampleAt({x, y: 0, z}, time);
            posAttr.setY(i, sample.surfaceHeight);
            this.colorFromWaterSample(sample, this.mode, this.tmpColor);
            colorAttr.setXYZ(i, this.tmpColor.r, this.tmpColor.g, this.tmpColor.b);
        }

        posAttr.needsUpdate = true;
        colorAttr.needsUpdate = true;
        geometry.computeVertexNormals();
    }

    colorFromWaterSample(sample, mode, target) {
        if (mode === "height") {
            const h = THREE.MathUtils.clamp((sample.surfaceHeight + 0.8) / 1.6, 0, 1);
            return target.setRGB(0.05 + h * 0.45, 0.28 + h * 0.5, 0.55 + h * 0.42);
        }

        if (mode === "velocity") {
            const speed = Math.sqrt(
                sample.velocity.x ** 2 + sample.velocity.y ** 2 + sample.velocity.z ** 2
            );
            const v = THREE.MathUtils.clamp(speed / 1.25, 0, 1);
            return target.setRGB(0.02 + v * 0.95, 0.24 + v * 0.55, 0.42 + (1 - v) * 0.35);
        }

        if (mode === "normal") {
            return target.setRGB(
                Math.abs(sample.normal.x),
                Math.abs(sample.normal.y),
                Math.abs(sample.normal.z)
            );
        }

        if (mode === "hull" || mode === "physics") {
            const a = Math.sqrt(
                sample.acceleration.x ** 2 + sample.acceleration.y ** 2 + sample.acceleration.z ** 2
            );
            const f = THREE.MathUtils.clamp(a / 2.5, 0, 1);
            return target.setRGB(0.04 + f * 0.75, 0.38 + (1 - f) * 0.28, 0.52 + (1 - f) * 0.2);
        }

        const crest = THREE.MathUtils.clamp((sample.surfaceHeight + 0.45) / 0.9, 0, 1);
        return target.setRGB(0.04 + crest * 0.1, 0.33 + crest * 0.2, 0.46 + crest * 0.28);
    }

    updateBoat(boatState) {
        const visualDraft = Math.min(
            Math.max(boatState.draft || boatState.dimensions.y * 0.25, 0),
            boatState.dimensions.y
        );
        const hullCenterAboveWaterline = boatState.dimensions.y * 0.5 - visualDraft;

        this.boat.position.set(
            boatState.pos.x,
            boatState.pos.y + hullCenterAboveWaterline,
            boatState.pos.z
        );
        this.boat.rotation.set(
            boatState.orientation.x,
            boatState.orientation.y,
            boatState.orientation.z
        );
        this.cameraTarget.lerp(this.boat.position, 0.025);
        this.updateCamera();
    }

    updateWaypoints() {
        const activeIdx = this.sim.state.goal.waypointIdx;
        this.waypoints.forEach((group, idx) => {
            group.visible = idx >= activeIdx;
            group.children.forEach((child) => {
                if (child.material && child.material.color) {
                    child.material.color.set(idx === activeIdx ? 0x56f39a : 0x2d8cff);
                }
            });
        });
    }

    updatePathLine() {
        const points = this.sim.state.goal.waypoints.map((wp) => new THREE.Vector3(wp.x, 0.25, wp.z));
        const geometry = new THREE.BufferGeometry().setFromPoints(points);
        const material = new THREE.LineBasicMaterial({color: 0x56f39a});
        this.pathLine = new THREE.Line(geometry, material);
        this.scene.add(this.pathLine);
    }

    updateTrajectoryLine() {
        const logs = this.sim.logs.boatStates;
        if (logs.length < 2) return;

        const stride = Math.max(1, Math.floor(logs.length / 250));
        const points = [];
        for (let i = 0; i < logs.length; i += stride) {
            const p = logs[i].pos;
            points.push(new THREE.Vector3(p.x, p.y + 0.08, p.z));
        }

        if (!this.trajectoryLine) {
            this.trajectoryLine = new THREE.Line(
                new THREE.BufferGeometry(),
                new THREE.LineBasicMaterial({color: 0xffd166})
            );
            this.scene.add(this.trajectoryLine);
        }
        const oldGeometry = this.trajectoryLine.geometry;
        this.trajectoryLine.geometry = new THREE.BufferGeometry().setFromPoints(points);
        oldGeometry.dispose();
    }

    updateHullDebug(localEnv) {
        const samples = localEnv ? localEnv.hullWaterSamples : [];
        const visible = this.mode === "hull" || this.mode === "physics";

        this.hullMarkers.forEach((marker, idx) => {
            const sample = samples[idx];
            marker.visible = Boolean(sample) && visible;
            this.hullLines[idx].visible = Boolean(sample) && visible;
            if (!sample || !visible) return;

            marker.position.set(sample.samplePos.x, sample.waterH + 0.15, sample.samplePos.z);
            const color = sample.submerged ? 0x56f39a : 0xffc857;
            marker.material.color.set(color);

            const start = new THREE.Vector3(sample.samplePos.x, sample.samplePos.y, sample.samplePos.z);
            const end = new THREE.Vector3(sample.samplePos.x, sample.waterH, sample.samplePos.z);
            this.hullLines[idx].geometry.setFromPoints([start, end]);
            this.hullLines[idx].material.color.set(color);
        });
    }

    updateFlowVectors(time) {
        const visible = this.mode === "velocity" || this.mode === "physics";
        const waterField = this.sim.envModel.waterField;

        this.flowArrows.forEach((line) => {
            line.visible = visible;
            if (!visible) return;
            const base = line.userData.base;
            const sample = waterField.sampleAt({x: base.x, y: 0, z: base.z}, time);
            const scale = 2.8;
            const start = new THREE.Vector3(base.x, sample.surfaceHeight + 0.18, base.z);
            const end = new THREE.Vector3(
                base.x + sample.velocity.x * scale,
                sample.surfaceHeight + 0.18,
                base.z + sample.velocity.z * scale
            );
            line.geometry.setFromPoints([start, end]);
        });
    }

    updateSensorDebug() {
        const feeds = this.sim.getSensorFeeds ? this.sim.getSensorFeeds() : [];
        feeds.forEach((feed) => {
            const marker = this.sensorMarkers.get(feed.id);
            const line = this.sensorLines.get(feed.id);
            if (!marker || !line) return;

            const visible = feed.active && feed.pose && this.mode === "physics";
            marker.visible = visible;
            line.visible = visible;
            if (!visible) return;

            const pos = feed.pose.position;
            const forward = feed.pose.forward;
            const start = new THREE.Vector3(pos.x, pos.y, pos.z);
            const end = new THREE.Vector3(
                pos.x + forward.x * 2.2,
                pos.y + forward.y * 2.2,
                pos.z + forward.z * 2.2
            );
            marker.position.copy(start);
            line.geometry.setFromPoints([start, end]);
        });
    }

    getSensorTargets() {
        return [
            this.waterMesh,
            ...this.obstacles
        ].filter(Boolean);
    }
}
