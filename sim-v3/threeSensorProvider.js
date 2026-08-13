import * as THREE from "./vendor/three.module.js";
import {
    rotateBodyOffset,
    sensorWorldPose,
    vec3
} from "./schema.js?v=26";

export class ThreeSensorProvider {
    constructor(threeRenderer, options = {}) {
        this.threeRenderer = threeRenderer;
        this.renderer = threeRenderer.renderer;
        this.scene = threeRenderer.scene;
        this.getTargets = options.getTargets || (() => threeRenderer.getSensorTargets());
        this.syncScene = options.syncScene || (() => threeRenderer.syncToSimulation());
        this.getWeather = options.getWeather || (() => "clear");
        this.raycaster = new THREE.Raycaster();
        this.sensorCamera = new THREE.PerspectiveCamera(60, 1, 0.05, 500);
        this.renderTargets = new Map();
        this.canvas = document.createElement("canvas");
        this.canvasContext = this.canvas.getContext("2d");
        this.syncedTime = null;
    }

    canObserve(sensor) {
        return sensor.type === "dayCam" ||
            sensor.type === "nightCam" ||
            sensor.type === "lidar";
    }

    observe(sensor, state) {
        this.ensureSceneSynced(state);
        if (sensor.type === "dayCam" || sensor.type === "nightCam") {
            return this.observeCamera(sensor, state);
        }
        if (sensor.type === "lidar") {
            return this.observeLidar(sensor, state);
        }
        return null;
    }

    beginFrame(state) {
        this.syncedTime = null;
    }

    ensureSceneSynced(state) {
        if (this.syncedTime === state.time) {
            return;
        }
        this.syncScene();
        this.syncedTime = state.time;
    }

    observeCamera(sensor, state) {
        const pose = sensorWorldPose(sensor, state.boat);
        const width = Math.max(Math.floor(sensor.width || 320), 1);
        const height = Math.max(Math.floor(sensor.height || 240), 1);
        const renderTarget = this.getRenderTarget(sensor, width, height);
        const oldTarget = this.renderer.getRenderTarget();

        this.sensorCamera.fov = sensor.fov || 60;
        this.sensorCamera.aspect = width / height;
        this.sensorCamera.near = 0.05;
        this.sensorCamera.far = sensor.range || 500;
        this.sensorCamera.position.set(pose.position.x, pose.position.y, pose.position.z);
        this.sensorCamera.up.set(pose.up.x, pose.up.y, pose.up.z).normalize();
        this.sensorCamera.lookAt(
            pose.position.x + pose.forward.x,
            pose.position.y + pose.forward.y,
            pose.position.z + pose.forward.z
        );
        this.sensorCamera.updateProjectionMatrix();

        this.renderer.setRenderTarget(renderTarget);
        this.renderer.render(this.scene, this.sensorCamera);

        const pixels = new Uint8Array(width * height * 4);
        this.renderer.readRenderTargetPixels(renderTarget, 0, 0, width, height, pixels);
        this.renderer.setRenderTarget(oldTarget);

        const weather = this.getWeather();
        const imageDataUrl = this.pixelsToDataUrl(pixels, width, height, sensor.type, weather);
        return {
            t: state.time,
            type: sensor.type,
            sensorId: sensor.id,
            sensorName: sensor.name,
            pose,
            width,
            height,
            fov: sensor.fov,
            imageDataUrl,
            rgbaPixels: pixels,
            weather
        };
    }

    observeLidar(sensor, state) {
        const pose = sensorWorldPose(sensor, state.boat);
        const horizontalFov = sensor.hRange || 90;
        const verticalFov = sensor.vRange || 30;
        const angularRes = Math.max(sensor.angularRes || 1, 0.1);
        const maxDistance = sensor.dRange || 25;
        const width = Math.floor(horizontalFov / angularRes) + 1;
        const height = Math.floor(verticalFov / angularRes) + 1;
        const ranges = [];
        const points = [];
        let hitCount = 0;
        let minRange = maxDistance;

        const origin = new THREE.Vector3(pose.position.x, pose.position.y, pose.position.z);
        const targets = this.getTargets().filter((target) => {
            if (!target || !target.userData.lidarTarget) {
                return false;
            }
            if (target.userData.type === "water") {
                return Boolean(sensor.includeWater);
            }
            return true;
        });

        for (let row = 0; row < height; row += 1) {
            const pitchDeg = height === 1
                ? 0
                : -verticalFov / 2 + row * angularRes;
            const pitch = THREE.MathUtils.degToRad(pitchDeg);

            for (let col = 0; col < width; col += 1) {
                const yawDeg = width === 1
                    ? 0
                    : -horizontalFov / 2 + col * angularRes;
                const yaw = THREE.MathUtils.degToRad(yawDeg);
                const localDirection = new vec3(
                    Math.sin(yaw) * Math.cos(pitch),
                    Math.sin(pitch),
                    Math.cos(yaw) * Math.cos(pitch)
                );
                const worldDirection = rotateBodyOffset(localDirection, pose.orientation);
                const direction = new THREE.Vector3(
                    worldDirection.x,
                    worldDirection.y,
                    worldDirection.z
                ).normalize();

                this.raycaster.set(origin, direction);
                this.raycaster.near = 0.02;
                this.raycaster.far = maxDistance;
                const hits = this.raycaster.intersectObjects(targets, false);
                const hit = hits[0];
                if (hit) {
                    ranges.push(hit.distance);
                    points.push({
                        x: hit.point.x,
                        y: hit.point.y,
                        z: hit.point.z,
                        label: hit.object.userData.label || null,
                        type: hit.object.userData.type || null
                    });
                    hitCount += 1;
                    minRange = Math.min(minRange, hit.distance);
                }
                else {
                    ranges.push(maxDistance);
                    points.push(null);
                }
            }
        }

        this.applyLidarWeather(ranges, points, maxDistance);
        hitCount = 0;
        minRange = maxDistance;
        ranges.forEach((range, idx) => {
            if (range < maxDistance && points[idx]) {
                hitCount += 1;
                minRange = Math.min(minRange, range);
            }
        });

        return {
            t: state.time,
            type: sensor.type,
            sensorId: sensor.id,
            sensorName: sensor.name,
            pose,
            width,
            height,
            horizontalFov,
            verticalFov,
            angularRes,
            maxDistance,
            ranges,
            points,
            hitCount,
            minRange: hitCount > 0 ? minRange : null
        };
    }

    getRenderTarget(sensor, width, height) {
        const key = sensor.id || sensor.name;
        const existing = this.renderTargets.get(key);
        if (existing && existing.width === width && existing.height === height) {
            return existing;
        }
        if (existing) {
            existing.dispose();
        }
        const renderTarget = new THREE.WebGLRenderTarget(width, height);
        renderTarget.texture.colorSpace = THREE.SRGBColorSpace;
        this.renderTargets.set(key, renderTarget);
        return renderTarget;
    }

    pixelsToDataUrl(pixels, width, height, sensorType, weather = "clear") {
        this.canvas.width = width;
        this.canvas.height = height;
        const imageData = this.canvasContext.createImageData(width, height);

        for (let y = 0; y < height; y += 1) {
            for (let x = 0; x < width; x += 1) {
                const sourceIdx = ((height - 1 - y) * width + x) * 4;
                const targetIdx = (y * width + x) * 4;
                if (sensorType === "nightCam") {
                    const gray = pixels[sourceIdx] * 0.22 + pixels[sourceIdx + 1] * 0.55 + pixels[sourceIdx + 2] * 0.23;
                    imageData.data[targetIdx] = gray * 0.35;
                    imageData.data[targetIdx + 1] = Math.min(gray * 1.3, 255);
                    imageData.data[targetIdx + 2] = gray * 0.45;
                }
                else {
                    imageData.data[targetIdx] = pixels[sourceIdx];
                    imageData.data[targetIdx + 1] = pixels[sourceIdx + 1];
                    imageData.data[targetIdx + 2] = pixels[sourceIdx + 2];
                }
                imageData.data[targetIdx + 3] = pixels[sourceIdx + 3];
            }
        }

        this.applyCameraWeather(imageData.data, width, height, weather);
        this.canvasContext.putImageData(imageData, 0, 0);
        return this.canvas.toDataURL("image/png");
    }

    applyCameraWeather(data, width, height, weather) {
        if (weather === "clear") return;
        const fogAmount = weather === "foggy" ? 0.48 : 0.16;
        const noiseAmount = weather === "foggy" ? 18 : 34;
        const darken = weather === "rainy" ? 0.78 : 0.95;
        for (let idx = 0; idx < data.length; idx += 4) {
            const noise = (Math.random() - 0.5) * noiseAmount;
            data[idx] = this.clampByte((data[idx] * darken) * (1 - fogAmount) + 178 * fogAmount + noise);
            data[idx + 1] = this.clampByte((data[idx + 1] * darken) * (1 - fogAmount) + 190 * fogAmount + noise);
            data[idx + 2] = this.clampByte((data[idx + 2] * darken) * (1 - fogAmount) + 194 * fogAmount + noise);
        }

        if (weather === "rainy") {
            const streakCount = Math.floor(width * height / 850);
            for (let i = 0; i < streakCount; i += 1) {
                const x = Math.floor(Math.random() * width);
                const y = Math.floor(Math.random() * height);
                const length = 8 + Math.floor(Math.random() * 18);
                for (let j = 0; j < length; j += 1) {
                    const sx = x + Math.floor(j * 0.25);
                    const sy = y + j;
                    if (sx < 0 || sx >= width || sy < 0 || sy >= height) continue;
                    const idx = (sy * width + sx) * 4;
                    data[idx] = this.clampByte(data[idx] + 42);
                    data[idx + 1] = this.clampByte(data[idx + 1] + 42);
                    data[idx + 2] = this.clampByte(data[idx + 2] + 46);
                }
            }
        }
    }

    applyLidarWeather(ranges, points, maxDistance) {
        const weather = this.getWeather();
        if (weather !== "rainy" && weather !== "foggy") return;
        const dropout = weather === "rainy" ? 0.18 : 0.06;
        const falseReturn = weather === "rainy" ? 0.04 : 0.015;
        const jitter = weather === "rainy" ? 0.35 : 0.12;
        for (let idx = 0; idx < ranges.length; idx += 1) {
            if (ranges[idx] < maxDistance && points[idx]) {
                if (Math.random() < dropout) {
                    ranges[idx] = maxDistance;
                    points[idx] = null;
                    continue;
                }
                ranges[idx] = Math.min(maxDistance, Math.max(0.05, ranges[idx] + (Math.random() - 0.5) * jitter));
            }
            else if (Math.random() < falseReturn) {
                ranges[idx] = maxDistance * (0.12 + Math.random() * 0.55);
                points[idx] = null;
            }
        }
    }

    clampByte(value) {
        return Math.max(0, Math.min(255, Math.round(value)));
    }

    dispose() {
        this.renderTargets.forEach((target) => target.dispose());
        this.renderTargets.clear();
        this.syncedTime = null;
    }
}
