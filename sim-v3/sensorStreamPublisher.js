const CAMERA_URL = "ws://127.0.0.1:8765";
const LIDAR_URL = "ws://127.0.0.1:8766";
const TELEMETRY_URL = "ws://127.0.0.1:8767";

export class SensorStreamPublisher {
    constructor(options = {}) {
        this.urls = {
            camera: options.cameraUrl || CAMERA_URL,
            lidar: options.lidarUrl || LIDAR_URL,
            telemetry: options.telemetryUrl || TELEMETRY_URL
        };
        this.enabled = options.enabled !== false;
        this.quiet = options.quiet !== false;
        this.frameIds = new Map();
        this.lastSent = new Map();
        this.connectAttempted = false;
        this.sockets = {
            camera: null,
            lidar: null,
            telemetry: null
        };
    }

    publish(feeds) {
        if (!this.enabled) {
            return;
        }
        this.ensureSockets();

        feeds.forEach((feed) => {
            if (!feed.active || feed.status !== "live") {
                return;
            }
            const key = `${feed.id}:${feed.t}`;
            if (this.lastSent.get(feed.id) === key) {
                return;
            }

            let sent = false;
            if (feed.displayType === "image") {
                sent = this.publishCamera(feed);
            }
            else if (feed.displayType === "pointCloud") {
                sent = this.publishLidar(feed);
            }
            else {
                sent = this.publishTelemetry(feed);
            }

            if (sent) {
                this.lastSent.set(feed.id, key);
            }
        });
    }

    publishCamera(feed) {
        const socket = this.sockets.camera;
        const pixels = feed.data && feed.data.rgbaPixels;
        if (!this.isOpen(socket) || !pixels) {
            return false;
        }

        const width = feed.data.width;
        const height = feed.data.height;
        const channels = 4;
        const expected = width * height * channels;
        if (pixels.byteLength !== expected) {
            return false;
        }

        const header = this.makeHeader({
            frameId: this.nextFrameId(feed.id),
            timestamp: this.timestamp(feed),
            a: width,
            b: height,
            c: channels
        });
        socket.send(header);
        socket.send(this.copyArrayBuffer(pixels));
        return true;
    }

    publishLidar(feed) {
        const socket = this.sockets.lidar;
        if (!this.isOpen(socket) || !feed.data || !Array.isArray(feed.data.points)) {
            return false;
        }

        const pointData = this.lidarPointsToFloat32(feed.data.points);
        const header = this.makeHeader({
            frameId: this.nextFrameId(feed.id),
            timestamp: this.timestamp(feed),
            a: pointData.length / 3,
            b: 3,
            c: 4
        });
        socket.send(header);
        socket.send(pointData.buffer);
        return true;
    }

    publishTelemetry(feed) {
        const socket = this.sockets.telemetry;
        if (!this.isOpen(socket)) {
            return false;
        }

        socket.send(JSON.stringify({
            frameId: this.nextFrameId(feed.id),
            timestamp: this.timestamp(feed),
            id: feed.id,
            name: feed.name,
            type: feed.type,
            status: feed.status,
            t: feed.t,
            pose: this.sanitize(feed.pose),
            data: this.sanitize(feed.data),
            summary: feed.summary
        }));
        return true;
    }

    lidarPointsToFloat32(points) {
        const hitPoints = points.filter(Boolean);
        const values = new Float32Array(hitPoints.length * 3);
        hitPoints.forEach((point, idx) => {
            values[idx * 3] = point.x;
            values[idx * 3 + 1] = point.y;
            values[idx * 3 + 2] = point.z;
        });
        return values;
    }

    makeHeader({frameId, timestamp, a, b, c}) {
        const header = new ArrayBuffer(24);
        const view = new DataView(header);
        view.setUint32(0, frameId, true);
        view.setFloat64(4, timestamp, true);
        view.setUint32(12, a, true);
        view.setUint32(16, b, true);
        view.setUint32(20, c, true);
        return header;
    }

    nextFrameId(feedId) {
        const next = (this.frameIds.get(feedId) || 0) + 1;
        this.frameIds.set(feedId, next);
        return next - 1;
    }

    timestamp(feed) {
        return Number.isFinite(feed.t) ? feed.t * 1000 : performance.now();
    }

    createSocket(kind) {
        if (typeof WebSocket === "undefined") {
            return null;
        }

        const socket = new WebSocket(this.urls[kind]);
        socket.binaryType = "arraybuffer";
        socket.addEventListener("open", () => {
            if (!this.quiet) console.info(`${kind} sensor stream connected: ${this.urls[kind]}`);
        });
        socket.addEventListener("close", () => {
            if (!this.quiet && socket.__opened) console.info(`${kind} sensor stream closed: ${this.urls[kind]}`);
        });
        socket.addEventListener("open", () => {
            socket.__opened = true;
        });
        socket.addEventListener("error", () => {
            // Receivers are optional; avoid noisy retries while developing the sim UI.
        });
        return socket;
    }

    ensureSockets() {
        if (this.connectAttempted) {
            return;
        }
        this.connectAttempted = true;
        this.sockets.camera = this.createSocket("camera");
        this.sockets.lidar = this.createSocket("lidar");
        this.sockets.telemetry = this.createSocket("telemetry");
    }

    close() {
        Object.values(this.sockets).forEach((socket) => {
            if (socket && typeof socket.close === "function") {
                socket.close();
            }
        });
        this.sockets = {camera: null, lidar: null, telemetry: null};
        this.connectAttempted = false;
        this.lastSent.clear();
    }

    dispose() {
        this.close();
    }

    isOpen(socket) {
        return socket && socket.readyState === WebSocket.OPEN;
    }

    copyArrayBuffer(view) {
        return view.buffer.slice(view.byteOffset, view.byteOffset + view.byteLength);
    }

    sanitize(value) {
        if (!value) {
            return value;
        }
        if (value instanceof Uint8Array || value instanceof Float32Array) {
            return undefined;
        }
        if (Array.isArray(value)) {
            return value.map((item) => this.sanitize(item));
        }
        if (typeof value === "object") {
            const output = {};
            Object.entries(value).forEach(([key, item]) => {
                const sanitized = this.sanitize(item);
                if (sanitized !== undefined) {
                    output[key] = sanitized;
                }
            });
            return output;
        }
        return value;
    }
}
