import asyncio
import struct

import cv2
import numpy as np
import websockets

HEADER_SIZE = 24
HEADER_FORMAT = "<IdIII"
WINDOW_NAME = "LiDAR Top-Down View"
CANVAS_WIDTH = 900
CANVAS_HEIGHT = 700
PADDING = 44
WORLD_MIN = -50.0
WORLD_MAX = 50.0

latest_points = None
latest_frame_id = None
state_lock = asyncio.Lock()


def world_to_canvas(points):
    xz = points[:, [0, 2]]
    drawable_w = CANVAS_WIDTH - PADDING * 2
    drawable_h = CANVAS_HEIGHT - PADDING * 2
    world_span = WORLD_MAX - WORLD_MIN

    pixels = np.empty_like(xz)
    pixels[:, 0] = PADDING + ((xz[:, 0] - WORLD_MIN) / world_span) * drawable_w
    pixels[:, 1] = CANVAS_HEIGHT - PADDING - ((xz[:, 1] - WORLD_MIN) / world_span) * drawable_h
    return pixels.astype(np.int32)


def draw_top_down(points, frame_id):
    canvas = np.full((CANVAS_HEIGHT, CANVAS_WIDTH, 3), 18, dtype=np.uint8)
    cv2.rectangle(
        canvas,
        (PADDING, PADDING),
        (CANVAS_WIDTH - PADDING, CANVAS_HEIGHT - PADDING),
        (55, 70, 76),
        1,
    )

    if points is None or len(points) == 0:
        message = f"frame={frame_id} points=0"
        cv2.putText(
            canvas,
            message,
            (PADDING, PADDING + 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (210, 230, 235),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(canvas, f"x/z range [{WORLD_MIN:.0f}, {WORLD_MAX:.0f}]", (PADDING, CANVAS_HEIGHT - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (170, 190, 198), 1, cv2.LINE_AA)
        return canvas

    pixels = world_to_canvas(points)
    y = points[:, 1]
    y_min = float(y.min())
    y_span = max(float(y.max() - y_min), 1e-6)

    for idx, pixel in enumerate(pixels):
        px, py = int(pixel[0]), int(pixel[1])
        if px < 0 or px >= CANVAS_WIDTH or py < 0 or py >= CANVAS_HEIGHT:
            continue
        height_norm = (float(y[idx]) - y_min) / y_span
        color = (
            int(255 - 155 * height_norm),
            int(115 + 120 * height_norm),
            int(45 + 210 * height_norm),
        )
        cv2.circle(canvas, (px, py), 4, color, -1, cv2.LINE_AA)

    origin = np.array([[0.0, 0.0, 0.0]])
    origin_px = world_to_canvas(origin)
    ox, oy = origin_px[-1]
    if 0 <= ox < CANVAS_WIDTH and 0 <= oy < CANVAS_HEIGHT:
        cv2.drawMarker(
            canvas,
            (int(ox), int(oy)),
            (80, 255, 120),
            markerType=cv2.MARKER_CROSS,
            markerSize=18,
            thickness=2,
        )

    visible = np.sum(
        (points[:, 0] >= WORLD_MIN) &
        (points[:, 0] <= WORLD_MAX) &
        (points[:, 2] >= WORLD_MIN) &
        (points[:, 2] <= WORLD_MAX)
    )
    title = f"frame={frame_id} points={len(points)} visible={visible}"
    bounds = f"fixed x/z range [{WORLD_MIN:.0f}, {WORLD_MAX:.0f}]"
    cv2.putText(canvas, title, (PADDING, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (225, 240, 245), 2, cv2.LINE_AA)
    cv2.putText(canvas, bounds, (PADDING, CANVAS_HEIGHT - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (170, 190, 198), 1, cv2.LINE_AA)
    cv2.putText(canvas, "+x right, +z up", (CANVAS_WIDTH - 210, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (170, 190, 198), 1, cv2.LINE_AA)
    return canvas


async def handler(websocket):
    global latest_points
    global latest_frame_id

    print("LiDAR client connected")

    try:
        while True:
            try:
                header = await websocket.recv()
            except websockets.exceptions.ConnectionClosedOK:
                print("LiDAR client disconnected")
                break

            if not isinstance(header, bytes) or len(header) != HEADER_SIZE:
                print("Bad LiDAR header")
                continue

            frame_id, timestamp, point_count, fields_per_point, bytes_per_field = struct.unpack(
                HEADER_FORMAT,
                header,
            )

            payload = await websocket.recv()
            if not isinstance(payload, bytes):
                print("Bad LiDAR payload")
                continue

            expected = point_count * fields_per_point * bytes_per_field
            if len(payload) != expected:
                print(f"Wrong LiDAR payload size: got {len(payload)} expected {expected}")
                continue

            points = np.frombuffer(payload, dtype=np.float32).reshape((point_count, fields_per_point))

            async with state_lock:
                latest_points = points[:, :3].astype(np.float64) if point_count > 0 else np.empty((0, 3))
                latest_frame_id = frame_id

            print(f"frame={frame_id} timestamp={timestamp:.3f} points={point_count}")
    except Exception as exc:
        print(f"LiDAR handler error: {exc}")


async def render_loop():
    global latest_points
    global latest_frame_id

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, CANVAS_WIDTH, CANVAS_HEIGHT)

    while True:
        async with state_lock:
            pending = latest_points
            frame_id = latest_frame_id
            latest_points = None

        if pending is not None:
            canvas = draw_top_down(pending, frame_id)
            cv2.imshow(WINDOW_NAME, canvas)

        key = cv2.waitKey(1)
        if key == 27:
            break
        await asyncio.sleep(1 / 60)

    cv2.destroyAllWindows()


async def main():
    print("Listening on ws://127.0.0.1:8766")
    render_task = asyncio.create_task(render_loop())
    try:
        async with websockets.serve(handler, "127.0.0.1", 8766, max_size=None):
            await asyncio.Future()
    finally:
        render_task.cancel()
        cv2.destroyAllWindows()


asyncio.run(main())
