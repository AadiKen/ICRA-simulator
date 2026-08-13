import asyncio
import struct
import numpy as np
import websockets
import cv2

HEADER_SIZE = 24 
HEADER_FORMAT = "<IdIII"

async def handler(websocket):
    print("Client connected")

    while True:
        header = await websocket.recv()
        if not isinstance(header, bytes) or len(header) != HEADER_SIZE:
            print("Bad header")
            continue

        frame_id, timestamp, width, height, channels = struct.unpack(
            HEADER_FORMAT, header
        )
        payload = await websocket.recv()
        if not isinstance(payload, bytes):
            print("bad payload")
            continue

        expected_size = width * height * channels
        if len(payload) != expected_size:
            print(f"Wrong  size got {len(payload)} expected {expected_size}")
            continue
        img = np.frombuffer(payload, dtype=np.uint8).reshape((height, width, channels))
        if channels == 4:
            img = img[:, :, :3]
        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        cv2.imshow("Boat Cam Feed", img_bgr)
        key = cv2.waitKey(1)
        if key == 27: 
            break

    cv2.destroyAllWindows()

async def main():
    async with websockets.serve(handler, "127.0.0.1", 8765, max_size=None):
        print("Listening on ws://127.0.0.1:8765")
        await asyncio.Future()

asyncio.run(main())