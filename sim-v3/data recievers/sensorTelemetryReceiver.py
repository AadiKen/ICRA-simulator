import asyncio
import json

import websockets


async def handler(websocket):
    print("Telemetry client connected")

    try:
        async for message in websocket:
            if not isinstance(message, str):
                print("Bad telemetry payload")
                continue

            payload = json.loads(message)
            sensor_type = payload.get("type")
            sensor_id = payload.get("id")
            timestamp = payload.get("timestamp")
            summary = payload.get("summary")
            print(f"{timestamp:.3f} {sensor_type}:{sensor_id} {summary}")
    except websockets.exceptions.ConnectionClosed:
        print("Telemetry client disconnected")


async def main():
    async with websockets.serve(handler, "127.0.0.1", 8767, max_size=None):
        print("Listening on ws://127.0.0.1:8767")
        await asyncio.Future()


asyncio.run(main())
