from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
import asyncio
import threading
import json
from typing import Dict, Any
from starlette.websockets import WebSocketDisconnect
import uvicorn

from ros_bridge import start_ros, bridge

app = FastAPI()

# Enable CORS for all origins
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/capabilities")
def get_capabilities():
    if bridge is None:
        return {"error": "Bridge not initialized yet"}
    return {"capabilities": bridge.capabilities}

@app.get("/status")
def get_status():
    if bridge is None:
        return {"error": "Bridge not initialized yet"}
    # Return a snapshot of all telemetry fields
    return {
        "lat": bridge.lat,
        "lon": bridge.lon,
        "heading": bridge.heading,
        "status": bridge.status,
        "battery_voltage": getattr(bridge, "battery_voltage", None),
        "gps_satellites": getattr(bridge, "gps_satellites", None),
        "num_cameras": getattr(bridge, "num_cameras", None),
        "height": getattr(bridge, "height", None),
        "distance": getattr(bridge, "distance", None),
        "speed": getattr(bridge, "speed", None),
    }

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    # Reject if bridge not ready
    if bridge is None:
        await ws.accept()
        await ws.close(code=1011)
        return

    await ws.accept()
    try:
        while True:
            if not bridge.stale:
                # Send full telemetry packet
                await ws.send_json({
                    "type": "telemetry",
                    "lat": bridge.lat,
                    "lon": bridge.lon,
                    "alt": getattr(bridge, "alt", None),
                    "heading": bridge.heading,
                    "status": bridge.status,
                    "battery_voltage": getattr(bridge, "battery_voltage", None),
                    "gps_satellites": getattr(bridge, "gps_satellites", None),
                    "num_cameras": getattr(bridge, "num_cameras", None),
                    "height": getattr(bridge, "height", None),
                    "distance": getattr(bridge, "distance", None),
                    "speed": getattr(bridge, "speed", None),
                })
                await asyncio.sleep(1)
    except WebSocketDisconnect:
        print("WebSocket disconnected")
    except Exception as e:
        print(f"WebSocket error: {e}")
    finally:
        try:
            await ws.close()
        except RuntimeError:
            pass

@app.post("/run_mission")
async def run_mission(mission: Dict[str, Any]):
    if bridge is None:
        return {"error": "Bridge not initialized yet"}
    payload = json.dumps(mission)
    bridge.run_mission(payload)
    return {"status": "ok"}

if __name__ == "__main__":
    # Start ROS bridge in background thread
    threading.Thread(target=start_ros, daemon=True).start()
    # Run FastAPI/uvicorn
    uvicorn.run("main:app", host="0.0.0.0", port=8080, reload=False)
