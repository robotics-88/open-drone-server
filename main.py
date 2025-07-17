from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
import asyncio
import threading
import json, rasterio
from rasterio.warp import transform_bounds
from typing import Dict, Any
from starlette.websockets import WebSocketDisconnect
import uvicorn

import os
from pathlib import Path

from ros_bridge import start_ros, bridge

app = FastAPI()

DEM_DIR = Path.home() / "r88_public" / "dems"

# Enable CORS for all origins
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/dems")
def list_dems():
    if not DEM_DIR.exists():
        return []

    results = []
    for tif_file in DEM_DIR.glob("*.tif"):
        try:
            with rasterio.open(tif_file) as src:
                # Reproject bounds from native CRS to EPSG:4326
                latlon_bounds = transform_bounds(src.crs, "EPSG:4326", *src.bounds)

                result = {
                    "filename": tif_file.name,
                    "bounds": {
                        "min_lon": latlon_bounds[0],
                        "min_lat": latlon_bounds[1],
                        "max_lon": latlon_bounds[2],
                        "max_lat": latlon_bounds[3],
                    }
                }
                results.append(result)
        except Exception as e:
            print(f"Error reading {tif_file.name}: {e}")
            continue

    return results

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

@app.post("/set_module_active")
async def set_module_active(req: Dict[str, Any]):
    if bridge is None:
        return {"error": "Bridge not initialized yet"}

    try:
        # Convert to JSON string and publish as std_msgs/String
        msg = json.dumps(req)
        bridge.publish_toggle_module(msg)
        return {"status": "ok"}
    except Exception as e:
        return {"error": str(e)}

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    # Reject if bridge not ready
    if bridge is None:
        await ws.accept()
        await ws.close(code=1011)
        return

    await ws.accept()
    last_log_time = 0  # local to this connection
    last_capabilities_time = 0 
    
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
                if not bridge.log_stamp == last_log_time:
                    last_log_time = bridge.log_stamp
                    print(f"last log time: {last_log_time}")
                    await ws.send_json({
                        "type": "log",
                        "message": bridge.log,
                        "level": bridge.log_level
                    })
                if not bridge.last_capabilities_time == last_capabilities_time:
                    last_capabilities_time = bridge.last_capabilities_time
                    print(f"last capabilities time: {last_capabilities_time}")
                    await ws.send_json({
                        "type": "capability_reload"
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
