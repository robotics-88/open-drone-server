from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
import asyncio
import threading
from ros_bridge import start_ros, bridge
import uvicorn
import json
from typing import Dict, Any

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"], allow_credentials=True, allow_methods=["*"], allow_headers=["*"]
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
    return {
        "lat": bridge.lat,
        "lon": bridge.lon,
        "heading": bridge.heading,
        "status": "in flight"  # placeholder
    }

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    if bridge is None:
        return {"error": "Bridge not initialized yet"}
    await ws.accept()
    while True:
        await ws.send_json({
            "lat": bridge.lat,
            "lon": bridge.lon,
            "alt": 10,  # placeholder for now
            "heading": bridge.heading,
            "status": "in flight"
        })
        await asyncio.sleep(1)

@app.post("/run_mission")
async def run_mission(mission: Dict[str, Any]):
    if bridge is None:
        return {"error": "Bridge not initialized yet"}
    mission_str = json.dumps(mission)
    bridge.run_mission(mission_str)
    return {"status": "ok"}

if __name__ == "__main__":
    threading.Thread(target=start_ros, daemon=True).start()

    uvicorn.run("main:app", host="127.0.0.1", port=8888, reload=False)
