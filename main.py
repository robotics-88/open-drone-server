from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
import asyncio
import threading
from ros_bridge import start_ros, bridge
import uvicorn
import json
from typing import Dict, Any
from starlette.websockets import WebSocketDisconnect

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
        try:
            await ws.accept()
            await ws.close(code=1011)
        except Exception:
            pass
        return

    await ws.accept()
    try:
        while True:
            await ws.send_json({
                "lat": bridge.lat,
                "lon": bridge.lon,
                "alt": 10,
                "heading": bridge.heading,
                "status": "in flight"
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
            # Socket already closed — ignore
            pass

@app.post("/run_mission")
async def run_mission(mission: Dict[str, Any]):
    if bridge is None:
        return {"error": "Bridge not initialized yet"}
    mission_str = json.dumps(mission)
    bridge.run_mission(mission_str)
    return {"status": "ok"}

if __name__ == "__main__":
    threading.Thread(target=start_ros, daemon=True).start()

    uvicorn.run("main:app", host="0.0.0.0", port=8080, reload=False)
