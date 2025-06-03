from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
import asyncio
import threading
from ros_bridge import start_ros, bridge

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"], allow_credentials=True, allow_methods=["*"], allow_headers=["*"]
)

@app.get("/capabilities")
def get_capabilities():
    return {"capabilities": bridge.capabilities}

@app.get("/status")
def get_status():
    return {
        "lat": bridge.lat,
        "lon": bridge.lon,
        "heading": bridge.heading,
        "status": "in flight"  # placeholder
    }

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
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

if __name__ == "__main__":
    threading.Thread(target=start_ros, daemon=True).start()

    import uvicorn
    uvicorn.run("main:app", host="127.0.0.1", port=8888, reload=False)
