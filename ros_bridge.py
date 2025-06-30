# main.py

import json
import threading
import time
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float64, UInt32, String
from sensor_msgs.msg import NavSatFix, BatteryState

from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
from starlette.websockets import WebSocketDisconnect
import uvicorn

# --- ROS bridge --------------------------------------------------------------

class DroneBridge(Node):
    def __init__(self):
        super().__init__('drone_bridge')

        self.stale = True
        self.last_telemetry_time = time.time()
        self.stale_timeout_sec = 5  # e.g. 5 seconds of silence = stale

        # telemetry fields
        self.capabilities: str = ''
        self.status: str = ''
        self.heading: float = 0.0
        self.lat: float = 0.0
        self.lon: float = 0.0

        # new fields
        self.battery_voltage: float = 0.0
        self.gps_satellites: int = 0
        self.num_cameras: int = 0
        self.height: float = 0.0
        self.distance: float = 0.0
        self.speed: float = 0.0

        # publishers & subscriptions
        self.run_mission_pub = self.create_publisher(String, '/frontend/run_mission', 10)

        self.create_subscription(String,   '/task_manager/capabilities', self.capabilities_cb, 10)
        self.create_subscription(String,   '/task_manager/rest_status',    self.status_cb,       10)
        self.create_subscription(Float64,  '/mavros/global_position/compass_hdg', self.heading_cb, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, '/mavros/global_position/global',      self.global_cb,  qos_profile_sensor_data)

        self.create_subscription(BatteryState, '/mavros/battery', self.battery_cb, qos_profile_sensor_data)
        self.create_subscription(UInt32,   '/mavros/global_position/raw/satellites', self.gps_cb, qos_profile_sensor_data)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt',   self.height_cb, qos_profile_sensor_data)

        self.create_timer(1.0, self.check_stale_telemetry)

    def check_stale_telemetry(self):
        if time.time() - self.last_telemetry_time > self.stale_timeout_sec:
            if not self.stale:
                self.get_logger().warn("Telemetry stale — clearing data")
                self.stale = True


    # existing callbacks
    def capabilities_cb(self, msg: String):
        self.capabilities = msg.data
        
    def status_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse status JSON: {e}")
            return

        # now payload is a dict, so these will work:
        self.status      = payload.get("status", "")
        self.num_cameras = payload.get("num_cameras", 0)
        self.distance    = payload.get("distance", 0.0)
        self.speed       = payload.get("speed", 0.0)

    def heading_cb(self, msg: Float64):
        self.heading = msg.data

    def global_cb(self, msg: NavSatFix):
        self.lat = msg.latitude
        self.lon = msg.longitude

    def battery_cb(self, msg: BatteryState):
        self.battery_voltage = msg.voltage

    def gps_cb(self, msg: UInt32):
        self.last_telemetry_time = time.time()
        self.gps_satellites = msg.data
        if self.last_telemetry_time > self.stale_timeout_sec:
            self.stale = False

    def height_cb(self, msg: Float64):
        self.height = msg.data

    def run_mission(self, mission_payload: str):
        m = String()
        m.data = mission_payload
        self.run_mission_pub.publish(m)


bridge: DroneBridge = None

def start_ros():
    global bridge
    rclpy.init()
    bridge = DroneBridge()
    rclpy.spin(bridge)
