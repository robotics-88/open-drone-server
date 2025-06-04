import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64, String
from sensor_msgs.msg import NavSatFix
import threading

class DroneBridge(Node):
    def __init__(self):
        super().__init__('drone_bridge')

        self.capabilities = ''
        self.heading = 0.0
        self.lat = 0.0
        self.lon = 0.0

        self.run_mission_publisher = self.create_publisher(
            String,
            '/frontend/run_mission',
            10
        )

        self.create_subscription(
            String,
            '/task_manager/capabilities',
            self.capabilities_cb,
            10
        )

        self.create_subscription(
            Float64,
            '/mavros/global_position/compass_hdg',
            self.heading_cb,
            qos_profile_sensor_data
        )

        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.global_cb,
            qos_profile_sensor_data
        )


    def capabilities_cb(self, msg):
        self.capabilities = msg.data

    def heading_cb(self, msg):
        self.heading = msg.data

    def global_cb(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude

    def run_mission(self, mission_payload):
        msg = String()
        msg.data = mission_payload
        self.run_mission_publisher.publish(msg)

bridge = None

def start_ros():
    global bridge
    rclpy.init()
    bridge = DroneBridge()
    rclpy.spin(bridge)
