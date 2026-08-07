import rclpy
from rclpy.node import Node

import json
import  base64
import paho.mqtt.client as mqtt
import time
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
# =========================
# MQTT CONFIG
# =========================
MQTT_BROKER = "broker.hivemq.com"
MQTT_PORT = 1883

# =========================
# MQTT Topics (EXTERN - Dashboard)
# =========================
MQTT_TOPIC_MANUAL = "dashboard/manual_cmd_vel"
MQTT_TOPIC_MODE = "dashboard/mode"
MQTT_TOPIC_EMERGENCY = "dashboard/emergency_stop"

MQTT_TOPIC_STATUS = "dashboard/feedback/status"
MQTT_TOPIC_IMAGE = "dashboard/camera/image" #

# =========================
# ROS Topics (INTERN)
# =========================
ROS_TOPIC_MANUAL = "/robot/manual_cmd_vel"
ROS_TOPIC_MODE = "/robot/mode"
ROS_TOPIC_EMERGENCY = "/robot/emergency_stop"

ROS_TOPIC_STATUS = "/robot/feedback/status"
ROS_TOPIC_IMAGE = "/robot/camera/processed"

MQTT_IMAGE_THROTTLE_SEC = 0.2  # max. 5 Bilder/Sekunde an MQTT

class MqttRosBridge(Node):

    def __init__(self):
        super().__init__('mqtt_ros_bridge')

        # =========================
        # ROS Publishers (MQTT → ROS)
        # =========================
        # Wir publishen Twist (für Fahrbefehle) und Bool (für Notstopp)
        self.pub_manual = self.create_publisher(Twist, ROS_TOPIC_MANUAL, 10)
        self.pub_mode = self.create_publisher(String, ROS_TOPIC_MODE, 10)
        self.pub_emergency = self.create_publisher(Bool, ROS_TOPIC_EMERGENCY, 10)

        # =========================
        # MQTT Client Setup
        # =========================
        self.client = mqtt.Client(
            client_id="ros2-bridge-node",
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2
        )

        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.client.connect(MQTT_BROKER, MQTT_PORT)

        self.client.loop_start()

        # =========================
        # ROS SUBSCRIBERS (ROS → MQTT)
        # =========================
        self.sub_status = self.create_subscription(
            String,
            ROS_TOPIC_STATUS,
            self.ros_status_callback,
            10
        )

        self.sub_image = self.create_subscription( 
            CompressedImage,
            ROS_TOPIC_IMAGE,
            self.ros_image_callback,
            10
        ) # Erwartetes komprimiertes Image, Callback für neue bilder

        self.get_logger().info("MQTT-ROS Bridge gestartet")
        self._last_image_publish = 0.0

    # =========================
    # MQTT CALLBACKS (Dashboard → ROS)
    # =========================
    # Wird beim erfolgreichen Verbinden mit dem MQTT-Broker aufgerufen und subscribed die relevanten Topics
    def on_connect(self, client, userdata, flags, reason_code, properties):
        self.get_logger().info("MQTT verbunden")

        client.subscribe(MQTT_TOPIC_MANUAL)
        client.subscribe(MQTT_TOPIC_MODE)
        client.subscribe(MQTT_TOPIC_EMERGENCY)

    # Verarbeitet eingehende MQTT Nachrichten und leitet sie als ROS Topics weiter (MQTT → ROS)
    def on_message(self, client, userdata, msg):

        payload = msg.payload.decode("utf-8")

        if msg.topic == MQTT_TOPIC_MANUAL:
            try:
                # Dashboard sendet JSON (z.B. {"linear": 0.5, "angular": 0.0})
                data = json.loads(payload)
                ros_msg = Twist()
                ros_msg.linear.x = float(data.get("linear", 0.0))
                ros_msg.angular.z = float(data.get("angular", 0.0))
                self.pub_manual.publish(ros_msg)
            except Exception as e:
                self.get_logger().error(f"Fehler beim Parsen von Manual Command: {e}")

        elif msg.topic == MQTT_TOPIC_MODE:
            ros_msg = String()
            ros_msg.data = payload.upper()
            self.pub_mode.publish(ros_msg)
            self.get_logger().info(f"Mode Change empfangen: {ros_msg.data}")

        elif msg.topic == MQTT_TOPIC_EMERGENCY:
            ros_msg = Bool()
            ros_msg.data = (payload.lower() == "true")
            self.pub_emergency.publish(ros_msg)

            if ros_msg.data:
                self.get_logger().warn("NOTSTOPP empfangen!")
            else:
                self.get_logger().info("Notstopp aufgehoben.")

    # =========================
    # ROS → MQTT CALLBACKS
    # =========================
    # Sendet ROS Statusdaten vom Robot zurück an das MQTT Dashboard (ROS → MQTT)
    def ros_status_callback(self, msg):
        self.client.publish(MQTT_TOPIC_STATUS, msg.data)

    def ros_image_callback(self, msg): #Bekommen JPEG-Image, in Base64 String übersetzten, im Dashboard kommt ein Bild


        now = time.time()
        if now - self._last_image_publish < MQTT_IMAGE_THROTTLE_SEC:
            return
        self._last_image_publish = now


        try:
            image_base64 = base64.b64encode(msg.data).decode("utf-8")
            self.client.publish(MQTT_TOPIC_IMAGE, image_base64)

        except Exception as e:
            self.get_logger().error(f"Image publish failed: {e}")





# =========================================================
# MAIN
# =========================================================

def main(args=None):
    rclpy.init(args=args)

    node = MqttRosBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Bridge gestoppt")
    finally:
        node.client.loop_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()