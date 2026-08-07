import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
import json

class Multiplexer(Node):
    def __init__(self):
        super().__init__('multiplexer')

        # --- SUBSCRIBER (Eingänge) ---
        self.mode_sub = self.create_subscription(String, '/robot/mode', self.mode_callback, 10)
        self.manual_estop_sub = self.create_subscription(Bool, '/robot/emergency_stop', self.manual_estop_callback, 10)
        self.lidar_estop_sub = self.create_subscription(Bool, '/robot/lidar_emergency_stop', self.lidar_estop_callback, 10)
        self.manual_sub = self.create_subscription(Twist, '/robot/manual_cmd_vel', self.manual_callback, 10)
        self.auto_sub = self.create_subscription(Twist, '/robot/autonomous_cmd_vel', self.auto_callback, 10)


        # --- PUBLISHER (Ausgänge) ---
        self.nav_pause_pub = self.create_publisher(Bool, '/robot/nav_pause', 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.feedback_pub = self.create_publisher(String, '/robot/feedback/status', 10)
        

        # --- ZUSTANDS-VARIABLEN ---
        self.mode = "MANUAL"
        self.manual_emergency_stop = False
        self.lidar_emergency_stop = False
        self.emergency_stop = False
        self.latest_manual = Twist()
        self.latest_auto = Twist()

        # Timer: Bewertet die Lage 10x pro Sekunde
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Control Node (Multiplexer) gestartet.")

    def mode_callback(self, msg):
        self.mode = msg.data
        self.update_nav_pause()

    def manual_estop_callback(self, msg):
        self.manual_emergency_stop = msg.data
        self.update_emergency_state()

    def lidar_estop_callback(self, msg):
        self.lidar_emergency_stop = msg.data
        self.update_emergency_state()

    def update_emergency_state(self):
        self.emergency_stop = self.manual_emergency_stop or self.lidar_emergency_stop
        if self.emergency_stop:
            self.get_logger().warn("NOTSTOPP AKTIV!")
        self.update_nav_pause()

    def update_nav_pause(self):
        # Deine Autonome Steuerung soll pausieren, wenn Notstopp ODER Manuell aktiv ist
        pause_msg = Bool()
        pause_msg.data = self.emergency_stop or (self.mode == "MANUAL")
        self.nav_pause_pub.publish(pause_msg)

    def manual_callback(self, msg):
        self.latest_manual = msg

    def auto_callback(self, msg):
        self.latest_auto = msg

    def control_loop(self):
        final_twist = Twist()

        # Multiplexer Logik (Prioritäten)
        if self.emergency_stop:
            final_twist.linear.x = 0.0
            final_twist.angular.z = 0.0
            active_source = "EMERGENCY_STOP"
        elif self.mode == "MANUAL":
            final_twist = self.latest_manual
            active_source = "MANUAL"
        else:
            final_twist = self.latest_auto
            active_source = "AUTO"

        # 1. Fahrbefehl an echte Motoren senden
        self.cmd_vel_pub.publish(final_twist)

        # 2. Feedback-String generieren und an die Bridge zurückschicken
        feedback_dict = {
            "mode": self.mode,
            "emergency_stop": self.emergency_stop,
            "active_source": active_source,
            "linear": final_twist.linear.x,
            "angular": final_twist.angular.z
        }
        feedback_msg = String()
        feedback_msg.data = json.dumps(feedback_dict)
        self.feedback_pub.publish(feedback_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Multiplexer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist())  # Sicherer Stopp
        node.get_logger().info("Sicherer Stopp gesendet. Node wird beendet.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()