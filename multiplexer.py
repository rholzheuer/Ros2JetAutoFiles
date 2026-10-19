import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Float32
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

        # AEB: Vollbremsung wirkt wie ein Notstopp, der proportionale Bremswert
        # drosselt die Vorwaertsgeschwindigkeit linear.
        self.aeb_estop_sub = self.create_subscription(Bool, '/robot/aeb_emergency_stop', self.aeb_estop_callback, 10)
        self.aeb_brake_sub = self.create_subscription(Float32, '/robot/aeb/brake', self.aeb_brake_callback, 10)


        # --- PUBLISHER (Ausgänge) ---
        self.nav_pause_pub = self.create_publisher(Bool, '/robot/nav_pause', 10)

        # Das Ausgangs-Topic ist ein Parameter, weil es von der Plattform abhaengt:
        # der JetAuto-Chassis-Controller hoert auf /controller/cmd_vel, ein
        # Simulations- oder Testaufbau eventuell auf /cmd_vel. Ein Publisher ohne
        # Subscriber ist in ROS2 kein Fehler -- der Roboter steht dann einfach
        # stumm da. Deshalb wird das benutzte Topic beim Start geloggt.
        self.declare_parameter('cmd_vel_topic', '/controller/cmd_vel')
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.feedback_pub = self.create_publisher(String, '/robot/feedback/status', 10)


        # --- ZUSTANDS-VARIABLEN ---
        self.mode = "MANUAL"
        self.manual_emergency_stop = False
        self.lidar_emergency_stop = False
        self.aeb_emergency_stop = False
        self.aeb_brake = 0.0
        self.emergency_stop = False
        self.latest_manual = Twist()
        self.latest_auto = Twist()

        # Timer: Bewertet die Lage 10x pro Sekunde
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info(
            f"Control Node (Multiplexer) gestartet. "
            f"Fahrbefehle gehen an {cmd_vel_topic}")

    def mode_callback(self, msg):
        self.mode = msg.data
        self.update_nav_pause()

    def manual_estop_callback(self, msg):
        self.manual_emergency_stop = msg.data
        self.update_emergency_state()

    def lidar_estop_callback(self, msg):
        self.lidar_emergency_stop = msg.data
        self.update_emergency_state()

    def aeb_estop_callback(self, msg):
        self.aeb_emergency_stop = msg.data
        self.update_emergency_state()

    def aeb_brake_callback(self, msg):
        # Bremswert 0.0 .. 1.0 sicherheitshalber begrenzen
        self.aeb_brake = max(0.0, min(1.0, float(msg.data)))

    def update_emergency_state(self):
        self.emergency_stop = (self.manual_emergency_stop
                               or self.lidar_emergency_stop
                               or self.aeb_emergency_stop)
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
        else:
            if self.mode == "MANUAL":
                source_twist = self.latest_manual
                active_source = "MANUAL"
            else:
                source_twist = self.latest_auto
                active_source = "AUTO"

            # Wichtig: neues Twist bauen statt die gespeicherte Nachricht zu
            # veraendern, sonst wuerde die Drosselung dauerhaft im Zustand bleiben.
            final_twist.linear.x = source_twist.linear.x
            final_twist.linear.y = source_twist.linear.y   # Mecanum: seitlich
            final_twist.angular.z = source_twist.angular.z

            # AEB-Teilbremsung: Vorwaertsanteil linear drosseln. Rueckwaerts und
            # Drehen bleiben unangetastet, das AEB schaut nur nach vorne.
            if self.aeb_brake > 0.0 and final_twist.linear.x > 0.0:
                final_twist.linear.x *= (1.0 - self.aeb_brake)
                active_source += "+AEB"

        # 1. Fahrbefehl an echte Motoren senden
        self.cmd_vel_pub.publish(final_twist)

        # 2. Feedback-String generieren und an die Bridge zurückschicken
        feedback_dict = {
            "mode": self.mode,
            "emergency_stop": self.emergency_stop,
            "aeb_brake": round(self.aeb_brake, 2),
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