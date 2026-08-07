import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
import yaml
import math
import os
import json



class AutonomeSteuerung(Node):
    SPEED_SCALE = 0.5       # Skalierungsfaktor: linear.x = SPEED_SCALE * distance_to_target
    MAX_FORWARD_SPEED = 0.2  # Maximale Vorwärtsgeschwindigkeit in m/s (Hard Cap)
    TURN_SPEED = 1.57

    def __init__(self):
        super().__init__("autonome_steuerung")

        self.publisher_ = self.create_publisher(
            Twist,
            "/robot/autonomous_cmd_vel",
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        self.pause_sub = self.create_subscription(
            Bool,
            "/robot/nav_pause",
            self.pause_callback,
            10
        )

        self.stopsign_sub = self.create_subscription(
            String,
            "/robot/stopsign",
            self.stopsign_callback,
            10
        )

        self.yaml_path = "/home/pi/ros2_ws/src/remote_robot_control/remote_robot_control/route.yaml"
        if not os.path.exists(self.yaml_path):
            local_path = os.path.join(os.path.dirname(__file__), "route.yaml")
            if os.path.exists(local_path):
                self.yaml_path = local_path
        
        self.route = self.load_route()
        self.current_step_index = 0

        self.initial_x = None
        self.initial_y = None
        self.initial_yaw = None

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.is_paused = True

        self.dist_tolerance = 0.08

        # --- Stoppschild-Steuerung ---

        self.stopsign_state = "DRIVING"  # DRIVING, DECELERATING, STOPPED, COOLDOWN
        self.stop_start_time = None
        self.cooldown_start_time = None
        self.last_stopsign_time = None
        self.stopsign_distance = None
        self.stopsign_visible = False

        self.STOP_TRIGGER_DISTANCE = 0.8  # m (ab dieser Entfernung abbremsen)
        self.STOP_THRESHOLD = 0.35         # m (ab dieser Entfernung stoppen)
        self.STOP_DURATION = 3.0          # Sek. (Wartezeit)
        self.COOLDOWN_DURATION = 5.0      # Sek. (Ignorierzeit nach Anfahren)
        self.STOPSIGN_TIMEOUT = 1.0       # Sek. (Toleranz für Signalverlust)

        self.get_logger().info(
            "Autonomous Node gestartet. Warte auf erste Sensordaten zur Kalibrierung..."
        )

    def load_route(self):
        try:
            with open(self.yaml_path, "r") as file:
                data = yaml.safe_load(file)

            route = data.get("route", [])

            if not route:
                self.get_logger().warn("Route ist leer oder nicht vorhanden.")

            return route

        except Exception as error:
            self.get_logger().error(f"Konnte YAML nicht laden: {error}")
            return []

    def pause_callback(self, msg):
        self.is_paused = msg.data

        if self.is_paused:
            self.get_logger().warn("Navigation PAUSIERT. Sende Stop an Robot Control Node.")
        else:
            self.get_logger().info("Navigation FORTGESETZT.")

    def stopsign_callback(self, msg):
        try:
            data = json.loads(msg.data)
            detected = data.get("detected", False)
            distance = data.get("distance")

            if detected:
                self.last_stopsign_time = self.get_clock().now()
                self.stopsign_distance = distance
                self.stopsign_visible = True
            else:
                self.stopsign_visible = False

        except Exception as error:
            self.get_logger().error(f"Fehler beim Parsen von Stopsign-Daten: {error}")

    def odom_callback(self, msg):
        # 1. Rohe Hardware-Daten auslesen
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y

        #4D vektor zu einfachen Winkel im Bogenmaß -180Grad bis +180Grad
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        raw_yaw = math.atan2(siny_cosp, cosy_cosp)

        # 2. Tara-Initialisierung (Passiert nur 1x beim Start der Node)
        if self.initial_x is None:
            self.initial_x = raw_x
            self.initial_y = raw_y
            self.initial_yaw = raw_yaw
            self.get_logger().info("Kalibrierung abgeschlossen! Virtueller Nullpunkt (0,0) gesetzt.")
            return  # Breche diesen Durchlauf ab, starte ab dem nächsten

        # 3. Koordinatensystem verschieben (Translation) dx = zurückgelegte strecke von jetauto startung (nicht von tara 0,0)
        dx = raw_x - self.initial_x
        dy = raw_y - self.initial_y

        # 4. Koordinatensystem virtuell zurückdrehen für die bestimmung von current auf basis von tara
        theta = -self.initial_yaw
        self.current_x = dx * math.cos(theta) - dy * math.sin(theta)
        self.current_y = dx * math.sin(theta) + dy * math.cos(theta)

        # 5. Blickrichtung bereinigen und auf -180 bis +180 normalisieren
        self.current_yaw = raw_yaw - self.initial_yaw
        self.current_yaw = math.atan2(math.sin(self.current_yaw), math.cos(self.current_yaw))

        # --- Ab hier läuft die normale Routen-Logik ---

        if self.is_paused:
            return

        # --- Stoppschild Zustandsautomat ---
        now = self.get_clock().now()

        # Prüfen, ob das Stoppschild sichtbar und nicht veraltet ist
        is_stopsign_active = False
        if self.stopsign_visible and self.last_stopsign_time is not None:
            elapsed_since_last_detection = (now - self.last_stopsign_time).nanoseconds / 1e9
            if elapsed_since_last_detection < self.STOPSIGN_TIMEOUT:
                is_stopsign_active = True

        # Zustand: DRIVING
        if self.stopsign_state == "DRIVING":
            if is_stopsign_active and self.stopsign_distance is not None:
                if self.stopsign_distance < self.STOP_TRIGGER_DISTANCE:
                    self.stopsign_state = "DECELERATING"
                    self.get_logger().info(
                        f"Stoppschild erkannt bei {self.stopsign_distance:.2f}m. Beginne kontrolliertes Abbremsen."
                    )

        # Zustand: DECELERATING
        elif self.stopsign_state == "DECELERATING":
            if not is_stopsign_active:
                self.stopsign_state = "DRIVING"
                self.get_logger().info("Stoppschild nicht mehr erkannt (Timeout). Setze normale Fahrt fort.")
            elif self.stopsign_distance is not None and self.stopsign_distance <= self.STOP_THRESHOLD:
                self.stopsign_state = "STOPPED"
                self.stop_start_time = now
                self.get_logger().warn(
                    f"Stoppschild-Haltelinie erreicht ({self.stopsign_distance:.2f}m). Stoppe für {self.STOP_DURATION}s."
                )

        # Zustand: STOPPED
        elif self.stopsign_state == "STOPPED":
            elapsed_stop = (now - self.stop_start_time).nanoseconds / 1e9
            if elapsed_stop >= self.STOP_DURATION:
                self.stopsign_state = "COOLDOWN"
                self.cooldown_start_time = now
                self.get_logger().info(
                    f"Wartezeit von {self.STOP_DURATION}s abgelaufen. Fahre fort. Cooldown gestartet."
                )
            else:
                # Sende Nullgeschwindigkeit während des Stopps
                self.publisher_.publish(Twist())
                return

        # Zustand: COOLDOWN
        elif self.stopsign_state == "COOLDOWN":
            elapsed_cooldown = (now - self.cooldown_start_time).nanoseconds / 1e9
            if elapsed_cooldown >= self.COOLDOWN_DURATION:
                self.stopsign_state = "DRIVING"
                self.get_logger().info("Stoppschild-Cooldown beendet. Überwachung wieder aktiv.")

        if not self.route:
            self.publisher_.publish(Twist())
            self.get_logger().warn("Keine Route geladen. Stoppe autonome Bewegung.")
            return

        if self.current_step_index >= len(self.route):
            self.get_logger().info("Route komplett abgefahren. Starte wieder von vorne.")
            self.current_step_index = 0

        target = self.route[self.current_step_index]

        target_x = target["x"]
        target_y = target["y"]

        #x und y entfernung
        inc_x = target_x - self.current_x
        inc_y = target_y - self.current_y

        #luftlinie berechnung, schnellster weg und der winkel vom aktuellen stand vom roboter
        distance_to_target = math.hypot(inc_x, inc_y)
        angle_to_target = math.atan2(inc_y, inc_x)

        angle_diff = angle_to_target - self.current_yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        twist = Twist()

        if distance_to_target < self.dist_tolerance:
            self.get_logger().info(
                f"Wegpunkt {self.current_step_index} erreicht."
            )

            self.current_step_index += 1
            self.publisher_.publish(Twist())
            return

        #Wenn winkel größer ist als ca.  Grad, dann auf der Stelle drehen, weil genauer als fahren und drehen
        if abs(angle_diff) > 0.5:
            twist.linear.x = 0.0
            twist.angular.z = self.TURN_SPEED * angle_diff
        else:
            twist.linear.x = min(
                self.SPEED_SCALE * distance_to_target,  #wenn target näher kommt, wird er langsamer (kommt auf die festlegung der parameter an)
                self.MAX_FORWARD_SPEED
            )
            twist.angular.z = self.TURN_SPEED * angle_diff

        # Falls wir im Zustand DECELERATING sind, linear UND angular proportional drosseln
        if self.stopsign_state == "DECELERATING" and self.stopsign_distance is not None:
            numerator = max(0.0, self.stopsign_distance - self.STOP_THRESHOLD)
            denominator = self.STOP_TRIGGER_DISTANCE - self.STOP_THRESHOLD
            scale = min(1.0, numerator / denominator) if denominator > 0.0 else 0.0
            twist.linear.x = twist.linear.x * scale
            twist.angular.z = twist.angular.z * scale  # Drehmomentkontrolle beim Abbremsen

        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    node = AutonomeSteuerung()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.publisher_.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()