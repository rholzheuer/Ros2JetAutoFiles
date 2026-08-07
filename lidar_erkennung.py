#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


class LidarErkennung(Node):
    def __init__(self):
        super().__init__('lidar_erkennung')

        # Parameter
        self.warning_distance = 0.5  # Warnung ab 0.8 m
        self.stop_distance = 0.2  # Notbremsung ab 0.45 m
        self.front_angle_deg = 60  # vorderer Bereich: 60 Grad

        self.obstacle_blocked = False
        self.previous_blocked_state = False  # NEU: Speichert den vorherigen Zustand
        self.last_direction = "frei"
        self.last_distance = None

        # Subscriber: LiDAR-Daten
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile_sensor_data
        )

        # Publisher: Notstopp für den Control Node (Multiplexer)
        self.estop_publisher = self.create_publisher(
            Bool,
            '/robot/lidar_emergency_stop',
            10
        )

        # Publisher: Nur für die Status-Anzeige im Dashboard
        self.status_publisher = self.create_publisher(
            String,
            '/lidar_obstacle/status',
            10
        )

        # Timer: Veröffentlicht NUR noch den Status-String regelmäßig (nicht den Notstopp!)
        self.timer = self.create_timer(0.2, self.publish_status_string)

        self.get_logger().info('LiDAR Obstacle Node gestartet')
        self.get_logger().info(f'Warnabstand: {self.warning_distance:.2f} m')
        self.get_logger().info(f'Stoppabstand: {self.stop_distance:.2f} m')

    def lidar_callback(self, msg):
        sectors = self.split_scan_into_sectors(msg)
        distances = {}

        for direction, values in sectors.items():
            min_value = self.get_min_valid_distance(values, msg)
            if min_value is not None:
                distances[direction] = min_value

        if not distances:
            # Bei Datenverlust oder wenn wir uns im Blindbereich (< min_range) befinden,
            # behalten wir aus Sicherheitsgründen den vorherigen Notstopp-Zustand bei.
            if self.previous_blocked_state:
                self.get_logger().warn("Gefahr von Blindbereich oder Sensorausfall! Notstopp bleibt aktiv.")
                self.obstacle_blocked = True
            else:
                self.obstacle_blocked = False
                self.last_direction = "unbekannt"
                self.last_distance = None
            self.check_and_publish_estop()
            return

        nearest_direction = min(distances, key=distances.get)
        nearest_distance = distances[nearest_direction]

        self.last_direction = nearest_direction
        self.last_distance = nearest_distance

        # Terminal-Visualisierung (hilfreich beim Debuggen)
        self.visualize_distance(nearest_direction, nearest_distance)

        # Hindernis vorne prüfen
        front_distance = distances.get("vorne")

        if front_distance is not None and front_distance < self.stop_distance:
            self.obstacle_blocked = True
        elif nearest_distance < self.warning_distance:
            self.obstacle_blocked = False
            self.get_logger().debug(
                f'WARNUNG: Hindernis {nearest_direction}! Abstand: {nearest_distance:.2f} m'
            )
        else:
            self.obstacle_blocked = False

        # NEU: Prüfen, ob sich der Zustand geändert hat und ggf. an Control Node senden
        self.check_and_publish_estop()

    def check_and_publish_estop(self):
        """
        Sendet den Notstopp-Befehl NUR, wenn sich der Zustand geändert hat.
        Verhindert, dass der LiDAR einen manuellen Notstopp aus dem Dashboard überschreibt.
        """
        if self.obstacle_blocked != self.previous_blocked_state:
            estop_msg = Bool()
            estop_msg.data = self.obstacle_blocked
            self.estop_publisher.publish(estop_msg)

            if self.obstacle_blocked:
                self.get_logger().error(f'NOTBREMSE AKTIVIERT: Hindernis vorne ({self.last_distance:.2f} m)!')
            else:
                self.get_logger().info('NOTBREMSE AUFGEHOBEN: Weg ist wieder frei.')

            self.previous_blocked_state = self.obstacle_blocked

    def split_scan_into_sectors(self, msg):
        ranges = list(msg.ranges)
        number_of_points = len(ranges)

        if number_of_points == 0:
            return {"vorne": [], "links": [], "rechts": []}

        angle_increment_deg = math.degrees(msg.angle_increment)
        if angle_increment_deg <= 0:
            angle_increment_deg = 1.0

        # Ändere diesen Wert auf 90, 180 oder 270, bis ein Hindernis VOR dem Auto auch im Terminal als "vorne" erkannt wird.
        offset_deg = 90

        offset_index = int(offset_deg / angle_increment_deg)

        ranges = ranges[offset_index:] + ranges[:offset_index]

        # --- Normale Sektor-Aufteilung ---
        half_front_points = int((self.front_angle_deg / 2) / angle_increment_deg)
        half_front_points = max(1, min(half_front_points, number_of_points // 4))

        front = ranges[0:half_front_points] + ranges[-half_front_points:]

        left_start = half_front_points
        left_end = number_of_points // 2

        right_start = number_of_points // 2
        right_end = number_of_points - half_front_points

        left = ranges[left_start:left_end]
        right = ranges[right_start:right_end]

        return {
            "vorne": front,
            "links": left,  # Ändern in 'right', falls spiegelverkehrt
            "rechts": right  # Ändern in 'left', falls spiegelverkehrt
        }

    def get_min_valid_distance(self, values, msg):
        valid_values = [
            distance for distance in values
            if math.isfinite(distance)
               and distance > msg.range_min
               and distance < msg.range_max
        ]

        if not valid_values:
            return None

        return min(valid_values)

    def visualize_distance(self, direction, distance):
        bar_length = int(distance * 20)
        bar_length = max(1, min(bar_length, 40))
        bar = "#" * bar_length
        # Nutze Debug-Level für den Bar, damit das Terminal nicht völlig überflutet wird
        self.get_logger().debug(f'Hindernis {direction:<6}: [{bar:<40}] {distance:.2f} m')

    def publish_status_string(self):
        status_msg = String()

        if self.last_distance is None:
            status_msg.data = "status=unknown;direction=unknown;distance=nan;blocked=false"
        else:
            status_msg.data = (
                f'status=obstacle;'
                f'direction={self.last_direction};'
                f'distance={self.last_distance:.2f};'
                f'blocked={str(self.obstacle_blocked).lower()}'
            )

        self.status_publisher.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarErkennung()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('LiDAR Obstacle Node beendet')
    finally:
        # Sicherheitshalber beim Beenden den Notstopp aufheben
        estop_msg = Bool()
        estop_msg.data = False
        node.estop_publisher.publish(estop_msg)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()