#!/usr/bin/env python3
"""
fake_scan.py — Pruefstand fuer das AEB ohne reale Hardware.

Der Node ersetzt das JetAuto durch ein virtuelles Fahrzeug: er veroeffentlicht
einen synthetischen 2D-LaserScan auf /scan und die passende Odometrie auf /odom.
Simuliert wird die Annaeherung an eine Wand, die in einem einstellbaren Abstand
vor dem Fahrzeug steht.

Der Pruefstand ist ein geschlossener Regelkreis. Er liest den Fahrbefehl, den
der Multiplexer ausgibt, zurueck und bewegt das virtuelle Fahrzeug damit:

    fake_scan  --/scan, /odom-->  aeb_node  --/robot/aeb/brake-->  multiplexer
        ^                                                               |
        +--------------------- /controller/cmd_vel --------------------+

Dadurch wirkt die Bremsung tatsaechlich auf die simulierte Bewegung zurueck,
und es laesst sich messen, bei welchem Restabstand das Fahrzeug steht. Der
Versuch ist exakt wiederholbar, weil Startabstand und Wunschgeschwindigkeit
feste Parameter sind.

Start (in einem eigenen Terminal, nach source install/setup.bash):

    python3 fake_scan.py

Mit abweichendem Startabstand und ohne Rueckkopplung (konstante Fahrt):

    python3 fake_scan.py --ros-args \\
        -p start_distance:=5.0 -p closed_loop:=false -p fixed_speed:=0.2

Neustart des Versuchs ohne den Node zu beenden:

    ros2 topic pub --once /fake_scan/reset std_msgs/msg/Empty {}
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty


class FakeScanNode(Node):

    def __init__(self):
        super().__init__('fake_scan')

        # ── Szenario ─────────────────────────────────────────────────────────
        self.declare_parameter('start_distance', 3.0)    # Abstand zur Wand beim Start [m]
        self.declare_parameter('obstacle_width', 1.0)    # Breite der Wand [m]
        self.declare_parameter('fixed_speed', 0.2)       # Wunschgeschwindigkeit [m/s]

        # closed_loop=True : Geschwindigkeit kommt aus dem Fahrbefehl des
        #                    Multiplexers, das AEB bremst das Fahrzeug wirklich.
        # closed_loop=False: konstante Fahrt mit fixed_speed, unabhaengig vom
        #                    AEB. Nuetzlich, um nur die Bremswerte zu beobachten.
        self.declare_parameter('closed_loop', True)
        self.declare_parameter('cmd_vel_topic', '/controller/cmd_vel')

        # ── Sensormodell ─────────────────────────────────────────────────────
        # Werte eines typischen 360-Grad-LiDAR, wie er am JetAuto verbaut ist.
        self.declare_parameter('num_rays', 360)
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 8.0)
        self.declare_parameter('noise_stddev', 0.0)      # Messrauschen [m]

        # Muss mit dem gleichnamigen Parameter des aeb_node uebereinstimmen,
        # sonst sucht das AEB die Wand in der falschen Richtung.
        self.declare_parameter('front_offset_deg', 90.0)

        self.declare_parameter('rate', 20.0)             # Takt [Hz]

        p = self.get_parameter
        self.start_distance = float(p('start_distance').value)
        self.obstacle_width = float(p('obstacle_width').value)
        self.fixed_speed    = float(p('fixed_speed').value)
        self.closed_loop    = bool(p('closed_loop').value)
        self.num_rays       = int(p('num_rays').value)
        self.range_min      = float(p('range_min').value)
        self.range_max      = float(p('range_max').value)
        self.noise_stddev   = float(p('noise_stddev').value)
        self.front_offset   = math.radians(float(p('front_offset_deg').value))
        self.period         = 1.0 / float(p('rate').value)

        # Strahlwinkel im Sensorrahmen, einmal vorberechnet
        self.angle_min       = -math.pi
        self.angle_increment = 2.0 * math.pi / self.num_rays
        scan_angles = self.angle_min + np.arange(self.num_rays) * self.angle_increment
        # Winkel im Fahrzeugrahmen: 0 = geradeaus. Das Vorzeichen muss zur
        # Umrechnung im aeb_node passen, die den Offset wieder abzieht.
        self.ray_angles = scan_angles - self.front_offset

        # ── Zustand ──────────────────────────────────────────────────────────
        self.distance = self.start_distance
        self.speed = 0.0
        self.travelled = 0.0
        self.cmd_speed = 0.0
        self.reported_stop = False
        self.reported_crash = False
        self.log_counter = 0

        # ── Schnittstellen ───────────────────────────────────────────────────
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        cmd_topic = p('cmd_vel_topic').value
        self.cmd_sub = self.create_subscription(Twist, cmd_topic, self.cmd_callback, 10)
        self.reset_sub = self.create_subscription(Empty, '/fake_scan/reset', self.reset_callback, 10)

        self.timer = self.create_timer(self.period, self.step)

        mode = 'geschlossener Regelkreis' if self.closed_loop else 'konstante Fahrt'
        self.get_logger().info('Pruefstand gestartet — Fahrzeug und LiDAR sind simuliert')
        self.get_logger().info(
            f'Modus: {mode}, Startabstand {self.start_distance:.2f} m, '
            f'Wunschgeschwindigkeit {self.fixed_speed:.2f} m/s')
        self.get_logger().info(f'Fahrbefehl wird gelesen von: {cmd_topic}')

    # ── Callbacks ────────────────────────────────────────────────────────────
    def cmd_callback(self, msg):
        self.cmd_speed = float(msg.linear.x)

    def reset_callback(self, _msg):
        self.distance = self.start_distance
        self.travelled = 0.0
        self.speed = 0.0
        self.reported_stop = False
        self.reported_crash = False
        self.get_logger().info(f'Versuch zurueckgesetzt — Abstand {self.distance:.2f} m')

    # ── Sensormodell ─────────────────────────────────────────────────────────
    def build_ranges(self):
        """
        Erzeugt die Entfernungswerte fuer eine flache Wand im Abstand
        self.distance vor dem Fahrzeug.

        Fuer jeden Strahl mit Winkel t gilt: der Strahl trifft die Wand bei
        x = distance, also y = distance * tan(t) und r = distance / cos(t).
        Strahlen, die nach hinten zeigen oder seitlich an der Wand vorbeigehen,
        liefern keinen Messwert. Als Kennzeichen dafuer wird 0.0 gesendet, denn
        genau so meldet der LiDAR des JetAuto ungueltige Strahlen (siehe den
        Filter r > 0.0 in object_detection.py aus dem Praktikum).
        """
        ranges = np.zeros(self.num_rays)

        if self.distance <= 0.0:
            return ranges

        cos_t = np.cos(self.ray_angles)
        forward = cos_t > 1e-6                       # nur Strahlen nach vorne

        r = np.zeros(self.num_rays)
        r[forward] = self.distance / cos_t[forward]
        y = r * np.sin(self.ray_angles)

        hit = (forward
               & (np.abs(y) <= self.obstacle_width / 2.0)
               & (r > self.range_min)
               & (r < self.range_max))

        ranges[hit] = r[hit]

        if self.noise_stddev > 0.0:
            noise = np.random.normal(0.0, self.noise_stddev, self.num_rays)
            ranges[hit] += noise[hit]

        return ranges

    # ── Simulationsschritt ───────────────────────────────────────────────────
    def step(self):
        # Geschwindigkeit bestimmen. Im geschlossenen Regelkreis kommt sie aus
        # dem Fahrbefehl, den der Multiplexer nach der AEB-Drosselung ausgibt.
        if self.closed_loop:
            self.speed = max(0.0, self.cmd_speed)
        else:
            self.speed = self.fixed_speed if self.distance > 0.0 else 0.0

        # Fahrzeug bewegen, der Abstand zur Wand schrumpft entsprechend
        step_len = self.speed * self.period
        self.distance = max(0.0, self.distance - step_len)
        self.travelled += step_len

        self.publish_scan()
        self.publish_odom()
        self.report()

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_min + (self.num_rays - 1) * self.angle_increment
        msg.angle_increment = self.angle_increment
        msg.time_increment = 0.0
        msg.scan_time = self.period
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = [float(r) for r in self.build_ranges()]
        self.scan_pub.publish(msg)

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.travelled
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x = self.speed
        self.odom_pub.publish(msg)

    def report(self):
        # Laufende Ausgabe mit 2 Hz, damit das Terminal lesbar bleibt
        self.log_counter += 1
        if self.log_counter % max(1, int(0.5 / self.period)) == 0:
            self.get_logger().info(
                f'Abstand {self.distance:5.2f} m   '
                f'v {self.speed:4.2f} m/s   '
                f'gefahren {self.travelled:5.2f} m')

        # Das eigentliche Messergebnis: Restabstand beim Stillstand
        if self.speed < 0.01 and self.travelled > 0.05 and not self.reported_stop:
            self.reported_stop = True
            self.get_logger().info(
                f'FAHRZEUG STEHT — Restabstand {self.distance:.2f} m '
                f'nach {self.travelled:.2f} m Fahrweg')
        elif self.speed >= 0.01:
            self.reported_stop = False

        if self.distance <= 0.0 and not self.reported_crash:
            self.reported_crash = True
            self.get_logger().warn('KOLLISION — das Fahrzeug hat die Wand erreicht')


def main(args=None):
    rclpy.init(args=args)
    node = FakeScanNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Pruefstand beendet')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
