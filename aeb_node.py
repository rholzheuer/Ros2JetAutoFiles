#!/usr/bin/env python3
"""
aeb_node.py — Automatic Emergency Braking (AEB) fuer das JetAuto unter ROS2.

Der Node nutzt exakt denselben TTC-Regler wie die BeamNG-Simulation
(aeb_logic.py). Nur die Umfelderfassung ist an die Hardware angepasst: statt
einer 3D-LiDAR-Punktwolke wird der 2D-LaserScan des JetAuto ausgewertet.

Ablauf pro Regelzyklus (20 Hz):
    1. /scan   -> Abstand zum naechsten Hindernis im Fahrkorridor vor dem Robot
    2. /odom   -> aktuelle Vorwaertsgeschwindigkeit
    3. AEBController.compute() -> Bremswert 0.0 .. 1.0 (TTC-basiert)
    4. Ausgabe:
         /robot/aeb/brake            (Float32) proportionaler Bremswert
         /robot/aeb_emergency_stop   (Bool)    True bei Vollbremsung
         /robot/aeb/status           (String)  Klartext fuer das Dashboard

Zusaetzlich zum TTC-Regler gibt es einen rastenden Abstandshalt. Der TTC-Regler
ist im Stillstand zwangslaeufig inaktiv (bei v = 0 ist eine TTC nicht
definiert), weshalb er allein den Roboter an ein Hindernis herankriechen
liesse. Der Halt ist geschwindigkeitsunabhaengig, rastet ein und loest nur,
wenn der Weg sichtbar frei ist oder /robot/aeb/reset empfangen wird.

Der Multiplexer wertet diese Topics aus: der Bremswert drosselt linear.x
proportional, der Notstopp setzt die Geschwindigkeit hart auf 0.

Wichtig: Der Node greift NICHT direkt auf /cmd_vel zu. Die Prioritaeten bleiben
vollstaendig im Multiplexer, damit es nur eine Stelle gibt, die die Motoren
kommandiert.
"""

import math
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty, Float32, String

try:
    # Normalfall: als Teil des ROS2-Packages
    from .aeb_logic import AEBController
except ImportError:
    # Fallback: Datei direkt mit python3 aeb_node.py gestartet
    from aeb_logic import AEBController


class AEBNode(Node):

    def __init__(self):
        super().__init__('aeb_node')

        # ── Parameter ────────────────────────────────────────────────────────
        # Alle Werte sind ROS2-Parameter und koennen ohne Codeaenderung per
        # Launch-File oder Kommandozeile angepasst werden. Sie entsprechen den
        # Werten aus config.py der Simulation, sind aber auf die Groesse und
        # Geschwindigkeit des JetAuto skaliert (max. ca. 0.2 m/s).

        # TTC-Schwellen des Reglers
        self.declare_parameter('ttc_warn', 2.0)    # ab hier Teilbremsung [s]
        self.declare_parameter('ttc_brake', 1.0)   # ab hier Vollbremsung [s]
        self.declare_parameter('min_speed', 0.03)  # darunter AEB inaktiv [m/s]

        # Fahrkorridor vor dem Roboter
        self.declare_parameter('corridor_half_width', 0.25)  # halbe Breite [m]
        self.declare_parameter('min_dist', 0.12)             # Eigenkontur ausblenden [m]
        self.declare_parameter('max_dist', 3.0)              # weiter weg egal [m]

        # Montage-Ausrichtung des LiDAR. 90 Grad entspricht dem Wert, der sich in
        # lidar_erkennung.py als "vorne" bewaehrt hat. Bei falscher Richtung hier
        # 0 / 90 / 180 / 270 durchprobieren.
        self.declare_parameter('front_offset_deg', 90.0)

        # Realismus-Optionen (identisch zur Simulation)
        self.declare_parameter('use_closing_speed', True)
        self.declare_parameter('brake_ramp_rate', 4.0)     # Bremseinheiten/s (0 = sofort)
        self.declare_parameter('detection_smoothing_n', 3)  # Zyklen Bestaetigung (1 = aus)

        self.declare_parameter('loop_period', 0.05)  # Regeltakt [s] -> 20 Hz

        # ── Haltezustand (Standstill Hold) ───────────────────────────────────
        # Der TTC-Regler allein laesst den Roboter an ein Hindernis heran-
        # kriechen, und zwar zwangslaeufig:
        #
        #   1. Vollbremsung -> Roboter steht.
        #   2. Im Stillstand ist speed < min_speed, der Regler ist damit
        #      inaktiv und gibt brake = 0.0 zurueck (aeb_logic.py:56). Das ist
        #      korrekt, denn eine TTC ist bei v = 0 nicht definiert.
        #   3. Der Multiplexer gibt den anliegenden Fahrbefehl wieder durch,
        #      der Roboter faehrt an.
        #   4. Unterhalb von min_dist (Eigenkontur-Filter) verschwindet das
        #      Hindernis aus dem Scan, der Abstand wird inf, es wird gar nicht
        #      mehr gebremst -- und der Roboter faehrt hinein.
        #
        # Deshalb rastet hier ein zweiter, rein abstandsbasierter Halt ein. Er
        # ist von der Geschwindigkeit unabhaengig und loest nicht von selbst.
        self.declare_parameter('hold_enabled', True)
        self.declare_parameter('hold_distance', 0.35)     # darunter rastet der Halt ein [m]

        # Freigabe: der Weg muss ueber dieser Entfernung *sichtbar* frei sein.
        # Bewusst grosszuegig, denn der Wert muss ueber dem Abstand liegen, bei
        # dem der Roboter zum Stehen kommt, sonst loest der Halt sofort wieder.
        self.declare_parameter('release_distance', 1.00)  # [m]
        self.declare_parameter('release_ticks', 10)       # 0.5 s Bestaetigung

        # inf bedeutet zweierlei: "Weg frei" ODER "Hindernis im Totbereich des
        # LiDAR". Beides ist nicht unterscheidbar. Darum zaehlt inf
        # standardmaessig NICHT als Freigabe -- sonst wuerde genau der Fall,
        # gegen den der Halt schuetzt, ihn aufheben. Freigabe dann per
        # /robot/aeb/reset oder durch Rueckwaertsfahren.
        self.declare_parameter('release_on_inf', False)

        p = self.get_parameter
        self.corridor_half_width = p('corridor_half_width').value
        self.min_dist            = p('min_dist').value
        self.max_dist            = p('max_dist').value
        self.front_offset_rad    = math.radians(p('front_offset_deg').value)
        self.brake_ramp_rate     = p('brake_ramp_rate').value
        self.period              = p('loop_period').value

        self.hold_enabled     = bool(p('hold_enabled').value)
        self.hold_distance    = float(p('hold_distance').value)
        self.release_distance = float(p('release_distance').value)
        self.release_ticks    = max(1, int(p('release_ticks').value))
        self.release_on_inf   = bool(p('release_on_inf').value)

        # ── Regler ───────────────────────────────────────────────────────────
        self.aeb = AEBController(
            ttc_warn=p('ttc_warn').value,
            ttc_brake=p('ttc_brake').value,
            min_speed=p('min_speed').value,
            use_closing_speed=p('use_closing_speed').value,
        )

        # ── Zustand ──────────────────────────────────────────────────────────
        self.latest_scan = None
        self.speed = 0.0
        self.brake_cmd = 0.0
        smoothing_n = max(1, int(p('detection_smoothing_n').value))
        self.dist_hist = deque(maxlen=smoothing_n)
        self.last_distance = float('inf')

        # Haltezustand. `hold_latched` bleibt True, bis der Weg nachweislich
        # frei ist oder per /robot/aeb/reset freigegeben wird.
        self.hold_latched = False
        self.clear_ticks = 0
        self.hold_reason = ''

        # ── Subscriber ───────────────────────────────────────────────────────
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Manuelle Freigabe des Haltezustands.
        self.reset_sub = self.create_subscription(
            Empty, '/robot/aeb/reset', self.reset_callback, 10)

        # ── Publisher ────────────────────────────────────────────────────────
        self.brake_pub  = self.create_publisher(Float32, '/robot/aeb/brake', 10)
        self.estop_pub  = self.create_publisher(Bool, '/robot/aeb_emergency_stop', 10)
        self.status_pub = self.create_publisher(String, '/robot/aeb/status', 10)

        self.timer = self.create_timer(self.period, self.control_loop)

        self.get_logger().info('AEB Node gestartet')
        self.get_logger().info(
            f'TTC warn/brake: {p("ttc_warn").value:.2f}/{p("ttc_brake").value:.2f} s, '
            f'Korridor +/-{self.corridor_half_width:.2f} m, '
            f'Reichweite {self.min_dist:.2f}-{self.max_dist:.2f} m')
        if self.hold_enabled:
            self.get_logger().info(
                f'Haltezustand aktiv: rastet unter {self.hold_distance:.2f} m '
                f'ein, Freigabe ab {self.release_distance:.2f} m '
                f'({self.release_ticks} Zyklen) oder /robot/aeb/reset')
        else:
            self.get_logger().warn(
                'Haltezustand DEAKTIVIERT — der Roboter kann nach einer '
                'Vollbremsung wieder anfahren und das Hindernis beruehren.')

    # ── Callbacks ────────────────────────────────────────────────────────────
    def scan_callback(self, msg):
        self.latest_scan = msg

    def odom_callback(self, msg):
        # Vorwaertsgeschwindigkeit aus der Odometrie. Betrag, damit ein
        # negatives Vorzeichen der Hardware das AEB nicht deaktiviert.
        self.speed = abs(msg.twist.twist.linear.x)

    def reset_callback(self, _msg):
        if self.hold_latched:
            self.get_logger().warn(
                'HALT MANUELL FREIGEGEBEN — der Weg wird nicht geprueft.')
        self.hold_latched = False
        self.clear_ticks = 0
        self.hold_reason = ''

    # ── Haltezustand ─────────────────────────────────────────────────────────
    def update_hold(self, brake_target, distance):
        """
        Rastenden Abstandshalt fuehren und zurueckgeben, ob er aktiv ist.

        Einrasten, sobald eines von beiden zutrifft:
          * der TTC-Regler fordert eine Vollbremsung, oder
          * das Hindernis ist naeher als hold_distance.

        Das erste Kriterium verhindert, dass der Roboter nach einer
        Vollbremsung ueberhaupt erst wieder anfaehrt. Das zweite greift auch
        im Stillstand, wo der TTC-Regler grundsaetzlich nichts liefert.

        Freigabe nur, wenn der Weg release_ticks Zyklen lang *sichtbar* weiter
        als release_distance frei ist. Ein inf zaehlt dabei nicht mit (siehe
        release_on_inf), weil es ein Hindernis im Totbereich des LiDAR genauso
        aussehen laesst wie eine freie Fahrbahn.
        """
        if not self.hold_enabled:
            return False

        if not self.hold_latched:
            if brake_target >= 1.0:
                self.hold_latched = True
                self.hold_reason = 'Vollbremsung'
            elif math.isfinite(distance) and distance <= self.hold_distance:
                self.hold_latched = True
                self.hold_reason = f'Hindernis {distance:.2f} m'
            if self.hold_latched:
                self.clear_ticks = 0
                self.get_logger().warn(
                    f'HALT EINGERASTET ({self.hold_reason}). Vorwaertsfahrt '
                    f'gesperrt, bis der Weg > {self.release_distance:.2f} m '
                    'frei ist oder /robot/aeb/reset kommt.')
            return self.hold_latched

        # Eingerastet: Freigabebedingung pruefen.
        clear = (math.isfinite(distance) and distance >= self.release_distance)
        if not clear and math.isinf(distance) and self.release_on_inf:
            clear = True

        self.clear_ticks = self.clear_ticks + 1 if clear else 0
        if self.clear_ticks >= self.release_ticks:
            self.hold_latched = False
            self.clear_ticks = 0
            self.hold_reason = ''
            self.get_logger().info('HALT AUFGEHOBEN — Weg ist frei.')

        return self.hold_latched

    # ── Umfelderfassung ──────────────────────────────────────────────────────
    def forward_distance(self, scan):
        """
        Kuerzester Abstand zu einem Hindernis im geraden Korridor vor dem Robot.

        Entspricht nearest_forward_obstacle() aus der Simulation, nur fuer einen
        2D-LaserScan: jeder Messstrahl wird von Polar- in Fahrzeugkoordinaten
        umgerechnet (x = vorwaerts, y = seitlich). Danach wird derselbe
        Korridor-Filter angewendet.

        Rueckgabe: Abstand in Metern, oder inf wenn der Weg frei ist.
        """
        if scan is None:
            return float('inf')

        ranges = np.asarray(scan.ranges, dtype=float)
        if ranges.size == 0:
            return float('inf')

        # Winkel jedes Strahls, korrigiert um die Einbaulage des LiDAR
        angles = (scan.angle_min
                  + np.arange(ranges.size) * scan.angle_increment
                  - self.front_offset_rad)

        # Ungueltige Messwerte (inf, nan, ausserhalb der Sensorreichweite) raus
        valid = (np.isfinite(ranges)
                 & (ranges > scan.range_min)
                 & (ranges < scan.range_max))

        x = ranges * np.cos(angles)   # vorwaerts
        y = ranges * np.sin(angles)   # seitlich

        mask = (valid
                & (x >= self.min_dist)
                & (x <= self.max_dist)
                & (np.abs(y) <= self.corridor_half_width))

        return float(np.min(x[mask])) if np.any(mask) else float('inf')

    # ── Regelzyklus ──────────────────────────────────────────────────────────
    def control_loop(self):
        dist_raw = self.forward_distance(self.latest_scan)

        # Glaettung / Bestaetigung: ein Hindernis zaehlt erst, wenn es mehrere
        # Zyklen hintereinander gesehen wurde. Als Abstand wird der Median der
        # Messungen genutzt. Das filtert einzelne Ausreisser des LiDAR.
        self.dist_hist.append(dist_raw)
        finite = [d for d in self.dist_hist if math.isfinite(d)]
        if len(finite) >= self.dist_hist.maxlen:
            distance = float(np.median(finite))
        else:
            distance = float('inf')
        self.last_distance = distance

        # TTC-Entscheidung (mit Annaeherungsgeschwindigkeit, daher dt)
        brake_target = self.aeb.compute(self.speed, distance, dt=self.period)

        # Bremsrampe: Druck wird aufgebaut statt sofort 0 -> 1 zu springen.
        # Das Loesen erfolgt sofort, damit die Fahrfreigabe sauber zurueckkommt.
        if self.brake_ramp_rate > 0 and brake_target > self.brake_cmd:
            step = self.brake_ramp_rate * self.period
            self.brake_cmd = min(brake_target, self.brake_cmd + step)
        else:
            self.brake_cmd = brake_target

        # Haltezustand hat Vorrang vor dem Regler und ueberschreibt die Rampe:
        # ein eingerasteter Halt ist keine Dosierung, sondern eine Sperre.
        holding = self.update_hold(brake_target, distance)
        if holding:
            self.brake_cmd = 1.0

        self.publish(self.brake_cmd, distance, holding)

    def publish(self, brake, distance, holding=False):
        brake_msg = Float32()
        brake_msg.data = float(brake)
        self.brake_pub.publish(brake_msg)

        # Notstopp nur bei der *dynamischen* Vollbremsung. Im eingerasteten
        # Halt bleibt er aus, denn der Multiplexer setzt bei Notstopp jede
        # Achse auf 0 -- auch Rueckwaerts und Drehen. Der Bremswert 1.0 sperrt
        # dagegen nur die Vorwaertsfahrt (multiplexer.py:109), sodass der
        # Roboter vom Hindernis wegfahren kann, aber nicht hinein.
        estop_msg = Bool()
        estop_msg.data = bool(brake >= 1.0 and not holding)
        self.estop_pub.publish(estop_msg)

        distance_str = 'inf' if math.isinf(distance) else f'{distance:.2f}'
        status_msg = String()
        state = 'HOLD' if holding else self.aeb.status(brake)
        status_msg.data = (
            f'aeb={state};'
            f'brake={brake:.2f};'
            f'distance={distance_str};'
            f'speed={self.speed:.2f}'
        )
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AEBNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('AEB Node beendet')
    finally:
        # Beim Beenden Bremse und Notstopp sauber loesen, damit das Fahrzeug
        # nicht dauerhaft blockiert bleibt.
        release = Float32()
        release.data = 0.0
        node.brake_pub.publish(release)

        estop = Bool()
        estop.data = False
        node.estop_pub.publish(estop)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
