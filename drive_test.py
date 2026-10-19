#!/usr/bin/env python3
"""
drive_test.py — Fahrbefehl fuer den AEB-Test, ohne `ros2 topic pub`.

Ersetzt

    ros2 topic pub -r 10 /robot/manual_cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.15}}"

und behebt gleich drei Probleme dieses Kommandos:

  * Das YAML-Quoting der CLI ist fragil (unsichtbare typografische
    Anfuehrungszeichen aus einem Chat- oder Browserfenster reichen, damit es
    fehlschlaegt).
  * `ros2 topic pub` laeuft weiter, bis man es abbricht, und der Multiplexer
    haelt den letzten Twist fest. Bricht die Verbindung ab, faehrt der Roboter
    unbegrenzt weiter. Dieses Skript sendet beim Beenden immer eine Null.
  * Waehrend des Tests will man Fahrbefehl und AEB-Zustand *gleichzeitig*
    sehen. Das Skript abonniert /robot/aeb/status und zeigt beides in einer
    Zeile.

Aufruf
------
    python3 drive_test.py                 # 0.15 m/s, laeuft bis Strg+C
    python3 drive_test.py --speed 0.20
    python3 drive_test.py --duration 8    # stoppt nach 8 s von allein

Strg+C stoppt den Roboter sauber.
"""

# Python 3.8 (ROS2 Foxy) wertet Annotationen sofort aus, und `float | None`
# gibt es dort noch nicht. Der Future-Import macht sie zu Strings.
from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String

PUBLISH_HZ = 10.0


class DriveTest(Node):

    def __init__(self, speed: float, duration: float | None):
        super().__init__('drive_test')

        self.speed = speed
        self.duration = duration
        self.elapsed = 0.0

        self.cmd_pub = self.create_publisher(Twist, '/robot/manual_cmd_vel', 10)

        # Der AEB-Status ist waehrend des Tests die eigentliche Messgroesse.
        # Fehlt der Node, bleibt die Anzeige leer statt abzubrechen.
        self.aeb_status = '(kein /robot/aeb/status)'
        self.create_subscription(
            String, '/robot/aeb/status', self.status_callback, 10)

        self.period = 1.0 / PUBLISH_HZ
        self.timer = self.create_timer(self.period, self.tick)

        self.get_logger().info(
            f'Fahre mit {speed:.2f} m/s auf /robot/manual_cmd_vel '
            f'({PUBLISH_HZ:.0f} Hz). Strg+C stoppt.')

    def status_callback(self, msg):
        self.aeb_status = msg.data

    def tick(self):
        if self.duration is not None and self.elapsed >= self.duration:
            self.get_logger().info(f'{self.duration:.1f} s erreicht — Stopp.')
            raise KeyboardInterrupt

        cmd = Twist()
        cmd.linear.x = self.speed
        self.cmd_pub.publish(cmd)

        self.elapsed += self.period

        # Eine Zeile, die sich selbst ueberschreibt: Sollwert + AEB-Zustand.
        sys.stdout.write(
            f'\r t={self.elapsed:5.1f} s  cmd={self.speed:.2f} m/s  '
            f'| {self.aeb_status:<70}')
        sys.stdout.flush()

    def stop(self):
        """
        Nullbefehl senden, mehrfach.

        Der Multiplexer haelt den letzten empfangenen Twist fest und
        wiederholt ihn mit 10 Hz. Ein einzelnes verlorenes Stopp-Paket wuerde
        den Roboter also weiterfahren lassen — deshalb mehrere.
        """
        zero = Twist()
        for _ in range(5):
            self.cmd_pub.publish(zero)
        self.get_logger().info('Stopp gesendet (linear.x = 0).')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--speed', type=float, default=0.15,
                    help='Vorwaertsgeschwindigkeit in m/s (Standard: 0.15)')
    ap.add_argument('--duration', type=float, default=None,
                    help='nach dieser Zeit automatisch stoppen [s]')
    args = ap.parse_args()

    rclpy.init()
    node = DriveTest(args.speed, args.duration)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
