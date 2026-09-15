"""
aeb_logic.py — der TTC-Regler des AEB.

Diese Datei ist eine 1:1-Kopie der Reglerlogik aus der BeamNG-Simulation. Sie
enthaelt bewusst keinerlei ROS- oder BeamNG-Abhaengigkeiten, sondern nur reines
Python. Dadurch laeuft in der Simulation und auf dem JetAuto exakt derselbe
Regler, und Messergebnisse aus der Simulation sind auf das reale Fahrzeug
uebertragbar.
"""

import math


class AEBController:
    """
    Time-to-Collision (TTC) based AEB controller.

    Parameters
    ----------
    ttc_warn  : TTC threshold where partial braking begins (s), default 3.0
    ttc_brake : TTC threshold for full emergency brake (s), default 1.5
    min_speed : below this speed AEB is inactive (m/s), default 1.0
    use_closing_speed : when True, base the TTC on how fast the gap to the
                        obstacle is actually shrinking (range rate) instead of the
                        ego's own speed. More realistic: a car ahead driving at the
                        same speed is not a collision course, so no braking is
                        applied. Needs the loop period passed as `dt`.
                        default False (keeps the simple ego-speed behaviour)
    """

    def __init__(self, ttc_warn: float = 3.0, ttc_brake: float = 1.5,
                 min_speed: float = 1.0, use_closing_speed: bool = False):
        self.ttc_warn  = ttc_warn
        self.ttc_brake = ttc_brake
        self.min_speed = min_speed
        self.use_closing_speed = use_closing_speed
        self._prev_dist = None      # last finite distance, for the range rate

    def compute(self, speed_mps: float, distance_m: float, dt: float = None) -> float:
        """
        Compute the required brake value.

        Parameters
        ----------
        speed_mps  : longitudinal vehicle speed in m/s (positive = forward)
        distance_m : distance to nearest obstacle ahead in metres
        dt         : time since the previous call in seconds (only needed when
                     use_closing_speed is True)

        Returns
        -------
        brake : float in [0.0, 1.0]
                0.0 = no braking, 1.0 = full emergency brake
        """
        # No usable obstacle, or too slow to bother: reset the history so the next
        # range-rate estimate does not span the gap.
        if speed_mps < self.min_speed or distance_m <= 0 or not math.isfinite(distance_m):
            self._prev_dist = distance_m if math.isfinite(distance_m) else None
            return 0.0

        # Approach rate the TTC is based on.
        if self.use_closing_speed and dt and dt > 0 and self._prev_dist is not None:
            # How fast the gap is closing (positive = getting closer).
            closing = (self._prev_dist - distance_m) / dt
            self._prev_dist = distance_m
            if closing <= 0.0:
                # Gap stable or widening -> not a collision course, don't brake.
                return 0.0
            rate = closing
        else:
            # Fall back to ego speed (first frame, or feature disabled).
            self._prev_dist = distance_m
            rate = speed_mps

        ttc = distance_m / rate

        if ttc <= self.ttc_brake:
            return 1.0
        if ttc <= self.ttc_warn:
            # Linear ramp: 0 at ttc_warn, 1 at ttc_brake
            return (self.ttc_warn - ttc) / (self.ttc_warn - self.ttc_brake)
        return 0.0

    def status(self, brake: float) -> str:
        """label for the current AEB state."""
        if brake >= 1.0:
            return "FULL BRAKE"
        if brake > 0.0:
            return f"PARTIAL BRAKE {brake:.0%}"
        return "CLEAR"
