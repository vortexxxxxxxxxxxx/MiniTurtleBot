import math
import threading
from dataclasses import dataclass
from typing import Optional, Tuple

# Geometry and encoder constants (from base_setup and square/follow_line examples)
TICKS_PER_REV = 12 * 4  # quadrature decoding
GEAR_RATIO = 30.0
WHEEL_RADIUS_M = 0.0325  # 65mm diameter wheels
TRACK_WIDTH_M = 0.10     # distance between wheels
WHEEL_CIRCUM_M = 2.0 * math.pi * WHEEL_RADIUS_M

# Encoder direction multipliers (flip if wiring yields opposite sign)
ENC_A_DIR = 1  # Right wheel
ENC_B_DIR = 1  # Left wheel


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # radians


class DifferentialOdometry:
    """Tracks pose (x, y, theta) from incremental encoder counts.

    Provide absolute encoder counts for wheel A (right) and wheel B (left).
    The class computes deltas internally and integrates pose using midpoint method.
    """

    def __init__(self):
        self.pose = Pose()
        self._last_counts: Optional[Tuple[int, int]] = None
        self._lock = threading.Lock()

    def reset(self) -> None:
        with self._lock:
            self.pose = Pose()
            self._last_counts = None

    def update_from_counts(self, count_a: int, count_b: int) -> Pose:
        """Update odometry given absolute encoder counts.

        Returns the updated pose.
        """
        with self._lock:
            if self._last_counts is None:
                self._last_counts = (count_a, count_b)
                return self.pose

            dA = int(count_a) - int(self._last_counts[0])
            dB = int(count_b) - int(self._last_counts[1])
            self._last_counts = (count_a, count_b)

            if dA == 0 and dB == 0:
                return self.pose

            revA = dA / float(TICKS_PER_REV * GEAR_RATIO)
            revB = dB / float(TICKS_PER_REV * GEAR_RATIO)
            dr = (revA * WHEEL_CIRCUM_M) * ENC_A_DIR  # right wheel distance (m)
            dl = (revB * WHEEL_CIRCUM_M) * ENC_B_DIR  # left wheel distance (m)

            dS = 0.5 * (dr + dl)
            dTh = (dr - dl) / TRACK_WIDTH_M

            # Midpoint integration for better accuracy on turns
            self.pose.x += dS * math.cos(self.pose.theta + 0.5 * dTh)
            self.pose.y += dS * math.sin(self.pose.theta + 0.5 * dTh)
            self.pose.theta += dTh

            return self.pose

    def get_pose(self) -> Pose:
        with self._lock:
            return Pose(self.pose.x, self.pose.y, self.pose.theta)
