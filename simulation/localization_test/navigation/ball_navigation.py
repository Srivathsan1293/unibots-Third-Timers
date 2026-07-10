"""
ball_navigator.py
Motion control and destination selection, kept separate.

  GoToPoint.step(pose, destination)  -> ((left_vel, right_vel), arrived)
      Pure go-to-goal for a differential-drive robot. It only drives the robot
      toward a fixed (x, y). No ball logic, no capture. Reports `arrived` when
      it reaches the point (and then commands zero velocity).

  nearest_point(robot_xy, points)    -> (x, y) or None
      The destination chooser, swappable. For now it returns the nearest point.
      Replace this later (value-weighted, route-optimised, etc.) without
      touching the controller.

Frame convention: robot forward in the world is (cos yaw, sin yaw), so the
bearing error to a destination is wrap(atan2(dy, dx) - yaw).
"""

import math


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def nearest_point(robot_xy, points):
    """Destination selector. Returns the nearest point to robot_xy, or None."""
    best, best_d = None, float("inf")
    for p in points:
        d = math.hypot(p[0] - robot_xy[0], p[1] - robot_xy[1])
        if d < best_d:
            best, best_d = p, d
    return best


class BallNavigator:
    def __init__(self,
                 wheel_radius=0.033,   # m  (set to your robot)
                 wheel_track=0.160,    # m  distance between wheels
                 max_lin=0.15,         # m/s forward cap
                 max_ang=3.0,          # rad/s turn-rate cap
                 k_lin=1.5,            # forward gain
                 k_ang=4.0,            # steering gain
                 align_tol=0.35,       # rad: turn in place until aimed within this
                 arrive_tol=0.05,      # m: consider the destination reached within this
                 max_wheel_speed=None):

        self.r = wheel_radius
        self.track = wheel_track
        self.max_lin = max_lin
        self.max_ang = max_ang
        self.k_lin = k_lin
        self.k_ang = k_ang
        self.align_tol = align_tol
        self.arrive_tol = arrive_tol
        self.max_wheel = max_wheel_speed or (max_lin / wheel_radius) * 1.5

    def _wheels(self, v, w):
        wl = (v - w * self.track / 2.0) / self.r
        wr = (v + w * self.track / 2.0) / self.r
        peak = max(abs(wl), abs(wr), 1e-9)
        if peak > self.max_wheel:
            wl *= self.max_wheel / peak
            wr *= self.max_wheel / peak
        return wl, wr

    def step(self, pose, destination):
        """Drive toward a fixed destination. Returns ((left_vel, right_vel), arrived)."""
        if destination is None:
            return (0.0, 0.0), False
        rx, ry, yaw = pose
        dx, dy = destination[0] - rx, destination[1] - ry
        dist = math.hypot(dx, dy)

        if dist < self.arrive_tol:
            return (0.0, 0.0), True

        heading_err = _wrap(math.atan2(dy, dx) - yaw)
        if abs(heading_err) > self.align_tol:     # turn in place until aimed
            v = 0.0
        else:                                     # aimed: drive, easing in near the goal
            v = _clamp(self.k_lin * dist, 0.0, self.max_lin)
        w = _clamp(self.k_ang * heading_err, -self.max_ang, self.max_ang)
        return self._wheels(v, w), False
