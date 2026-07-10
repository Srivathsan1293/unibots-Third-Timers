"""
webots_launch_fsm.py  --  Webots-simulation version of the Pi launch FSM.

This is the pi_fsm.py state machine adapted to the simulation. The real-robot
version arbitrated between a phone's seeking command and a Pi intake camera; the
sim has full ball positions and pose, so FOLLOW simply drives to the nearest
ball with go-to-goal. The launcher does not exist in Webots, so firing is a
print.

States:
  FOLLOW       drive to the nearest ball. On a collection, begin the launch.
  APPROACH_BOX if farther than launch_radius from the box, drive straight to it.
  ALIGN        rotate in place to face the box, then "launch" and resume.

Conventions (matching the rest of the sim):
  pose = (x, y, yaw), robot forward in world = (cos yaw, sin yaw),
  yaw increases turning left (CCW). Differential drive returns
  (left_vel, right_vel) wheel speeds in rad/s.

Integration (in your controller loop):

    fsm = LaunchFSM(box_position=(0.0, -1.0))   # set to YOUR scoring net
    ...
    pose = (x, y, yaw)                          # from your localizer / supervisor
    balls = tracker.confirmed_positions()       # list of (x, y) world coords
    collected = collector.check_and_collect()   # truthy on the frame a ball is taken
    (left, right) = fsm.step(pose, balls, collected, robot.getTime())
    left_motor.setVelocity(left)
    right_motor.setVelocity(right)
"""

import math


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def nearest_point(robot_xy, points):
    """Nearest (x, y) in points to robot_xy, or None if empty."""
    if not points:
        return None
    rx, ry = robot_xy
    return min(points, key=lambda p: (p[0] - rx) ** 2 + (p[1] - ry) ** 2)


class LaunchFSM:
    FOLLOW, APPROACH_BOX, ALIGN = "follow", "approach_box", "align"

    def __init__(self, box_position, *,
                 launch_radius=0.70,     # must be this close to the box to launch
                 drive_align_tol=0.35,   # rad: turn in place above this while driving
                 launch_align_tol=0.10,  # rad: "facing the box" tolerance for launch
                 arrive_tol=0.05,        # m: reached a ball
                 max_speed=6.28,         # rad/s wheel cap
                 turn_scale=0.5,         # fraction of max_speed used when turning
                 wheel_track=0.16,
                 wheel_radius=0.033):
        self.box = box_position
        self.launch_radius = launch_radius
        self.drive_align_tol = drive_align_tol
        self.launch_align_tol = launch_align_tol
        self.arrive_tol = arrive_tol
        self.max_speed = max_speed
        self.turn_scale = turn_scale
        self.wheel_track = wheel_track
        self.wheel_radius = wheel_radius

        self.state = self.FOLLOW
        self._prev_collected = False

    # --- differential-drive go-to-goal: turn in place, then drive straight ---
    def _goto(self, pose, target):
        x, y, yaw = pose
        tx, ty = target
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)
        herr = _wrap(math.atan2(dy, dx) - yaw)

        if abs(herr) > self.drive_align_tol:
            turn = self.max_speed * self.turn_scale * (1.0 if herr > 0 else -1.0)
            return (-turn, turn)                 # rotate in place (CCW if herr>0)

        # drive forward with proportional steering inside the alignment band
        fwd = self.max_speed
        steer = self.max_speed * self.turn_scale * (herr / self.drive_align_tol)
        left = fwd - steer
        right = fwd + steer
        peak = max(abs(left), abs(right))
        if peak > self.max_speed:
            left *= self.max_speed / peak
            right *= self.max_speed / peak
        return (left, right)

    def _rotate_toward(self, pose, target):
        """Rotate in place to face target. Returns (vels, facing_bool)."""
        x, y, yaw = pose
        err = _wrap(math.atan2(target[1] - y, target[0] - x) - yaw)
        if abs(err) < self.launch_align_tol:
            return (0.0, 0.0), True
        turn = self.max_speed * self.turn_scale * (1.0 if err > 0 else -1.0)
        return (-turn, turn), False

    def _dist_to_box(self, x, y):
        return math.hypot(self.box[0] - x, self.box[1] - y)

    def step(self, pose, ball_positions, collected, now=None):
        x, y, yaw = pose

        rising = bool(collected) and not self._prev_collected
        self._prev_collected = bool(collected)

        # ---- FOLLOW: drive to nearest ball; a new collection starts a launch ----
        if self.state == self.FOLLOW:
            if rising:
                if self._dist_to_box(x, y) > self.launch_radius:
                    self.state = self.APPROACH_BOX
                else:
                    self.state = self.ALIGN
                return (0.0, 0.0)
            target = nearest_point((x, y), ball_positions)
            if target is None:
                return (0.0, 0.0)                # no ball in view: hold
            dx, dy = target[0] - x, target[1] - y
            if math.hypot(dx, dy) < self.arrive_tol:
                return (0.0, 0.0)
            return self._goto(pose, target)

        # ---- APPROACH_BOX: straight line to the box until within launch_radius ----
        if self.state == self.APPROACH_BOX:
            if self._dist_to_box(x, y) <= self.launch_radius:
                self.state = self.ALIGN
                return (0.0, 0.0)
            return self._goto(pose, self.box)

        # ---- ALIGN: face the box, then launch and resume ----
        if self.state == self.ALIGN:
            vels, facing = self._rotate_toward(pose, self.box)
            if facing:
                print("BALL LAUNCHED")
                self.state = self.FOLLOW
                return (0.0, 0.0)
            return vels

        return (0.0, 0.0)