"""
ball_tracker.py
===============
A short-term memory for detected balls. Detections flicker because the detector
misses a frame here and there; this tracker keeps a stable set of balls so the
overlay does not blink, and builds a persistent world-frame map of where the
balls are.

WHY WORLD COORDINATES, NOT BOXES
    The balls are static; only the robot moves, and the robot pose is known. A
    ball stored in world coordinates is therefore not invalidated when the robot
    drives or turns. To draw its box in the current frame we reproject the world
    point back into the image using the current pose. Storing boxes instead would
    force a reset on every motion, which is exactly when the memory is needed.

LIFECYCLE OF A TRACK
    new detection            -> tentative track (not drawn yet)
    seen `confirm_hits` times -> confirmed track (drawn, part of the map)
    matched again            -> position refined by EMA, miss counter cleared
    in view but not matched  -> miss counter grows; deleted after a limit
                                (this is the "sweep an empty spot" case)
    out of view, not matched -> kept untouched (still part of the map)

The deletion limit is larger for confirmed tracks (bridges the flicker) than for
tentative ones (kills one-frame false positives quickly).

COORDINATE FRAMES
    Body frame: +right, +forward, origin under the camera (what GroundProjector
    returns). World frame: ENU ground plane (x, y), the same frame your
    ground-truth balls live in. Conversions use the robot pose (x, y, yaw) and
    the camera mount offset.
"""

import math
from dataclasses import dataclass

from perception.detector import BBox


@dataclass
class BallTrack:
    """One remembered ball, in world coordinates."""
    id: int
    x: float
    y: float
    hits: int = 1               # times matched
    missed_visible: int = 0     # consecutive misses while inside the FOV
    confirmed: bool = False
    radius_px: float = 0.0      # last observed pixel radius (for redraw sizing)
    range_at_obs: float = 1.0   # range when that radius was observed


class BallTracker:
    def __init__(self, projector,
                 assoc_gate=0.08,          # m; > per-frame noise, < ball spacing
                 ema_alpha=0.35,           # position smoothing (0=frozen,1=raw)
                 confirm_hits=3,           # detections before a track is trusted
                 max_missed=8,             # in-view misses before a CONFIRMED track dies
                 tentative_max_missed=2,   # in-view misses before a TENTATIVE track dies
                 edge_margin=3,            # px inward; balls nearer the edge are not penalized
                 cam_offset_forward=0.0,   # camera position in the robot body frame, m
                 cam_offset_right=0.0):
        self.proj = projector
        self.assoc_gate = assoc_gate
        self.ema = ema_alpha
        self.confirm_hits = confirm_hits
        self.max_missed = max_missed
        self.tentative_max_missed = tentative_max_missed
        self.edge_margin = edge_margin
        self.cam_off_fwd = cam_offset_forward
        self.cam_off_right = cam_offset_right
        self.tracks: list[BallTrack] = []
        self._next_id = 0

    # ------------------------------------------------------------------
    # Frame transforms
    # ------------------------------------------------------------------
    def _camera_origin(self, robot_x, robot_y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        cx = robot_x + self.cam_off_fwd * c + self.cam_off_right * s
        cy = robot_y + self.cam_off_fwd * s - self.cam_off_right * c
        return cx, cy

    @staticmethod
    def _body_to_world(origin, yaw, right, forward):
        c, s = math.cos(yaw), math.sin(yaw)
        return (origin[0] + forward * c + right * s,
                origin[1] + forward * s - right * c)

    @staticmethod
    def _world_to_body(origin, yaw, wx, wy):
        dx, dy = wx - origin[0], wy - origin[1]
        c, s = math.cos(yaw), math.sin(yaw)
        forward = dx * c + dy * s
        right = dx * s - dy * c
        return right, forward

    def _visible(self, right, forward):
        """Is a body-frame point inside the current image? Returns (bool, (u,v))."""
        if forward <= 0:
            return False, None
        uv = self.proj.forward_project(right, forward)   # (right, forward) -> pixel
        if uv is None:
            return False, None
        u, v = uv
        m = self.edge_margin
        if m <= u <= self.proj.width - 1 - m and m <= v <= self.proj.height - 1 - m:
            return True, (u, v)
        return False, None

    # ------------------------------------------------------------------
    # Main update
    # ------------------------------------------------------------------
    def update(self, detections, robot_pose):
        """Advance the tracker one frame.

        detections : list of BBox from your detector for this frame.
        robot_pose : (robot_x, robot_y, yaw) in the world ground frame.

        Returns the list of (BBox, (world_x, world_y)) for confirmed tracks that
        fall inside the current frame, ready to draw. Call confirmed_positions()
        for the full map.
        """
        robot_x, robot_y, yaw = robot_pose
        cam = self._camera_origin(robot_x, robot_y, yaw)

        # 1) Localize this frame's detections into the world.
        obs = []
        for d in detections:
            rel = self.proj.ball_ground_position(d)    # (right, forward) or None
            if rel is None:
                continue
            right, forward = rel
            if forward <= 0:
                continue
            wx, wy = self._body_to_world(cam, yaw, right, forward)
            obs.append({ "x": wx, "y": wy, "rng": math.hypot(right, forward), "r": float(d.radius) })

        # 2) Associate observations to EXISTING tracks, greedily by distance.
        pairs = []
        for oi, o in enumerate(obs):
            for ti, t in enumerate(self.tracks):
                dist = math.hypot(o["x"] - t.x, o["y"] - t.y)
                if dist <= self.assoc_gate:
                    pairs.append((dist, oi, ti))

        pairs.sort(key=lambda p: p[0])
        matched_o, matched_t = set(), set()
        for dist, oi, ti in pairs:
            if oi in matched_o or ti in matched_t:
                continue
            matched_o.add(oi)
            matched_t.add(ti)
            t, o = self.tracks[ti], obs[oi]
            a = self.ema
            t.x = (1 - a) * t.x + a * o["x"]
            t.y = (1 - a) * t.y + a * o["y"]
            t.hits += 1
            t.missed_visible = 0
            t.radius_px = o["r"]
            t.range_at_obs = o["rng"]
            if t.hits >= self.confirm_hits:
                t.confirmed = True

        # 3) Negative evidence: unmatched EXISTING tracks that should be visible.
        survivors = []
        for ti, t in enumerate(self.tracks):
            if ti in matched_t:
                survivors.append(t)
                continue
            right, forward = self._world_to_body(cam, yaw, t.x, t.y)
            in_fov, _ = self._visible(right, forward)

            if in_fov:
                t.missed_visible += 1
                limit = self.max_missed if t.confirmed else self.tentative_max_missed
                if t.missed_visible > limit:
                    continue   # delete: looked right at it and it was not there
            survivors.append(t)   # out of view -> keep as part of the map

        self.tracks = survivors

        # 4) Spawn tentative tracks for unmatched observations.
        for oi, o in enumerate(obs):
            if oi in matched_o:
                continue
            self.tracks.append(BallTrack(
                id=self._next_id, x=o["x"], y=o["y"],
                radius_px=o["r"], range_at_obs=o["rng"]))
            self._next_id += 1

        return self.overlay_boxes(robot_pose)

    # ------------------------------------------------------------------
    # Outputs
    # ------------------------------------------------------------------
    def confirmed_positions(self):
        """All confirmed ball positions in world coords (the map)."""
        return [(t.x, t.y) for t in self.tracks if t.confirmed]

    def overlay_boxes(self, robot_pose):
        """Reproject confirmed tracks into the current frame for drawing.

        Returns list of (BBox, (world_x, world_y)) for tracks inside the image.
        """
        robot_x, robot_y, yaw = robot_pose
        cam = self._camera_origin(robot_x, robot_y, yaw)
        out = []

        for t in self.tracks:
            if not t.confirmed:
                continue
            right, forward = self._world_to_body(cam, yaw, t.x, t.y)
            in_fov, uv = self._visible(right, forward)

            if not in_fov:
                out.append((None, (t.x, t.y)))

            else:
                u_c, v_c = uv                       # ground-contact pixel
                rng_now = max(math.hypot(right, forward), 1e-6)
                r = int(round(max(1.0, t.radius_px * (t.range_at_obs / rng_now))))

                box = BBox(x=int(u_c - r), y=int(v_c - 2 * r), w=2 * r, h=2 * r,
                        cx=int(u_c), cy=int(v_c - r), radius=r)
                out.append((box, (t.x, t.y)))

        return out
